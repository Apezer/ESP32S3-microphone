/*
 * ESP32-S3 + INMP441 无线麦克风 (带 OLED 电平显示)
 *
 * 功能:
 *   - I2S 采集 INMP441 麦克风音频 (16kHz/16bit)
 *   - WiFi AP + WebSocket 实时推流到浏览器
 *   - 浏览器端 Web Audio API 播放
 *   - OLED 实时显示音频电平
 *
 * 接线:
 *   OLED SDA -> GPIO1   OLED VCC -> 3.3V   OLED GND -> GND
 *   OLED SCL -> GPIO2
 *
 *   INMP441 SCK -> GPIO36   INMP441 VDD -> 3.3V
 *   INMP441 WS -> GPIO37    INMP441 GND -> GND
 *   INMP441 SD -> GPIO35    INMP441 L/R -> GND
 *
 * 使用:
 *   1. 上传固件
 *   2. 连接 WiFi: ESP32-Mic / 12345678
 *   3. 浏览器打开 http://192.168.4.1
 *   4. 点击「开始收听」
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <driver/i2s.h>

// ===== OLED 配置 =====
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
#define I2C_SDA       1
#define I2C_SCL       2

// ===== INMP441 引脚 =====
#define MIC_SD_PIN    35
#define MIC_WS_PIN    37
#define MIC_SCK_PIN   36

// ===== 音频参数 =====
#define SAMPLE_RATE   16000
#define DMA_BUF_COUNT 8
#define DMA_BUF_LEN   256

// ===== WiFi 配置 =====
#define WIFI_SSID     "ESP32-Mic"
#define WIFI_PASS     "12345678"
#define WIFI_CHANNEL  6
#define MAX_CLIENTS   4

// ===== 全局对象 =====
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 音频
static int16_t audio_buffer[DMA_BUF_LEN];
static int16_t peak_value = 0;
static int16_t rms_value = 0;

// ===== 音频处理链 (参考小智 AFE 处理思路) =====

// 1. DC 偏移去除 (高通, 截止 ~20Hz)
static float dc_x1 = 0, dc_y1 = 0;

// 2. 二阶带通滤波器 (Butterworth, 300Hz~3.4kHz)
//    由高通(300Hz) + 低通(3.4kHz) 串联组成
// 高通 300Hz
static float hp_x1 = 0, hp_x2 = 0, hp_y1 = 0, hp_y2 = 0;
// 低通 3.4kHz
static float lp_x1 = 0, lp_x2 = 0, lp_y1 = 0, lp_y2 = 0;

// 3. 滞回噪声门限 (防止门限附近咔嗒声)
static bool gate_open = false;
static const int16_t GATE_OPEN_THRESH  = 400;   // 开门阈值 (需要更大信号才开门)
static const int16_t GATE_CLOSE_THRESH = 200;   // 关门阈值 (更小信号才关门)
static const int16_t GATE_OUTPUT = 0;            // 门关闭时输出值

// DC 偏移去除 (一阶高通 IIR, fc=20Hz)
// y[n] = 0.999 * (y[n-1] + x[n] - x[n-1])
inline int16_t removeDC(int16_t x) {
    float xf = (float)x;
    float y = 0.999f * (dc_y1 + xf - dc_x1);
    dc_x1 = xf;
    dc_y1 = y;
    return (int16_t)y;
}

// 二阶 Butterworth 高通, fc=300Hz, fs=16kHz
// 系数由 MATLAB/Python butter(2, 300/8000, 'high') 计算
inline int16_t highpass300(int16_t x) {
    static const float b0 = 0.8631f, b1 = -1.7262f, b2 = 0.8631f;
    static const float a1 = -1.7237f, a2 = 0.7287f;
    float xf = (float)x;
    float y = b0*xf + b1*hp_x1 + b2*hp_x2 - a1*hp_y1 - a2*hp_y2;
    hp_x2 = hp_x1; hp_x1 = xf;
    hp_y2 = hp_y1; hp_y1 = y;
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    return (int16_t)y;
}

// 二阶 Butterworth 低通, fc=3.4kHz, fs=16kHz
inline int16_t lowpass3400(int16_t x) {
    static const float b0 = 0.1341f, b1 = 0.2682f, b2 = 0.1341f;
    static const float a1 = -0.7265f, a2 = 0.2630f;
    float xf = (float)x;
    float y = b0*xf + b1*lp_x1 + b2*lp_x2 - a1*lp_y1 - a2*lp_y2;
    lp_x2 = lp_x1; lp_x1 = xf;
    lp_y2 = lp_y1; lp_y1 = y;
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    return (int16_t)y;
}

// 完整音频处理链
inline int16_t processSample(int16_t x) {
    // 1. 去除 DC 偏移
    x = removeDC(x);
    // 2. 高通 300Hz (去除低频嗡嗡声)
    x = highpass300(x);
    // 3. 低通 3.4kHz (去除高频电流声)
    x = lowpass3400(x);
    // 4. 滞回噪声门限
    int16_t absx = abs(x);
    if (!gate_open && absx > GATE_OPEN_THRESH) {
        gate_open = true;
    } else if (gate_open && absx < GATE_CLOSE_THRESH) {
        gate_open = false;
    }
    return gate_open ? x : GATE_OUTPUT;
}

// ===== I2S 麦克风初始化 =====
bool initMicrophone() {
    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate = SAMPLE_RATE;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = DMA_BUF_COUNT;
    i2s_config.dma_buf_len = DMA_BUF_LEN;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = false;
    i2s_config.fixed_mclk = 0;

    i2s_pin_config_t pin_config = {};
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
    pin_config.bck_io_num = MIC_SCK_PIN;
    pin_config.ws_io_num = MIC_WS_PIN;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num = MIC_SD_PIN;

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
    if (err != ESP_OK) return false;

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) return false;

    i2s_zero_dma_buffer(I2S_NUM_0);
    return true;
}

// ===== OLED 电平显示 (诊断页面 2) =====
void drawBar(int x, int y, int w, int h, int level, int max_level) {
    oled.drawRect(x, y, w, h, SSD1306_WHITE);
    int fill = (level * (w - 2)) / max_level;
    if (fill > w - 2) fill = w - 2;
    if (fill > 0) {
        oled.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
    }
}

void updateOled() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    // 标题
    oled.setCursor(0, 0);
    oled.print("Audio Level");
    oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

    // 峰值电平条
    int bar_y = 14;
    int bar_h = 14;
    drawBar(4, bar_y, 90, bar_h, peak_value, 16000);

    // 峰值数字
    oled.setCursor(98, bar_y + 3);
    oled.printf("%5d", peak_value);

    // RMS 电平条
    int rms_y = bar_y + bar_h + 4;
    drawBar(4, rms_y, 90, 10, rms_value, 8000);
    oled.setCursor(98, rms_y + 1);
    oled.print("RMS");

    // 底部状态
    oled.setCursor(0, 56);
    oled.printf("Clients:%d  16kHz", ws.count());

    oled.display();
}

// ===== HTML 客户端页面 =====
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-S3 无线麦克风</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, sans-serif;
    background: #0f172a; color: #e2e8f0;
    display: flex; justify-content: center; align-items: center;
    min-height: 100vh;
  }
  .card {
    background: #1e293b; border-radius: 16px; padding: 32px;
    width: 360px; text-align: center;
    box-shadow: 0 8px 32px rgba(0,0,0,0.4);
  }
  h1 { font-size: 20px; margin-bottom: 8px; color: #38bdf8; }
  .sub { font-size: 13px; color: #64748b; margin-bottom: 24px; }
  #status {
    display: inline-block; padding: 6px 16px; border-radius: 20px;
    font-size: 13px; margin-bottom: 20px;
    background: #334155; color: #94a3b8;
  }
  #status.connected { background: #064e3b; color: #34d399; }
  #status.error { background: #7f1d1d; color: #fca5a5; }
  button {
    background: #2563eb; color: #fff; border: none; padding: 14px 32px;
    border-radius: 10px; font-size: 16px; cursor: pointer;
    width: 100%; transition: background 0.2s;
  }
  button:hover { background: #1d4ed8; }
  button.rec { background: #16a34a; }
  button.rec:hover { background: #15803d; }
  button.recording { background: #dc2626; animation: pulse 1s infinite; }
  @keyframes pulse { 50% { opacity: 0.7; } }
  .btn-row { display: flex; gap: 10px; margin-bottom: 8px; }
  .btn-row button { flex: 1; }
  .rec-info { font-size: 12px; color: #f59e0b; min-height: 18px; margin-bottom: 8px; }
  .meter-wrap {
    margin: 20px 0; background: #0f172a; border-radius: 8px;
    height: 24px; overflow: hidden; position: relative;
  }
  .meter-bar {
    height: 100%; width: 0%; transition: width 0.05s;
    background: linear-gradient(90deg, #22c55e, #eab308, #ef4444);
    border-radius: 8px;
  }
  .info {
    font-size: 11px; color: #475569; margin-top: 16px; line-height: 1.6;
  }
  .gain-ctrl { margin: 16px 0; display: flex; align-items: center; gap: 10px; }
  .gain-ctrl label { font-size: 13px; color: #94a3b8; white-space: nowrap; }
  .gain-ctrl input[type=range] { flex: 1; }
  .gain-val { font-size: 13px; color: #38bdf8; min-width: 40px; }
</style>
</head>
<body>
<div class="card">
  <h1>ESP32-S3 无线麦克风</h1>
  <p class="sub">INMP441 I2S &rarr; WebSocket &rarr; 浏览器播放</p>
  <div id="status">未连接</div>
  <div class="meter-wrap"><div class="meter-bar" id="meter"></div></div>
  <div class="gain-ctrl">
    <label>增益</label>
    <input type="range" id="gain" min="1" max="20" value="5">
    <span class="gain-val" id="gainVal">5x</span>
  </div>
  <div class="btn-row">
    <button id="btn" onclick="toggle()">开始收听</button>
    <button id="recBtn" class="rec" onclick="toggleRec()" disabled>录音</button>
  </div>
  <div class="rec-info" id="recInfo"></div>
  <p class="info">
    采样率: 16000 Hz | 单声道 16-bit<br>
    连接 WiFi: <b>ESP32-Mic</b> / 12345678
  </p>
</div>
<script>
let ws, audioCtx, playing = false;
let gainNode, meterEl, gainSlider, gainValEl;

// 录音相关
let recording = false;
let recChunks = [];
let recStartTime = 0;
let recTimer = null;

function initAudio() {
  audioCtx = new (window.AudioContext || window.webkitAudioContext)({ sampleRate: 16000 });
  gainNode = audioCtx.createGain();
  gainNode.connect(audioCtx.destination);
  gainNode.gain.value = 5;
}

function playSamples(int16arr) {
  if (!audioCtx) return;
  // 录音时保存原始 PCM 数据
  if (recording) {
    recChunks.push(new Int16Array(int16arr));
  }
  const float32 = new Float32Array(int16arr.length);
  for (let i = 0; i < int16arr.length; i++) {
    float32[i] = int16arr[i] / 32768.0;
  }
  const buf = audioCtx.createBuffer(1, float32.length, 16000);
  buf.getChannelData(0).set(float32);
  const src = audioCtx.createBufferSource();
  src.buffer = buf;
  src.connect(gainNode);
  src.start();

  let peak = 0;
  for (let i = 0; i < float32.length; i++) {
    const v = Math.abs(float32[i]);
    if (v > peak) peak = v;
  }
  const pct = Math.min(peak * 100 * gainNode.gain.value, 100);
  if (meterEl) meterEl.style.width = pct + '%';
}

function toggle() {
  if (playing) stop(); else start();
}

function start() {
  if (playing) return;
  initAudio();
  const btn = document.getElementById('btn');
  const st = document.getElementById('status');

  ws = new WebSocket('ws://' + location.host + '/ws', 'audio');
  ws.binaryType = 'arraybuffer';

  ws.onopen = () => {
    playing = true;
    btn.textContent = '停止收听';
    document.getElementById('recBtn').disabled = false;
    st.textContent = '已连接 - 正在传输';
    st.className = 'connected';
  };

  ws.onmessage = (e) => {
    if (e.data instanceof ArrayBuffer) {
      playSamples(new Int16Array(e.data));
    }
  };

  ws.onclose = () => {
    stop();
    st.textContent = '已断开';
    st.className = '';
  };

  ws.onerror = () => {
    st.textContent = '连接错误';
    st.className = 'error';
    stop();
  };
}

function stop() {
  if (recording) stopRec();
  playing = false;
  if (ws) { ws.close(); ws = null; }
  if (audioCtx) { audioCtx.close(); audioCtx = null; }
  document.getElementById('btn').textContent = '开始收听';
  document.getElementById('recBtn').disabled = true;
  const st = document.getElementById('status');
  st.textContent = '已停止';
  st.className = '';
  if (meterEl) meterEl.style.width = '0%';
}

// ===== 录音功能 =====
function toggleRec() {
  if (recording) stopRec(); else startRec();
}

function startRec() {
  recording = true;
  recChunks = [];
  recStartTime = Date.now();
  const btn = document.getElementById('recBtn');
  btn.textContent = '停止录音';
  btn.className = 'recording';
  updateRecInfo();
  recTimer = setInterval(updateRecInfo, 500);
}

function updateRecInfo() {
  const sec = ((Date.now() - recStartTime) / 1000).toFixed(1);
  const samples = recChunks.reduce((s, c) => s + c.length, 0);
  const sizeKB = (samples * 2 / 1024).toFixed(1);
  document.getElementById('recInfo').textContent =
    '录音中 ' + sec + 's | ' + sizeKB + ' KB';
}

function stopRec() {
  recording = false;
  clearInterval(recTimer);
  const btn = document.getElementById('recBtn');
  btn.textContent = '录音';
  btn.className = 'rec';

  if (recChunks.length === 0) {
    document.getElementById('recInfo').textContent = '无录音数据';
    return;
  }

  // 合并所有 PCM 块
  let totalLen = 0;
  for (const c of recChunks) totalLen += c.length;
  const pcm = new Int16Array(totalLen);
  let offset = 0;
  for (const c of recChunks) {
    pcm.set(c, offset);
    offset += c.length;
  }
  recChunks = [];

  // 生成 WAV 文件
  const wav = encodeWAV(pcm, 16000);
  const blob = new Blob([wav], { type: 'audio/wav' });
  const url = URL.createObjectURL(blob);

  // 下载
  const a = document.createElement('a');
  const sec = ((Date.now() - recStartTime) / 1000).toFixed(1);
  a.href = url;
  a.download = 'recording_' + Date.now() + '.wav';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);

  const sizeKB = (blob.size / 1024).toFixed(1);
  document.getElementById('recInfo').textContent =
    '已保存: ' + sec + 's, ' + sizeKB + ' KB';
}

function encodeWAV(samples, sampleRate) {
  const numChannels = 1;
  const bitsPerSample = 16;
  const byteRate = sampleRate * numChannels * bitsPerSample / 8;
  const blockAlign = numChannels * bitsPerSample / 8;
  const dataSize = samples.length * (bitsPerSample / 8);
  const buffer = new ArrayBuffer(44 + dataSize);
  const view = new DataView(buffer);

  // RIFF header
  writeString(view, 0, 'RIFF');
  view.setUint32(4, 36 + dataSize, true);
  writeString(view, 8, 'WAVE');

  // fmt chunk
  writeString(view, 12, 'fmt ');
  view.setUint32(16, 16, true);          // chunk size
  view.setUint16(20, 1, true);           // PCM format
  view.setUint16(22, numChannels, true);
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, byteRate, true);
  view.setUint16(32, blockAlign, true);
  view.setUint16(34, bitsPerSample, true);

  // data chunk
  writeString(view, 36, 'data');
  view.setUint32(40, dataSize, true);

  // PCM data
  let pos = 44;
  for (let i = 0; i < samples.length; i++) {
    view.setInt16(pos, samples[i], true);
    pos += 2;
  }

  return view;
}

function writeString(view, offset, str) {
  for (let i = 0; i < str.length; i++) {
    view.setUint8(offset + i, str.charCodeAt(i));
  }
}

window.onload = () => {
  meterEl = document.getElementById('meter');
  gainSlider = document.getElementById('gain');
  gainValEl = document.getElementById('gainVal');
  gainSlider.oninput = () => {
    if (gainNode) gainNode.gain.value = +gainSlider.value;
    gainValEl.textContent = gainSlider.value + 'x';
  };
};
</script>
</body>
</html>
)rawliteral";

// ===== WebSocket 事件处理 =====
void onWsEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client #%u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
}

// ===== setup =====
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32-S3 Wireless Mic + OLED ===");

    // 1. 初始化 OLED
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[FAIL] OLED init");
        while (1) delay(1000);
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(20, 25);
    oled.println("Starting...");
    oled.display();

    // 2. 初始化麦克风
    if (!initMicrophone()) {
        oled.clearDisplay();
        oled.setCursor(0, 25);
        oled.println("I2S INIT FAILED!");
        oled.display();
        while (1) delay(1000);
    }
    Serial.println("Microphone OK");

    // 3. 启动 WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, MAX_CLIENTS);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("WiFi: %s / %s\n", WIFI_SSID, WIFI_PASS);
    Serial.printf("Open http://%s\n", ip.toString().c_str());

    // 4. 启动 Web 服务器
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", HTML_PAGE);
    });
    server.begin();
    Serial.println("Server ready");

    // 5. OLED 显示就绪信息
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("WiFi: ESP32-Mic");
    oled.println("Pass: 12345678");
    oled.println();
    oled.print("IP: ");
    oled.println(ip);
    oled.println();
    oled.println("Waiting for client...");
    oled.display();
    delay(2000);
}

// ===== loop =====
void loop() {
    ws.cleanupClients();

    if (ws.count() > 0) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_read(I2S_NUM_0, audio_buffer,
                                 sizeof(audio_buffer), &bytes_read,
                                 pdMS_TO_TICKS(100));
        if (err == ESP_OK && bytes_read > 0) {
            int samples = bytes_read / sizeof(int16_t);

            // 计算峰值和 RMS
            int16_t peak = 0;
            int64_t sum_sq = 0;
            for (int i = 0; i < samples; i++) {
                int16_t v = abs(audio_buffer[i]);
                if (v > peak) peak = v;
                sum_sq += (int32_t)v * v;
            }
            peak_value = peak;
            rms_value = (int16_t)sqrt(sum_sq / samples);

            // 音频处理链: DC去除 → 带通滤波 → 噪声门限
            for (int i = 0; i < samples; i++) {
                audio_buffer[i] = processSample(audio_buffer[i]);
            }

            // WebSocket 推送
            ws.binaryAll((const char*)audio_buffer, bytes_read);

            // 更新 OLED
            updateOled();
        }
    } else {
        // 无客户端时降低占用
        delay(10);
    }
}
