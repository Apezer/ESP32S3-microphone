/*
 * ESP32-S3 + INMP441 无线麦克风 (短录音 WAV + OLED 电平显示)
 *
 * 功能:
 *   - I2S 采集 INMP441 麦克风音频 (16kHz/16bit)
 *   - ESP32-S3 开启 WiFi AP 和网页服务
 *   - 浏览器点击录音按钮后获取 WAV 文件并播放
 *   - OLED 实时显示 RMS、峰值、dBFS 和录音状态
 *
 * 接线:
 *   OLED SDA -> GPIO1   OLED VCC -> 3.3V   OLED GND -> GND
 *   OLED SCL -> GPIO2
 *
 *   INMP441 SCK -> GPIO36   INMP441 VDD -> 3.3V
 *   INMP441 WS  -> GPIO37   INMP441 GND -> GND
 *   INMP441 SD  -> GPIO35   INMP441 L/R -> GND
 *
 * 使用:
 *   1. 上传固件
 *   2. 连接 WiFi: ESP32S3-MIC / 12345678
 *   3. 浏览器打开 http://192.168.4.1
 *   4. 使用预设时长录音，或点击开始/结束进行自定义录音
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include <math.h>

// ===== OLED 配置 =====
#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64
#define OLED_RESET         -1
#define OLED_ADDR          0x3C
#define I2C_SDA            1
#define I2C_SCL            2

// ===== INMP441 引脚 =====
#define MIC_SD_PIN         35
#define MIC_WS_PIN         37
#define MIC_SCK_PIN        36

// ===== 音频参数 =====
#define SAMPLE_RATE        16000
#define DMA_BUF_COUNT      8
#define DMA_BUF_LEN        256
#define READ_SAMPLES       512
#define VOICE_THRESH_DB    -38.0f
#define LEVEL_MIN_DB       -35.0f
#define LEVEL_MAX_DB       -5.0f

// ===== WiFi 配置 =====
#define WIFI_SSID          "ESP32S3-MIC"
#define WIFI_PASS          "12345678"

// ===== 全局对象 =====
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

// ===== 音频状态 =====
static int32_t raw_samples[READ_SAMPLES];
static int16_t pcm_samples[READ_SAMPLES];

static float last_rms = 0.0f;
static float last_db = -90.0f;
static int16_t last_peak = 0;
static bool voice_active = false;
static bool oled_ok = false;
static bool http_capture_active = false;
static uint32_t frame_count = 0;
static uint32_t last_ui_ms = 0;

// ===== OLED 启动画面 =====
void showBootScreen(const char *line1, const char *line2) {
    if (!oled_ok) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("ESP32-S3 MIC");
    oled.drawLine(0, 11, 127, 11, SSD1306_WHITE);
    oled.setCursor(0, 22);
    oled.println(line1);
    oled.setCursor(0, 38);
    oled.println(line2);
    oled.display();
}

// ===== OLED 初始化 =====
bool initOled() {
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        return false;
    }

    oled_ok = true;
    showBootScreen("OLED ready", "Init I2S mic...");
    return true;
}

// ===== I2S 麦克风初始化 =====
bool initMicrophone() {
    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate = SAMPLE_RATE;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
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

// ===== 麦克风读取 =====
bool readMicFrame(size_t *samples_read) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_0,
                             raw_samples,
                             sizeof(raw_samples),
                             &bytes_read,
                             pdMS_TO_TICKS(100));

    if (err != ESP_OK || bytes_read == 0) {
        *samples_read = 0;
        return false;
    }

    size_t samples = bytes_read / sizeof(raw_samples[0]);
    for (size_t i = 0; i < samples; i++) {
        // INMP441 的有效 24bit 数据放在 32bit I2S 槽内，这里缩放成 PCM16。
        int32_t sample = raw_samples[i] >> 14;
        if (sample > INT16_MAX) sample = INT16_MAX;
        if (sample < INT16_MIN) sample = INT16_MIN;
        pcm_samples[i] = (int16_t)sample;
    }

    *samples_read = samples;
    return true;
}

// ===== 音频电平分析 =====
void analyzeFrame(const int16_t *data, size_t samples) {
    if (samples == 0) {
        last_rms = 0;
        last_db = -90;
        last_peak = 0;
        voice_active = false;
        return;
    }

    double sum_sq = 0.0;
    int16_t peak = 0;

    for (size_t i = 0; i < samples; i++) {
        int32_t value = data[i];
        int16_t magnitude = abs(value);
        if (magnitude > peak) peak = magnitude;
        sum_sq += (double)value * (double)value;
    }

    last_rms = sqrt(sum_sq / samples);
    last_peak = peak;

    float normalized = last_rms / 32768.0f;
    last_db = normalized > 0.000001f ? 20.0f * log10f(normalized) : -90.0f;
    voice_active = last_db > VOICE_THRESH_DB;
    frame_count++;
}

// ===== OLED 电平显示 =====
void drawBar(int x, int y, int w, int h, float db) {
    float level = (db - LEVEL_MIN_DB) / (LEVEL_MAX_DB - LEVEL_MIN_DB);
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    level = level * level;

    int fill = (int)((w - 2) * level);
    oled.drawRect(x, y, w, h, SSD1306_WHITE);
    if (fill > 0) {
        oled.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
    }
}

void updateOled() {
    if (!oled_ok) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    oled.setCursor(0, 0);
    oled.print(http_capture_active ? "WiFi WAV capture" : "INMP441 16k I2S");

    oled.setCursor(0, 14);
    if (http_capture_active) {
        oled.print("Recording...");
    } else {
        oled.print("State: ");
        oled.print(voice_active ? "VOICE" : "quiet");
    }

    oled.setCursor(0, 26);
    oled.print("RMS: ");
    oled.print(last_rms, 0);

    oled.setCursor(72, 26);
    oled.print("Pk:");
    oled.print(last_peak);

    oled.setCursor(0, 38);
    oled.print("dBFS: ");
    oled.print(last_db, 1);

    drawBar(0, 52, 128, 10, last_db);
    oled.display();
}

// ===== WAV 文件头 =====
void writeWavHeader(WiFiClient &client, uint32_t data_size) {
    uint8_t header[44] = {};
    uint32_t riff_size = data_size + 36;
    uint16_t audio_format = 1;
    uint16_t channels = 1;
    uint32_t sample_rate = SAMPLE_RATE;
    uint16_t bits_per_sample = 16;
    uint32_t byte_rate = sample_rate * channels * bits_per_sample / 8;
    uint16_t block_align = channels * bits_per_sample / 8;
    uint32_t fmt_size = 16;

    memcpy(header + 0, "RIFF", 4);
    memcpy(header + 4, &riff_size, 4);
    memcpy(header + 8, "WAVE", 4);
    memcpy(header + 12, "fmt ", 4);
    memcpy(header + 16, &fmt_size, 4);
    memcpy(header + 20, &audio_format, 2);
    memcpy(header + 22, &channels, 2);
    memcpy(header + 24, &sample_rate, 4);
    memcpy(header + 28, &byte_rate, 4);
    memcpy(header + 32, &block_align, 2);
    memcpy(header + 34, &bits_per_sample, 2);
    memcpy(header + 36, "data", 4);
    memcpy(header + 40, &data_size, 4);

    client.write(header, sizeof(header));
}

// ===== HTML 客户端页面 =====
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-S3 无线麦克风</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    min-height: 100vh;
    display: flex; justify-content: center; align-items: center;
    padding: 18px;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    background: #0f172a; color: #e2e8f0;
  }
  .card {
    width: min(390px, 100%);
    background: #1e293b;
    border-radius: 16px;
    padding: 28px;
    text-align: center;
    box-shadow: 0 8px 32px rgba(0,0,0,0.4);
  }
  h1 { font-size: 20px; margin-bottom: 8px; color: #38bdf8; }
  .sub { font-size: 13px; color: #94a3b8; margin-bottom: 20px; line-height: 1.6; }
  #status {
    display: inline-block;
    padding: 6px 16px;
    border-radius: 20px;
    font-size: 13px;
    margin-bottom: 18px;
    background: #334155;
    color: #94a3b8;
  }
  #status.voice { background: #064e3b; color: #34d399; }
  #status.recording { background: #7f1d1d; color: #fca5a5; animation: pulse 1s infinite; }
  #status.error { background: #7f1d1d; color: #fca5a5; }
  @keyframes pulse { 50% { opacity: 0.7; } }
  .meter-wrap {
    margin: 0 0 18px;
    height: 24px;
    overflow: hidden;
    position: relative;
    border-radius: 8px;
    background: #0f172a;
  }
  .meter-bar {
    height: 100%;
    width: 0%;
    transition: width 0.08s;
    border-radius: 8px;
    background: linear-gradient(90deg, #22c55e, #eab308, #ef4444);
  }
  .stats {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 8px;
    margin-bottom: 18px;
  }
  .stat {
    background: #0f172a;
    border: 1px solid #334155;
    border-radius: 10px;
    padding: 10px 6px;
  }
  .stat span {
    display: block;
    color: #64748b;
    font-size: 11px;
    margin-bottom: 4px;
  }
  .stat b { display: block; color: #e2e8f0; font-size: 16px; }
  .btn-row {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 10px;
    margin-bottom: 10px;
  }
  button, a.btn {
    width: 100%;
    border: 0;
    border-radius: 10px;
    padding: 13px 10px;
    color: #fff;
    background: #2563eb;
    font-size: 15px;
    cursor: pointer;
    text-decoration: none;
    display: inline-block;
  }
  button:hover, a.btn:hover { background: #1d4ed8; }
  button.recording { background: #dc2626; animation: pulse 1s infinite; }
  .custom-row { margin-bottom: 10px; }
  button.stop { background: #dc2626; animation: pulse 1s infinite; }
  .rec-info { font-size: 12px; color: #f59e0b; min-height: 18px; margin: 8px 0; }
  audio { width: 100%; margin: 10px 0 12px; filter: invert(1) hue-rotate(180deg); }
  .download { margin-bottom: 14px; }
  .info { color: #64748b; font-size: 11px; line-height: 1.7; }
</style>
</head>
<body>
<div class="card">
  <h1>ESP32-S3 无线麦克风</h1>
  <p class="sub">INMP441 I2S &rarr; WiFi WAV &rarr; 浏览器播放</p>

  <div id="status">就绪</div>
  <div class="meter-wrap"><div class="meter-bar" id="meter"></div></div>

  <div class="stats">
    <div class="stat"><span>RMS</span><b id="rms">0</b></div>
    <div class="stat"><span>PEAK</span><b id="peak">0</b></div>
    <div class="stat"><span>dBFS</span><b id="db">-90</b></div>
  </div>

  <div class="btn-row">
    <button onclick="record(3)">3 秒</button>
    <button onclick="record(5)">5 秒</button>
    <button onclick="record(10)">10 秒</button>
  </div>

  <div class="custom-row">
    <button id="customBtn" onclick="toggleCustomRecord()">开始自定义录音</button>
  </div>

  <div class="rec-info" id="hint">选择录制时长后开始说话</div>
  <audio id="player" controls></audio>
  <a class="btn download" id="download" href="/capture.wav?seconds=5">下载 WAV</a>

  <p class="info">
    采样率: 16000 Hz | 单声道 16-bit<br>
    连接 WiFi: <b>ESP32S3-MIC</b> / 12345678<br>
    地址: http://192.168.4.1
  </p>
</div>

<script>
let recordingUntil = 0;
let customRecording = false;
let customAbort = null;
let customChunks = [];
let customBytes = 0;
let customStartMs = 0;
let customTimer = null;

function pct(db) {
  const minDb = -35;
  const maxDb = -5;
  const level = Math.max(0, Math.min(1, (db - minDb) / (maxDb - minDb)));
  return level * level * 100;
}

async function refresh() {
  if (customRecording || Date.now() < recordingUntil) {
    return;
  }

  try {
    const r = await fetch('/status.json', { cache: 'no-store' });
    const s = await r.json();
    const status = document.getElementById('status');
    status.textContent = s.capturing ? '录音中' : (s.voice ? '检测到声音' : '就绪');
    status.className = s.capturing ? 'recording' : (s.voice ? 'voice' : '');
    document.getElementById('rms').textContent = Math.round(s.rms);
    document.getElementById('peak').textContent = s.peak;
    document.getElementById('db').textContent = s.dbfs.toFixed(1);
    document.getElementById('meter').style.width = pct(s.dbfs) + '%';
  } catch (e) {
    const status = document.getElementById('status');
    status.textContent = '离线';
    status.className = 'error';
  }
}

async function record(sec) {
  if (customRecording) return;
  sec = Math.max(1, Math.min(30, Number(sec) || 5));
  const player = document.getElementById('player');
  const buttons = document.querySelectorAll('button');
  const status = document.getElementById('status');
  let chunks = [];
  let totalBytes = 0;
  let elapsedMs = 0;

  recordingUntil = Date.now() + sec * 1000 + 1200;
  status.textContent = '录音中';
  status.className = 'recording';
  document.getElementById('hint').textContent = '录音中 ' + sec + ' 秒... 请保持页面打开';
  buttons.forEach(btn => {
    btn.disabled = true;
    btn.classList.add('recording');
  });
  try {
    while (elapsedMs < sec * 1000) {
      const chunkMs = Math.min(500, sec * 1000 - elapsedMs);
      const resp = await fetch('/chunk.pcm?ms=' + chunkMs + '&t=' + Date.now());
      if (!resp.ok) throw new Error('chunk request failed');
      const data = new Uint8Array(await resp.arrayBuffer());
      chunks.push(data);
      totalBytes += data.byteLength;
      elapsedMs += chunkMs;
      document.getElementById('hint').textContent = '录音中 ' + (elapsedMs / 1000).toFixed(1) + ' / ' + sec + ' 秒';
    }

    if (totalBytes === 0) {
      document.getElementById('hint').textContent = '没有收到录音数据';
      return;
    }

    const pcm = mergeChunks(chunks, totalBytes);
    const wav = encodeWav(pcm, 16000);
    const blob = new Blob([wav], { type: 'audio/wav' });
    const url = URL.createObjectURL(blob);
    const download = document.getElementById('download');
    player.src = url;
    player.load();
    download.href = url;
    download.download = 'inmp441_' + sec + 's_' + Date.now() + '.wav';
    document.getElementById('hint').textContent = '录音完成，可以播放或下载 WAV';
    player.play().catch(() => {});
  } catch (e) {
    document.getElementById('hint').textContent = '录音失败，请重新点击录制';
    status.textContent = '错误';
    status.className = 'error';
  } finally {
    recordingUntil = 0;
    buttons.forEach(btn => {
      btn.disabled = false;
      btn.classList.remove('recording');
    });
  }
}

function toggleCustomRecord() {
  if (customRecording) {
    stopCustomRecord();
  } else {
    startCustomRecord();
  }
}

async function startCustomRecord() {
  customRecording = true;
  customAbort = new AbortController();
  customChunks = [];
  customBytes = 0;
  customStartMs = Date.now();

  const status = document.getElementById('status');
  const customBtn = document.getElementById('customBtn');
  const presetButtons = document.querySelectorAll('.btn-row button');
  status.textContent = '录音中';
  status.className = 'recording';
  customBtn.textContent = '结束录音';
  customBtn.className = 'stop';
  presetButtons.forEach(btn => btn.disabled = true);

  updateCustomInfo();
  customTimer = setInterval(updateCustomInfo, 300);

  try {
    while (customRecording) {
      const resp = await fetch('/chunk.pcm?ms=500&t=' + Date.now(), { signal: customAbort.signal });
      if (!resp.ok) throw new Error('chunk request failed');
      const data = new Uint8Array(await resp.arrayBuffer());
      if (data.byteLength > 0) {
        customChunks.push(data);
        customBytes += data.byteLength;
      }
    }
  } catch (e) {
    if (customRecording) {
      document.getElementById('hint').textContent = '录音连接中断';
      finishCustomRecord();
    }
  } finally {
    if (customRecording) {
      finishCustomRecord();
    }
  }
}

function stopCustomRecord() {
  customRecording = false;
  if (customAbort) {
    customAbort.abort();
  }
  finishCustomRecord();
}

function finishCustomRecord() {
  customRecording = false;
  clearInterval(customTimer);
  customTimer = null;

  const customBtn = document.getElementById('customBtn');
  const presetButtons = document.querySelectorAll('.btn-row button');
  customBtn.textContent = '开始自定义录音';
  customBtn.className = '';
  presetButtons.forEach(btn => btn.disabled = false);
  recordingUntil = 0;

  if (customBytes === 0) {
    document.getElementById('hint').textContent = '没有收到录音数据';
    return;
  }

  const pcm = mergeChunks(customChunks, customBytes);
  const wav = encodeWav(pcm, 16000);
  const blob = new Blob([wav], { type: 'audio/wav' });
  const url = URL.createObjectURL(blob);
  const player = document.getElementById('player');
  const download = document.getElementById('download');
  const sec = ((Date.now() - customStartMs) / 1000).toFixed(1);
  player.src = url;
  player.load();
  download.href = url;
  download.download = 'inmp441_custom_' + Date.now() + '.wav';
  document.getElementById('hint').textContent = '自定义录音完成: ' + sec + ' 秒，可以播放或下载 WAV';
  player.play().catch(() => {});
}

function updateCustomInfo() {
  const sec = ((Date.now() - customStartMs) / 1000).toFixed(1);
  const kb = (customBytes / 1024).toFixed(1);
  document.getElementById('hint').textContent = '自定义录音中 ' + sec + ' 秒 | ' + kb + ' KB';
}

function mergeChunks(chunks, totalBytes) {
  const pcm = new Uint8Array(totalBytes);
  let offset = 0;
  for (const chunk of chunks) {
    pcm.set(chunk, offset);
    offset += chunk.byteLength;
  }
  return pcm;
}

function encodeWav(pcmBytes, sampleRate) {
  const dataSize = pcmBytes.byteLength;
  const buffer = new ArrayBuffer(44 + dataSize);
  const view = new DataView(buffer);
  writeString(view, 0, 'RIFF');
  view.setUint32(4, 36 + dataSize, true);
  writeString(view, 8, 'WAVE');
  writeString(view, 12, 'fmt ');
  view.setUint32(16, 16, true);
  view.setUint16(20, 1, true);
  view.setUint16(22, 1, true);
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate * 2, true);
  view.setUint16(32, 2, true);
  view.setUint16(34, 16, true);
  writeString(view, 36, 'data');
  view.setUint32(40, dataSize, true);
  new Uint8Array(buffer, 44).set(pcmBytes);
  return buffer;
}

function writeString(view, offset, text) {
  for (let i = 0; i < text.length; i++) {
    view.setUint8(offset + i, text.charCodeAt(i));
  }
}

setInterval(refresh, 350);
refresh();
</script>
</body>
</html>
)rawliteral";

// ===== HTTP 请求处理 =====
void handleRoot() {
    server.send_P(200, "text/html", HTML_PAGE);
}

void handleStatusJson() {
    String json = "{";
    json += "\"state\":\"";
    json += http_capture_active ? "RECORDING" : (voice_active ? "VOICE" : "READY");
    json += "\",\"capturing\":";
    json += http_capture_active ? "true" : "false";
    json += ",\"voice\":";
    json += voice_active ? "true" : "false";
    json += ",\"rms\":";
    json += String(last_rms, 1);
    json += ",\"peak\":";
    json += last_peak;
    json += ",\"dbfs\":";
    json += String(last_db, 1);
    json += ",\"frames\":";
    json += frame_count;
    json += ",\"sampleRate\":";
    json += SAMPLE_RATE;
    json += "}";

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

void handleCaptureWav() {
    int seconds = 5;
    if (server.hasArg("seconds")) {
        seconds = server.arg("seconds").toInt();
    }
    seconds = constrain(seconds, 1, 30);

    uint32_t total_samples = (uint32_t)SAMPLE_RATE * seconds;
    uint32_t data_size = total_samples * sizeof(int16_t);
    uint32_t content_length = data_size + 44;

    http_capture_active = true;
    updateOled();

    server.sendHeader("Content-Disposition", "inline; filename=\"inmp441_capture.wav\"");
    server.setContentLength(content_length);
    server.send(200, "audio/wav", "");

    WiFiClient client = server.client();
    writeWavHeader(client, data_size);

    uint32_t samples_sent = 0;
    while (client.connected() && samples_sent < total_samples) {
        size_t samples_read = 0;
        if (!readMicFrame(&samples_read) || samples_read == 0) {
            delay(1);
            continue;
        }

        uint32_t remaining = total_samples - samples_sent;
        if (samples_read > remaining) {
            samples_read = remaining;
        }

        analyzeFrame(pcm_samples, samples_read);
        client.write((const uint8_t *)pcm_samples, samples_read * sizeof(int16_t));
        samples_sent += samples_read;

        if (millis() - last_ui_ms >= 120) {
            last_ui_ms = millis();
            updateOled();
        }

        yield();
    }

    http_capture_active = false;
}

void handlePcmChunk() {
    int ms = 500;
    if (server.hasArg("ms")) {
        ms = server.arg("ms").toInt();
    }
    ms = constrain(ms, 100, 1000);

    uint32_t total_samples = ((uint32_t)SAMPLE_RATE * (uint32_t)ms) / 1000;
    uint32_t data_size = total_samples * sizeof(int16_t);

    http_capture_active = true;
    updateOled();

    server.sendHeader("Cache-Control", "no-store");
    server.setContentLength(data_size);
    server.send(200, "application/octet-stream", "");

    WiFiClient client = server.client();
    uint32_t samples_sent = 0;
    while (client.connected() && samples_sent < total_samples) {
        size_t samples_read = 0;
        if (!readMicFrame(&samples_read) || samples_read == 0) {
            delay(1);
            continue;
        }

        uint32_t remaining = total_samples - samples_sent;
        if (samples_read > remaining) {
            samples_read = remaining;
        }

        analyzeFrame(pcm_samples, samples_read);
        client.write((const uint8_t *)pcm_samples, samples_read * sizeof(int16_t));
        samples_sent += samples_read;

        if (millis() - last_ui_ms >= 120) {
            last_ui_ms = millis();
            updateOled();
        }

        yield();
    }

    http_capture_active = false;
}

// ===== WiFi 和 Web 服务器 =====
void initWifiServer() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status.json", HTTP_GET, handleStatusJson);
    server.on("/capture.wav", HTTP_GET, handleCaptureWav);
    server.on("/chunk.pcm", HTTP_GET, handlePcmChunk);
    server.begin();
}

// ===== setup =====
void setup() {
    if (!initOled()) {
        // OLED 初始化失败，继续运行
    }

    if (!initMicrophone()) {
        showBootScreen("I2S INIT FAILED", "Check wiring");
        while (1) delay(1000);
    }

    initWifiServer();
    showBootScreen("WiFi: ESP32S3-MIC", WiFi.softAPIP().toString().c_str());
    delay(800);
}

// ===== loop =====
void loop() {
    server.handleClient();

    size_t samples_read = 0;
    if (readMicFrame(&samples_read)) {
        analyzeFrame(pcm_samples, samples_read);
    }

    if (millis() - last_ui_ms >= 120) {
        last_ui_ms = millis();
        updateOled();
    }
}
