/*
 * ESP32-S3 + INMP441 麦克风诊断工具
 *
 * 功能:
 *   - OLED 显示 I2S 初始化状态
 *   - 实时显示音频电平 (柱状图)
 *   - 显示采样数据 (峰值/RMS/是否收到数据)
 *   - 帮助判断接线是否正确
 *
 * 接线:
 *   OLED SDA -> GPIO1   OLED VCC -> 3.3V   OLED GND -> GND
 *   OLED SCL -> GPIO2
 *
 *   INMP441 SCK -> GPIO36   INMP441 VDD -> 3.3V
 *   INMP441 WS -> GPIO37    INMP441 GND -> GND
 *   INMP441 SD -> GPIO35    INMP441 L/R -> GND
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 诊断状态
bool i2s_ok = false;
bool mic_receiving = false;
int16_t peak_value = 0;
int16_t rms_value = 0;
int silent_count = 0;
int active_count = 0;
int sample_count = 0;

// ===== OLED 辅助函数 =====
void showHeader(const char* title) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(title);
    display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
}

// 画柱状图 (音频电平)
void drawBar(int x, int y, int w, int h, int level, int max_level) {
    display.drawRect(x, y, w, h, SSD1306_WHITE);
    int fill = (level * (w - 2)) / max_level;
    if (fill > w - 2) fill = w - 2;
    if (fill > 0) {
        display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
    }
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
    if (err != ESP_OK) {
        Serial.printf("[FAIL] i2s_driver_install: %d\n", err);
        return false;
    }

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[FAIL] i2s_set_pin: %d\n", err);
        return false;
    }

    i2s_zero_dma_buffer(I2S_NUM_0);
    Serial.println("[OK] I2S microphone initialized");
    return true;
}

// ===== 读取并分析一帧音频 =====
void analyzeAudio() {
    int16_t buf[DMA_BUF_LEN];
    size_t bytes_read = 0;

    esp_err_t err = i2s_read(I2S_NUM_0, buf, sizeof(buf), &bytes_read, pdMS_TO_TICKS(200));
    if (err != ESP_OK || bytes_read == 0) {
        return;
    }

    int samples = bytes_read / sizeof(int16_t);
    sample_count++;

    // 计算峰值和 RMS
    int16_t peak = 0;
    int64_t sum_sq = 0;
    for (int i = 0; i < samples; i++) {
        int16_t v = abs(buf[i]);
        if (v > peak) peak = v;
        sum_sq += (int32_t)v * v;
    }

    peak_value = peak;
    rms_value = (int16_t)sqrt(sum_sq / samples);

    // 判断是否收到有效音频
    if (peak_value > 200) {
        active_count++;
        silent_count = 0;
    } else {
        silent_count++;
    }

    // 累计一定数量后判定
    if (sample_count > 20) {
        mic_receiving = (active_count > 0);
    }
}

// ===== 显示页面 1: 诊断概览 =====
void showDiagPage() {
    showHeader("INMP441 Diag");

    // I2S 状态
    display.setCursor(0, 13);
    display.print("I2S Init: ");
    display.println(i2s_ok ? "OK" : "FAIL");

    // 引脚信息
    display.setCursor(0, 23);
    display.printf("SCK:%d WS:%d SD:%d", MIC_SCK_PIN, MIC_WS_PIN, MIC_SD_PIN);

    // 麦克风接收状态
    display.setCursor(0, 35);
    display.print("MIC Status: ");
    if (sample_count < 20) {
        display.println("Testing...");
    } else {
        display.println(mic_receiving ? "ACTIVE" : "NO SIGNAL");
    }

    // 数据统计
    display.setCursor(0, 47);
    display.printf("Peak:%5d RMS:%5d", peak_value, rms_value);

    // 底部提示
    display.setCursor(0, 57);
    display.print("Frames: ");
    display.print(sample_count);

    display.display();
}

// ===== 显示页面 2: 实时电平表 =====
void showLevelPage() {
    showHeader("Audio Level");

    // 电平条
    int bar_y = 14;
    int bar_h = 20;
    drawBar(4, bar_y, 120, bar_h, peak_value, 16000);

    // 峰值数字
    display.setCursor(4, bar_y + bar_h + 4);
    display.setTextSize(2);
    display.printf("%5d", peak_value);
    display.setTextSize(1);

    // RMS 条 (较短)
    int rms_y = bar_y + bar_h + 22;
    drawBar(4, rms_y, 120, 10, rms_value, 8000);
    display.setCursor(100, rms_y + 12);
    display.print("RMS");

    // 状态
    display.setCursor(0, 56);
    if (!i2s_ok) {
        display.print("ERROR: I2S failed!");
    } else if (sample_count < 20) {
        display.print("Warming up...");
    } else if (!mic_receiving) {
        display.print("NO SIGNAL - check wiring!");
    } else {
        display.print("OK - receiving audio");
    }

    display.display();
}

// ===== setup =====
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== INMP441 Microphone Diagnostics ===");

    // 初始化 I2C + OLED
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[FAIL] SSD1306 init");
        while (1) delay(1000);
    }

    // 开机画面
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(20, 20);
    display.println("INMP441 Diag");
    display.setCursor(20, 35);
    display.println("Initializing...");
    display.display();
    delay(1000);

    // 初始化 I2S 麦克风
    i2s_ok = initMicrophone();

    if (!i2s_ok) {
        // 初始化失败 - 显示错误
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 10);
        display.println("I2S INIT FAILED!");
        display.setCursor(0, 25);
        display.println("Check wiring:");
        display.setCursor(0, 37);
        display.printf("SCK->%d", MIC_SCK_PIN);
        display.setCursor(0, 47);
        display.printf("WS ->%d", MIC_WS_PIN);
        display.setCursor(0, 57);
        display.printf("SD ->%d", MIC_SD_PIN);
        display.display();
        while (1) delay(1000);
    }

    Serial.println("Diagnostics running...");
}

// ===== loop =====
void loop() {
    // 读取并分析音频
    analyzeAudio();

    // 交替显示两个页面
    static unsigned long last_switch = 0;
    static int page = 0;

    if (millis() - last_switch > 3000) {
        page = (page + 1) % 2;
        last_switch = millis();
    }

    if (page == 0) {
        showDiagPage();
    } else {
        showLevelPage();
    }

    // 串口输出诊断
    static unsigned long last_print = 0;
    if (millis() - last_print > 1000) {
        last_print = millis();
        Serial.printf("Peak:%6d  RMS:%6d  Active:%d  Silent:%d  %s\n",
                      peak_value, rms_value, active_count, silent_count,
                      mic_receiving ? "RECEIVING" : "NO SIGNAL");
    }
}
