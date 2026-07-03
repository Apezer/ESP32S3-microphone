# ESP32-S3 无线麦克风

基于 ESP32-S3 + INMP441 I2S 麦克风的无线录音系统，通过 WiFi AP 提供网页界面，支持实时音频电平显示和 WAV 录音。

## 功能

- **I2S 音频采集**：INMP441 MEMS 麦克风，16kHz 采样率，16-bit PCM
- **WiFi AP 热点**：ESP32-S3 自建 WiFi 热点，无需外部路由器
- **网页录音界面**：浏览器打开网页即可录音，支持预设时长和自定义录音
- **实时音频电平**：网页和 OLED 同时显示 RMS、峰值、dBFS
- **语音活动检测**：基于 dBFS 阈值自动判断是否有语音
- **WAV 文件下载**：录音完成后可在浏览器播放或下载 WAV 文件
- **OLED 显示**：实时显示音频状态、电平条和录音状态

## 硬件接线

### INMP441 麦克风

| INMP441 引脚 | ESP32-S3 引脚 |
|:---:|:---:|
| SCK | GPIO36 |
| WS | GPIO37 |
| SD | GPIO35 |
| L/R | GND |
| VDD | 3.3V |
| GND | GND |

### SSD1306 OLED 显示屏 (I2C)

| OLED 引脚 | ESP32-S3 引脚 |
|:---:|:---:|
| SDA | GPIO1 |
| SCL | GPIO2 |
| VCC | 3.3V |
| GND | GND |

## 使用方法

1. **编译上传固件**
   ```bash
   # 使用 PlatformIO
   pio run -t upload
   ```

2. **连接 WiFi**
   - SSID：`ESP32S3-MIC`
   - 密码：`12345678`

3. **打开浏览器访问**
   - 地址：`http://192.168.4.1`

4. **开始录音**
   - 点击预设按钮（3秒、5秒、10秒）进行固定时长录音
   - 点击「开始自定义录音」按钮进行任意时长录音，再次点击结束

## 代码架构

### 源文件

- `src/main.cpp` — 主程序，包含所有功能代码
- `platformio.ini` — PlatformIO 项目配置

### 核心模块

| 模块 | 函数 | 说明 |
|:---|:---|:---|
| OLED 初始化 | `initOled()` | 初始化 I2C 和 SSD1306 OLED |
| 启动画面 | `showBootScreen()` | 显示启动信息 |
| I2S 初始化 | `initMicrophone()` | 配置 I2S 32-bit 采集模式 |
| 麦克风读取 | `readMicFrame()` | 读取 512 个 32-bit 样本，右移 14 位转为 PCM16 |
| 音频分析 | `analyzeFrame()` | 计算 RMS、峰值、dBFS、语音活动检测 |
| OLED 更新 | `updateOled()` | 显示音频状态和电平条 |
| WAV 头生成 | `writeWavHeader()` | 生成 44 字节标准 WAV 文件头 |
| 状态接口 | `handleStatusJson()` | 返回 JSON 格式的音频状态 |
| WAV 录音 | `handleCaptureWav()` | 服务端实时录制 WAV 文件 |
| PCM 分块 | `handlePcmChunk()` | 返回指定时长的原始 PCM 数据 |

### HTTP 端点

| 端点 | 方法 | 说明 |
|:---|:---|:---|
| `/` | GET | 网页界面 |
| `/status.json` | GET | 返回音频状态 JSON（rms, peak, dbfs, voice） |
| `/capture.wav?seconds=N` | GET | 下载 N 秒 WAV 录音（1-30秒） |
| `/chunk.pcm?ms=N` | GET | 获取 N 毫秒原始 PCM 数据（100-1000ms） |

### 网页端功能

- 实时音频电平条（每 350ms 轮询 `/status.json`）
- 预设时长录音（3/5/10秒）
- 自定义开始/结束录音
- 浏览器端 WAV 编码和播放
- WAV 文件下载

## 依赖库

| 库 | 版本 | 用途 |
|:---|:---|:---|
| Adafruit SSD1306 | ^2.5.7 | OLED 显示驱动 |
| Adafruit GFX Library | ^1.11.5 | 图形绘制函数 |

## 硬件平台

- **开发板**：DFRobot FireBeetle 2 ESP32-S3
- **麦克风**：INMP441 I2S MEMS 麦克风
- **显示屏**：0.96寸 SSD1306 OLED（I2C，128x64）
