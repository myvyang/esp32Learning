
用python写一个简单的接收代码：

```python
import socket
import threading
import os
import time

# 配置
HOST = '0.0.0.0'      # 监听所有接口
PORT = 8080           # 端口
SAVE_DIR = './recordings'
BUFFER_SIZE = 4096    # 接收缓冲区大小

# 确保保存目录存在
os.makedirs(SAVE_DIR, exist_ok=True)

def handle_client(client_socket, address):
    print(f"[+] 客户端连接: {address}")
    filename = f"recording_{int(time.time())}.pcm"
    filepath = os.path.join(SAVE_DIR, filename)
    
    try:
        with open(filepath, 'wb') as f:
            while True:
                data = client_socket.recv(BUFFER_SIZE)
                if not data:
                    print(f"[-] 客户端 {address} 断开连接")
                    break
                f.write(data)
        print(f"[✓] 音频已保存至: {filepath}")
    except Exception as e:
        print(f"[!] 接收数据出错: {e}")
    finally:
        client_socket.close()

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)
    print(f"[*] PCM 服务器启动，监听 {HOST}:{PORT}")

    try:
        while True:
            client_sock, addr = server.accept()
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_sock, addr)
            )
            client_thread.daemon = True
            client_thread.start()
    except KeyboardInterrupt:
        print("\n[!] 服务器关闭中...")
    finally:
        server.close()

if __name__ == "__main__":
    start_server()
```

AI写的简单的存储发送到远端的代码：

```c
#include <WiFi.h>
#include "driver/i2s.h"

// ===== WiFi 配置 =====
const char* ssid = "xxx";
const char* password = "xxxxxxxx";

// ===== 服务器配置 =====
const char* serverIP = "192.168.31.159";  // 替换为你的 Python 服务器 IP
const uint16_t serverPort = 8080;

// ===== I2S 配置 =====
#define I2S_PORT        I2S_NUM_0
#define I2S_SCK_PIN     16   // BCLK
#define I2S_WS_PIN      15   // LRCLK
#define I2S_SD_PIN      17   // DOUT

#define SAMPLE_RATE     16000
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT  // INMP441 实际 24-bit，但用 32-bit 传输
#define BUFFER_SAMPLES  1024
#define BUFFER_SIZE_BYTES (BUFFER_SAMPLES * sizeof(int32_t))

WiFiClient client;
bool connected = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 + INMP441 (driver/i2s.h) Audio Streamer");

  // 连接 WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // === 配置 I2S ===
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = BITS_PER_SAMPLE,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // INMP441 L/R 接 GND => 左声道
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = BUFFER_SAMPLES,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT  // 自动匹配 bits_per_sample
  };

  i2s_pin_config_t pin_config = {
      I2S_PIN_NO_CHANGE,  // mck_io_num (MCLK, 不用)
      I2S_SCK_PIN,        // bck_io_num (BCLK)
      I2S_WS_PIN,         // ws_io_num (LRCLK)
      I2S_PIN_NO_CHANGE,  // data_out_num (TX, 不用)
      I2S_SD_PIN          // data_in_num (DIN, 接麦克风 SD)
  };

  // 安装驱动
  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("❌ I2S 驱动安装失败: %d\n", err);
    return;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("❌ I2S 引脚设置失败: %d\n", err);
    return;
  }

  // 启动 I2S
  i2s_start(I2S_PORT);
  Serial.println("🎤 I2S 麦克风初始化完成");

  // 连接服务器
  if (client.connect(serverIP, serverPort)) {
    Serial.println("📡 连接到服务器");
    connected = true;
  } else {
    Serial.println("❌ 无法连接服务器");
  }
}

void loop() {
  if (!connected) {
    delay(2000);
    return;
  }

  static int32_t rx_buffer[BUFFER_SAMPLES];
  size_t bytes_read = 0;

  // 从 I2S 读取数据
  esp_err_t ret = i2s_read(I2S_PORT, rx_buffer, BUFFER_SIZE_BYTES, &bytes_read, portMAX_DELAY);
  
  if (ret == ESP_OK && bytes_read > 0) {
    // 发送原始 PCM 数据
    client.write((uint8_t*)rx_buffer, bytes_read);

    // 调试：每秒打印一次
    static uint32_t total = 0;
    static uint32_t last_print = millis();
    total += bytes_read;
    if (millis() - last_print > 1000) {
      Serial.printf("📤 %.1f KB/s\n", total / 1024.0);
      total = 0;
      last_print = millis();
    }
  }

  // 检查连接
  if (!client.connected()) {
    Serial.println("🔌 服务器断开");
    connected = false;
    client.stop();
    delay(3000);
    if (client.connect(serverIP, serverPort)) {
      connected = true;
      Serial.println("🔄 重连成功");
    }
  }

  delay(1);
}

// 可选：退出时清理
void cleanup() {
  i2s_stop(I2S_PORT);
  i2s_driver_uninstall(I2S_PORT);
}
```

注意这里非常重要的参数：

```c
#define SAMPLE_RATE     16000
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT  // INMP441 实际 24-bit，但用 32-bit 传输
```

采样率是16000，位深是32，用 https://pcm.qer.im/ 播放。如果设置错误，播放的就都是噪声。

服务端代码其实应该可以直接写成播放PCM文件的就行。

原理很简单，inmp441输出给esp32的就是一串字节流，反应特定时间的声音频率序列。

理论上，模拟录音器中，前端是一个咪头，然后接入 LM386 音频放大模块，就可以让esp32接收到模拟信号。只是精度太低。

在PCM最前面加上格式，就是WAV格式了。










