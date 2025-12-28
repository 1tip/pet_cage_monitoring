/**************************************************************
 * Lizard Cage Monitoring (Stable + Scroll Graph + External AP + Long-term Graph)
 * ESP32 + TFT_eSPI + DHT22 + Rotary Encoder
 * LED UI + REAL PWM LED control (2ch)
 * GRAPH_INTERVAL 적용 + WebServer 외부 AP 설정
 * 장시간 평균 기반 그래프 (1~24시간 X축) + 가변 타임스탬프
 *************************************************************/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>

// ================= CONFIG =================
#define BG_COLOR TFT_BLACK
#define DHTPIN 26
#define DHTTYPE DHT22
#define ENCODER_CLK 25
#define ENCODER_DT  27
#define ENCODER_SW  22
#define ENCODER_DIR 1
#define BRIGHTNESS_STEPS 10
#define LED1_PIN 21
#define LED2_PIN 32


#define SAMPLE_INTERVAL 1000          // DHT22

// ---------- ESP32 AP ----------
#define AP_SSID "Cage"
#define AP_PASSWORD "cage1234"
IPAddress apIP(192,168,4,1);


// ---------- NTP ----------
const char* ntpServer = "time.bora.net";
const long gmtOffset_sec = 9*3600;
const int daylightOffset_sec = 0;


// ---------- Graph ----------
#define GRAPH_X 40
#define GRAPH_Y 64
#define GRAPH_W 260           // 그래프박스 너비
#define GRAPH_H 90

#define DISPLAY_HOURS 6   // 1~24시간   (그래프 X축 폭 [GRAPH_W 260] 전체 채움 소요 시간)
#define NUM_LABELS 6      // X축 레이블 개수


// ======= Global Variables =======
unsigned long lastTimeAxisTick = 0;   // 그래프 + X축 레이블 공용

int labelOffset = 0; // X축 레이블 floating offset (1px 단위)

// 1px 이동 시간 (DISPLAY_HOURS 기준)
const unsigned long LABEL_SCROLL_INTERVAL_MS = (DISPLAY_HOURS * 3600UL * 1000UL) / GRAPH_W;



float tempBuf[GRAPH_W];
float humiBuf[GRAPH_W];

// ---------- UI layout ----------
#define TITLE_Y 5
#define INFO_Y  40
#define LED_UI_Y1 180
#define LED_UI_Y2 210

TFT_eSPI tft;
DHT dht(DHTPIN, DHTTYPE);

// ---------- Encoder ----------
int brightnessStep[2] = {0};
int selectedLED = 0;
int lastCLK;
unsigned long lastBtnTime = 0;
const unsigned long debounceMs = 200;

// ---------- PWM ----------
int pwmValue[2] = {0, 0};

// ---------- Timing ----------
//unsigned long lastGraphUpdate = 0;
unsigned long lastSample = 0;

// ---------- WiFi & WebServer ----------
WebServer server(80);
Preferences preferences;
bool externalAPConnected = false;

// =========================================
// Utility
// =========================================
int stepToPWM(int step) {
  return map(step, 0, BRIGHTNESS_STEPS, 0, 255);
}

// =========================================
// UI Functions
// =========================================
void drawTitle() {
  tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(10, TITLE_Y);
  tft.print(" Lizard Cage Monitoring");
}

void drawTempHumi(float t, float h, bool timeAvailable) {
  tft.fillRect(0, INFO_Y, 240, 20, BG_COLOR);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setCursor(10, INFO_Y);

  char buffer[32];
  bool sensorErr = isnan(t) || isnan(h);

  if (!timeAvailable || sensorErr) {
    snprintf(buffer, sizeof(buffer), "--/-- --:--:-- T:-- H:--");
  } else {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      snprintf(buffer, sizeof(buffer), "--/-- --:--:-- T:-- H:--");
    } else {
      snprintf(buffer, sizeof(buffer), "%02d/%02d %02d:%02d:%02d T:%02.0f H:%02.0f",
               timeinfo.tm_mon+1, timeinfo.tm_mday,
               timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
               t, h);
    }
  }

  tft.print(buffer);
}

void drawLEDUI(int led, int y, int level, bool selected) {
  tft.fillRect(0, y, 240, 28, BG_COLOR);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setCursor(10, y);

  if (selected) tft.print("*");
  else          tft.print(" ");

  tft.printf(" LED%d", led);

  int barX = 90;
  int barW = 120;
  int barH = 14;

  tft.drawRect(barX, y, barW, barH, TFT_WHITE);

  int fillW = map(level, 0, BRIGHTNESS_STEPS, 0, barW - 2);
  if (fillW > 0) {
    tft.fillRect(barX + 1, y + 1, fillW, barH - 2, TFT_GREEN);
  }
}

// =========================================
// Graph
// =========================================
void initGraph() {
  for (int i = 0; i < GRAPH_W; i++) {
    tempBuf[i] = NAN;
    humiBuf[i] = NAN;
  }

  tft.drawRect(GRAPH_X - 1, GRAPH_Y - 1,
               GRAPH_W + 2, GRAPH_H + 2, TFT_WHITE);
}

void pushGraphData(float t, float h) {
  for (int i = 0; i < GRAPH_W - 1; i++) {
    tempBuf[i] = tempBuf[i + 1];
    humiBuf[i] = humiBuf[i + 1];
  }

  tempBuf[GRAPH_W - 1] = (t > 80) ? 80 : t;
  humiBuf[GRAPH_W - 1] = (h > 80) ? 80 : h;
}


void drawGraph() {
  // 그래프 영역 초기화
  tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, BG_COLOR);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, BG_COLOR);

  // ---------- Y축 레이블 및 기준선 ----------
  int y0 = GRAPH_Y + GRAPH_H;
  tft.setCursor(GRAPH_X - 20, y0 - 4);
  tft.printf("0");

  for (int i = 20; i <= 60; i += 20) {
    int y = map(i, 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_W - 1, y, TFT_DARKGREY);
    tft.setCursor(GRAPH_X - 20, y - 4);
    tft.printf("%d", i);
  }

  // ---------- X축 세로 기준선 추가 (6등분) ----------
  int totalDiv = NUM_LABELS;   // 6등분
  int stepPx = GRAPH_W / (totalDiv - 1);

  for(int i = 1; i < totalDiv; i++){  // i=0은 가장 오래된 데이터 위치, 표시 제외
    int x = GRAPH_X + i * stepPx - labelOffset;  // labelOffset은 floating 이동 적용

    // 그래프 내부에 있을 때만 선 표시
    if(x >= GRAPH_X && x <= GRAPH_X + GRAPH_W){
      tft.drawLine(x, GRAPH_Y, x, GRAPH_Y + GRAPH_H, TFT_DARKGREY);
    }
  }

  // ---------- 온도 그래프 ----------
  for (int i = 1; i < GRAPH_W; i++) {
    if (!isnan(tempBuf[i - 1]) && !isnan(tempBuf[i])) {
      int y1 = map(tempBuf[i - 1], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      int y2 = map(tempBuf[i], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      tft.drawLine(GRAPH_X + i - 1, y1, GRAPH_X + i, y2, TFT_RED);
    }
  }

  // ---------- 습도 그래프 ----------
  for (int i = 1; i < GRAPH_W; i++) {
    if (!isnan(humiBuf[i - 1]) && !isnan(humiBuf[i])) {
      int hy1 = map(humiBuf[i - 1], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      int hy2 = map(humiBuf[i], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      tft.drawLine(GRAPH_X + i - 1, hy1, GRAPH_X + i, hy2, TFT_CYAN);
    }
  }

}


// =========================================
// X-axis Labels (6등분, 가장 오래된 1개는 미표시, floating)
// =========================================

void drawGraphLabels() {
  const int totalDiv = 6;        // X축 6등분
  const int stepPx = GRAPH_W / (totalDiv - 1); // 6등분 간격

  // 현재 시간 가져오기
  time_t nowSec;
  time(&nowSec);
  struct tm timeinfo;
  localtime_r(&nowSec, &timeinfo);

  // 현재 분을 10분 단위로 내림
  int min10 = (timeinfo.tm_min / 10) * 10;

  // 6등분 좌표 중 가장 오래된 1개는 표시 안함
  for(int i = 1; i < totalDiv; i++){  // i=1 부터 시작, i=0은 생략
    int labelHour = timeinfo.tm_hour;
    int labelMin = min10 - (totalDiv - 1 - i) * 10;

    if(labelMin < 0){
      labelMin += 60;
      labelHour = (labelHour + 23) % 24;
    }

    char label[6];
    snprintf(label, sizeof(label), "%02d:%02d", labelHour, labelMin);

    int x = GRAPH_X + (i) * stepPx - labelOffset;

    // 화면 밖이면 continue
    if(x < GRAPH_X || x > GRAPH_X + GRAPH_W) continue;

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, BG_COLOR);
    tft.setCursor(x - 12, GRAPH_Y + GRAPH_H + 6); // Y 좌표 6px 하강
    tft.print(label);
  }

  tft.drawRect(GRAPH_X - 1, GRAPH_Y - 1, GRAPH_W + 2, GRAPH_H + 2, TFT_WHITE);   // 그래프영역 외곽선 그리기(겹침발생시 대비)
}



// =========================================
// WiFi / NTP
// =========================================
void startAP() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
}

void handleRoot() {
  String page = "<html><body>";
  page += "<h3>Set External WiFi</h3>";
  page += "<form method='POST' action='/save'>";
  page += "SSID: <input name='ssid'><br>";
  page += "Password: <input name='password'><br>";
  page += "<input type='submit' value='Save'>";
  page += "</form></body></html>";
  server.send(200, "text/html", page);
}

void handleSave() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String ssid = server.arg("ssid");
    String pass = server.arg("password");

    preferences.begin("wifi", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", pass);
    preferences.end();

    server.send(200, "text/html", "<html><body>Saved! Rebooting...</body></html>");
    delay(1000);
    ESP.restart();
  }
}

void connectExternalAP() {
  preferences.begin("wifi", true);
  String ssid = preferences.getString("ssid", "");
  String pass = preferences.getString("password", "");
  preferences.end();

  if (ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
    }
  }
  externalAPConnected = (WiFi.status() == WL_CONNECTED);

  if(externalAPConnected){
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }
}

// =========================================
void setup() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  analogWrite(LED1_PIN, 0);
  analogWrite(LED2_PIN, 0);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(BG_COLOR);

  dht.begin();
  lastCLK = digitalRead(ENCODER_CLK);

  drawTitle();
  initGraph();
  drawLEDUI(1, LED_UI_Y1, brightnessStep[0], true);
  drawLEDUI(2, LED_UI_Y2, brightnessStep[1], false);

  startAP();
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();

  connectExternalAP();
}

// =========================================
void loop() {
  server.handleClient();

  // ================== Encoder ==================
  int clk = digitalRead(ENCODER_CLK);
  if (clk != lastCLK) {
    if (digitalRead(ENCODER_DT) != clk)
      brightnessStep[selectedLED] += ENCODER_DIR;
    else
      brightnessStep[selectedLED] -= ENCODER_DIR;

    brightnessStep[selectedLED] =
      constrain(brightnessStep[selectedLED], 0, BRIGHTNESS_STEPS);

    pwmValue[0] = stepToPWM(brightnessStep[0]);
    pwmValue[1] = stepToPWM(brightnessStep[1]);

    analogWrite(LED1_PIN, pwmValue[0]);
    analogWrite(LED2_PIN, pwmValue[1]);

    drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
    drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);

    lastCLK = clk;
  }

  if (!digitalRead(ENCODER_SW)) {
    if (millis() - lastBtnTime > debounceMs) {
      selectedLED ^= 1;
      drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
      drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);
      lastBtnTime = millis();
    }
  }

  // ================== Sensor Sampling (1초) ==================
  static float lastTemp = NAN;
  static float lastHumi = NAN;

  if (millis() - lastSample >= SAMPLE_INTERVAL) {
    lastSample = millis();

    lastTemp = dht.readTemperature();
    lastHumi = dht.readHumidity();

    drawTempHumi(lastTemp, lastHumi, externalAPConnected);
  }


// ================== Time Axis Tick (Graph + Labels) ==================
if (millis() - lastTimeAxisTick >= LABEL_SCROLL_INTERVAL_MS) {
  lastTimeAxisTick = millis();


  // X축 레이블 스크롤(1px) 이전 배경 지우기 (Y 좌표 6px 하강, X축 기준으로 10px 확장)
  tft.fillRect(GRAPH_X - 10, GRAPH_Y + GRAPH_H + 6, GRAPH_W, 16, BG_COLOR);

  // 1px floating 이동
  labelOffset++;
  if (labelOffset >= (GRAPH_W / (NUM_LABELS - 1))) {
    labelOffset = 0;
  }

  // 그래프 데이터 이동
  if (!isnan(lastTemp) && !isnan(lastHumi)) {
    pushGraphData(lastTemp, lastHumi);
    drawGraph();
  }
}


  // ================== X-axis Labels ==================
  drawGraphLabels();
}
