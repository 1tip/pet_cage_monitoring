// 그리프와 격자간 겹침발생시 오류 수정 (그래프가 그리드에 묻힘)

/**************************************************************
 * Lizard Cage Monitoring (Stable + Scroll Graph + External AP + Long-term Graph)
 * ESP32 + TFT_eSPI + DHT22 + Rotary Encoder
 * LED UI + REAL PWM LED control (2ch)
 * GRAPH_INTERVAL 적용 + WebServer 외부 AP 설정
 * 장시간 평균 기반 그래프 (1~24시간 X축) + 가변 타임스탬프
 *************************************************************/
// 업로드할때 ESP32 BOOT 버튼을 누르고 있는 상태여야 함

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <Wire.h>
#include <SensirionI2cSht4x.h>
#define SHT41_I2C_ADDR 0x44



#define DISPLAY_HOURS 12   // 1~24시간   (그래프 X축 폭 [GRAPH_W 260] 전체 채움 소요 시간)
#define NUM_LABELS 7       // X축 레이블 개수(칸갯수: n-1)



// ================= CONFIG =================
#define BG_COLOR TFT_BLACK
#define BRIGHTNESS_STEPS 10
#define LED1_PIN 12
#define LED2_PIN 32

#define I2C_SDA 33
#define I2C_SCL 26


// ================= Rotary Encoder HW =================
#define HW_VER_2

#if defined(HW_VER_1)     // HW Ver.1
#define ENCODER_CLK 25
#define ENCODER_DT  27
#define ENCODER_SW  22
#define ENCODER_DIR 1

#elif defined(HW_VER_2)   // HW Ver.2 : S1(황_21), S2(녹_19), Key(보_4)
#define ENCODER_CLK 19
#define ENCODER_DT  21
#define ENCODER_SW  4
#define ENCODER_DIR -1

#else
#error "HW_VER_1 또는 HW_VER_2 중 하나를 반드시 정의해야 합니다."
#endif


// ---------- Encoder ----------
int brightnessStep[2] = {0, 0};
int selectedLED = 0;

uint8_t lastEncState = 0;   // Ver1 / Ver2 공용


bool btnPrevState = HIGH;


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
#define GRAPH_Y 66  //64
#define GRAPH_W 260           // 그래프박스 너비
#define GRAPH_H 90


// ======= Global Variables =======
unsigned long lastTimeAxisTick = 0;   // 그래프 + X축 레이블 공용

// 1px 이동 시간 (DISPLAY_HOURS 기준)
const unsigned long LABEL_SCROLL_INTERVAL_MS = (DISPLAY_HOURS * 3600UL * 1000UL) / GRAPH_W;


#define GRID_COLOR  0x4208   // 매우 어두운 회색 (RGB565)  (TFT_DARKGREY보다 약 50% 정도 어두움)


float tempBuf[GRAPH_W];
float humiBuf[GRAPH_W];

// ---------- UI layout ----------
#define TITLE_Y 5
#define INFO_Y  40
#define LED_UI_Y1 180
#define LED_UI_Y2 210

TFT_eSPI tft;

SensirionI2cSht4x sht4;

float lastTemp = NAN;
float lastHumi = NAN;



// ---------- PWM ----------
int pwmValue[2] = {0, 0};

// ---------- Timing ----------
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
  const int TITLE_H = 28;   // 타이틀 영역 높이

  // 타이틀 배경 (파란색 음영)
  tft.fillRect(0, TITLE_Y, tft.width(), TITLE_H, TFT_NAVY);

  // 타이틀 텍스트
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextSize(2);
  tft.setCursor(0, TITLE_Y + 6);  // 세로 중앙 보정
  tft.print("  Lizard Cage Monitoring");

  // 하단 구분선 (선택 사항, 입체감)
  tft.drawFastHLine(0, TITLE_Y + TITLE_H - 1, tft.width(), TFT_DARKGREY);
}


void drawTimeTempHumi(float t, float h, bool timeAvailable) {
  tft.fillRect(0, INFO_Y, 240, 20, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(10, INFO_Y);

  if (!timeAvailable || isnan(t) || isnan(h)) {
    tft.setTextColor(TFT_LIGHTGREY, BG_COLOR);
    tft.print(" --:--:--  --.-C  --.-%");
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    tft.setTextColor(TFT_LIGHTGREY, BG_COLOR);
    tft.print(" --:--:--  --.-C  --.-%");
    return;
  }

  // 시간
  tft.setTextColor(TFT_BLUE, BG_COLOR);
  tft.printf(" %02d:%02d:%02d  ",
             timeinfo.tm_hour,
             timeinfo.tm_min,
             timeinfo.tm_sec);

  // 온도 (소수 1자리)
  tft.setTextColor(TFT_YELLOW, BG_COLOR);
  tft.printf("%4.1fC  ", t);

  // 습도 (소수 1자리)
  tft.setTextColor(TFT_GREEN, BG_COLOR);
  tft.printf("%4.1f%% ", h);
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






//---------------------------------2025.12.31------------------------------
void drawGraphGrid() {
  // ----- Y축 격자선 -----
  for (int i = 20; i <= 80; i += 20) {
    int y = map(i, 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_W - 1, y, GRID_COLOR);
  }

  // ----- X축 격자선 -----
  const float totalMin = DISPLAY_HOURS * 60.0f;
  const float minPerPx = totalMin / GRAPH_W;
  const float labelStepMin = totalMin / (NUM_LABELS - 1);

  time_t nowSec;
  time(&nowSec);
  struct tm timeinfo;
  localtime_r(&nowSec, &timeinfo);

  int nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int anchorMin = (nowMin / 10) * 10;

  float anchorOffsetPx = (nowMin - anchorMin) / minPerPx;
  int anchorX = GRAPH_X + GRAPH_W - (int)(anchorOffsetPx + 0.5f);

  for (int i = 0; i < NUM_LABELS; i++) {
    float offsetMin = (NUM_LABELS - 1 - i) * labelStepMin;
    int x = anchorX - (int)((offsetMin / minPerPx) + 0.5f);
    if (x < GRAPH_X || x > GRAPH_X + GRAPH_W) continue;
    tft.drawFastVLine(x, GRAPH_Y, GRAPH_H, GRID_COLOR);
  }
}






// =========================================
// X-axis Labels (#define NUM_LABELS : 격자선 개수(n-1), floating)
// Y축 기준선 추가
// =========================================

void drawGraphLabels() {              // (1.drawGraphGrid -> 2.drawGraph -> 3.drawGraphLabels)

  // ---------- Y축 레이블 및 기준선(시작) ----------
  tft.setTextSize(1);
  //tft.setTextColor(TFT_DARKGREY, BG_COLOR);
  tft.setTextColor(TFT_DARKCYAN, BG_COLOR);

  for (int i = 20; i <= 80; i += 20) {
    int y = map(i, 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
    //tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_W - 1, y, TFT_DARKGREY);
    /////////////////////////////////////////////////////////////////////////////////////tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_W - 1, y, GRID_COLOR);       // 매우 어두운 회색 (RGB565)
    tft.setCursor(GRAPH_X - 18, y - 4);
    tft.printf("%d", i);
  }
  int y0 = GRAPH_Y + GRAPH_H;
  tft.setCursor(GRAPH_X - 18, y0 - 4);
  tft.printf(" 0");
  // ---------- Y축 레이블 및 기준선(종료) ----------



  // ---------- X축 레이블 및 기준선 ----------
  // ===== 전체 시간폭 =====
  const float totalMin = DISPLAY_HOURS * 60.0f;
  const float minPerPx = totalMin / GRAPH_W;

  // ===== 레이블 간 시간 =====
  const float labelStepMin = totalMin / (NUM_LABELS - 1);

  // ===== 현재 시각 =====
  time_t nowSec;
  time(&nowSec);
  struct tm timeinfo;
  localtime_r(&nowSec, &timeinfo);

  int nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  // ===== 가장 최근 10분 단위 (기준점) =====
  int anchorMin = (nowMin / 10) * 10;

  // ===== anchor가 그래프 우측에서 떨어진 px =====
  float anchorOffsetPx = (nowMin - anchorMin) / minPerPx;
  int anchorX = GRAPH_X + GRAPH_W - (int)(anchorOffsetPx + 0.5f);

  // ===== X축 레이블 영역 클리어 =====
  tft.fillRect(GRAPH_X - 12,
               GRAPH_Y + GRAPH_H + 6,
               GRAPH_W + 24 + 20,
               14,
               BG_COLOR);

  // ===== 레이블 + 세로 기준선 =====
  for (int i = 0; i < NUM_LABELS; i++) {

    float offsetMin = (NUM_LABELS - 1 - i) * labelStepMin;
    int labelMin = anchorMin - (int)(offsetMin + 0.5f);

    while (labelMin < 0)        labelMin += 1440;
    while (labelMin >= 1440)    labelMin -= 1440;

    int x = anchorX - (int)((offsetMin / minPerPx) + 0.5f);

    if (x < GRAPH_X || x > GRAPH_X + GRAPH_W) continue;

    // 세로 기준선
    // tft.drawFastVLine(x, GRAPH_Y, GRAPH_H, TFT_DARKGREY);
    ////////////////////////////////////////////////////////////////////tft.drawFastVLine(x, GRAPH_Y, GRAPH_H, GRID_COLOR);           // 매우 어두운 회색 (RGB565)

    // 레이블
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d",
             labelMin / 60, labelMin % 60);

    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREEN, BG_COLOR);             // TFT_BLUE
    tft.setCursor(x - 12, GRAPH_Y + GRAPH_H + 6);
    tft.print(buf);
  }
  tft.drawRect(GRAPH_X - 1, GRAPH_Y - 1, GRAPH_W + 2, GRAPH_H + 2, TFT_WHITE);   // 그래프영역 외곽선 그리기(겹침발생시 대비)
}



void drawGraph() {
  // 그래프 영역 초기화
  tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, BG_COLOR);       // 그래프박스 내부영역 지우기
  drawGraphGrid();      // 1. 격자 그리기 (1.drawGraphGrid -> 2.drawGraph -> 3.drawGraphLabels)
  drawGraphLabels();

  // ---------- 습도 그래프 ----------
  for (int i = 1; i < GRAPH_W; i++) {
    if (!isnan(humiBuf[i - 1]) && !isnan(humiBuf[i])) {
      int hy1 = map(humiBuf[i - 1], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      int hy2 = map(humiBuf[i], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      tft.drawLine(GRAPH_X + i - 1, hy1, GRAPH_X + i, hy2, TFT_GREEN);
    }
  }

  // ---------- 온도 그래프 ----------
  for (int i = 1; i < GRAPH_W; i++) {
    if (!isnan(tempBuf[i - 1]) && !isnan(tempBuf[i])) {
      int y1 = map(tempBuf[i - 1], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      int y2 = map(tempBuf[i], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      //tft.drawLine(GRAPH_X + i - 1, y1, GRAPH_X + i, y2, TFT_ORANGE);
      tft.drawLine(GRAPH_X + i - 1, y1, GRAPH_X + i, y2, TFT_YELLOW);
    }
  }

}







void handleEncoderRotation() {
  uint8_t encState =
    (digitalRead(ENCODER_CLK) << 1) |
     digitalRead(ENCODER_DT);

  if (encState == lastEncState) return;

  int delta = 0;

  if (
      (lastEncState == 0b00 && encState == 0b01) ||
      (lastEncState == 0b01 && encState == 0b11) ||
      (lastEncState == 0b11 && encState == 0b10) ||
      (lastEncState == 0b10 && encState == 0b00)
     ) {
    delta = +1;
  }
  else if (
      (lastEncState == 0b00 && encState == 0b10) ||
      (lastEncState == 0b10 && encState == 0b11) ||
      (lastEncState == 0b11 && encState == 0b01) ||
      (lastEncState == 0b01 && encState == 0b00)
     ) {
    delta = -1;
  }

  if (delta != 0) {
    brightnessStep[selectedLED] += delta * ENCODER_DIR;
    brightnessStep[selectedLED] =
      constrain(brightnessStep[selectedLED], 0, BRIGHTNESS_STEPS);

    pwmValue[0] = stepToPWM(brightnessStep[0]);
    pwmValue[1] = stepToPWM(brightnessStep[1]);

    analogWrite(LED1_PIN, pwmValue[0]);
    analogWrite(LED2_PIN, pwmValue[1]);

    drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
    drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);
  }

  lastEncState = encState;
}



void handleEncoderButton() {
  bool btnCurrState = digitalRead(ENCODER_SW);

  // LOW -> HIGH : Release
  if (btnPrevState == LOW && btnCurrState == HIGH) {
    selectedLED ^= 1;

    // 즉시 UI 반영
    drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
    drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);
  }

  btnPrevState = btnCurrState;
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

  // 초기 인코더 상태 (CLK + DT)
  lastEncState =
    (digitalRead(ENCODER_CLK) << 1) |
     digitalRead(ENCODER_DT);


  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  analogWrite(LED1_PIN, 0);
  analogWrite(LED2_PIN, 0);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(BG_COLOR);

  Wire.begin(I2C_SDA, I2C_SCL);
  sht4.begin(Wire, SHT41_I2C_ADDR);
  delay(10);
 
  sht4.softReset();


  drawTitle();
  drawGraphGrid();        // 2025.12.31 수정
  initGraph();


  drawLEDUI(1, LED_UI_Y1, brightnessStep[0], true);
  drawLEDUI(2, LED_UI_Y2, brightnessStep[1], false);


  startAP();
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();

  connectExternalAP();

  delay(5000);        // 시각동기완료 대기


  if (!isnan(lastTemp) && !isnan(lastHumi)) {
    pushGraphData(lastTemp, lastHumi);
    drawGraph();
  }
  drawGraphLabels();  // X, Y축 레이블, 격자 그리기
  lastSample = millis();
}



// =========================================
void loop() {
  server.handleClient();

  handleEncoderRotation();
  handleEncoderButton();


  // ================== Sensor Sampling (1초) ==================
  if (millis() - lastSample >= SAMPLE_INTERVAL) {
  lastSample = millis();

  float temperature = NAN;
  float humidity = NAN;

  uint16_t error = sht4.measureHighPrecision(temperature, humidity);

  if (error == 0) {
    lastTemp = temperature;
    lastHumi = humidity;
  } else {
    lastTemp = NAN;
    lastHumi = NAN;
  }

  drawTimeTempHumi(lastTemp, lastHumi, externalAPConnected);
}


/*
  // ================== Time Axis Tick (Graph + Labels) ==================
  if (millis() - lastTimeAxisTick >= LABEL_SCROLL_INTERVAL_MS) {
    lastTimeAxisTick = millis();

    if (!isnan(lastTemp) && !isnan(lastHumi)) {
      pushGraphData(lastTemp, lastHumi);

      tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, BG_COLOR);

      //drawGraphGrid();    // 1. 격자
      //drawGraph();        // 2. 그래프

      drawGraphLabels();  // 3. 라벨/외곽선
    }
  }
*/


  // ================== Time Axis Tick (Graph + Labels) ==================
  if (millis() - lastTimeAxisTick >= LABEL_SCROLL_INTERVAL_MS) {
    lastTimeAxisTick = millis();


    if (!isnan(lastTemp) && !isnan(lastHumi)) {
      pushGraphData(lastTemp, lastHumi);
      //drawGraphGrid();     // 임시TEST
      drawGraph();        // 그래프
    }
    //drawGraphGrid();      // 1. 격자  (1.drawGraphGrid -> 2.drawGraph -> 3.drawGraphLabels)
    //drawGraphLabels();    // 3. 라벨/외곽선   (1.drawGraphGrid -> 2.drawGraph -> 3.drawGraphLabels)
  }

}
