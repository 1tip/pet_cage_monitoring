
/*
<UI - 1차 완성>  2025.12.27
  - 버그수정(0~80으로 그래프 범위 제한)
  - 그래프 박스 최적화(위치, 사이즈)
*/

/**************************************************************
 * Lizard Cage Monitoring (Stable + Scroll Graph)
 * ESP32 + TFT_eSPI + DHT22 + Rotary Encoder
 * LED UI + REAL PWM LED control (2ch)
 * GRAPH_INTERVAL 적용
 *************************************************************/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <DHT.h>

// ================= CONFIG =================
#define BG_COLOR TFT_BLACK                  // 배경색

#define DHTPIN 26
#define DHTTYPE DHT22

#define ENCODER_CLK 25
#define ENCODER_DT  27
#define ENCODER_SW  22
#define ENCODER_DIR 1

#define BRIGHTNESS_STEPS 10

#define LED1_PIN 21
#define LED2_PIN 32

// 그래프 기록 간격(ms)
// 1초: 1000, 1분: 60000, 5분: 300000, 10분: 600000
#define GRAPH_INTERVAL 2000
// =========================================

TFT_eSPI tft;
DHT dht(DHTPIN, DHTTYPE);

// ---------- DHT timing ----------
unsigned long lastSample = 0;
#define SAMPLE_INTERVAL 1000

// ---------- Graph config ----------
#define GRAPH_X 40
#define GRAPH_Y 64    //74
#define GRAPH_W 260   //200
#define GRAPH_H 90    //70

float tempBuf[GRAPH_W];
float humiBuf[GRAPH_W];

// ---------- UI layout ----------
#define TITLE_Y 5
#define INFO_Y  40
#define LED_UI_Y1 180
#define LED_UI_Y2 210

// ---------- Encoder ----------
int brightnessStep[2] = {0};
int selectedLED = 0;
int lastCLK;
unsigned long lastBtnTime = 0;
const unsigned long debounceMs = 200;

// ---------- PWM ----------
int pwmValue[2] = {0, 0};

// ---------- Graph interval ----------
unsigned long lastGraphUpdate = 0;

// =========================================
// Utility
// =========================================
int stepToPWM(int step) {
  return map(step, 0, BRIGHTNESS_STEPS, 0, 255);
}

// =========================================
// UI
// =========================================
void drawTitle() {
  tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(10, TITLE_Y);
  tft.print("Lizard Cage Monitoring");
}

void drawTempHumi(float t, float h, bool err) {
  //tft.fillRect(0, INFO_Y, 240, 30, BG_COLOR);
  tft.fillRect(0, INFO_Y, 240, 20, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(10, INFO_Y);

  if (err) {
    tft.setTextColor(TFT_RED, BG_COLOR);
    tft.print("TEMP / HUMI : ERR");
  } else {
    tft.setTextColor(TFT_WHITE, BG_COLOR);
    tft.printf("T: %.1fC  H: %.1f%%", t, h);
  }
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

  // 최대 80으로 제한
  tempBuf[GRAPH_W - 1] = (t > 80) ? 80 : t;
  humiBuf[GRAPH_W - 1] = (h > 80) ? 80 : h;
}

void drawGraph() {
  // 그래프 영역 초기화
  tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, BG_COLOR);

  // ---------- Y축 기준선 표시 (0 제외) ----------
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, BG_COLOR);

  // 0 텍스트 표시
  int y0 = GRAPH_Y + GRAPH_H; // 0 값 위치
  tft.setCursor(GRAPH_X - 20, y0 - 4);
  tft.printf("0");

  // 20, 40, 60 기준선 표시
  for (int i = 20; i <= 60; i += 20) {
    int y = map(i, 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_W - 1, y, TFT_DARKGREY);
    tft.setCursor(GRAPH_X - 20, y - 4); // 왼쪽 여백
    tft.printf("%d", i);
  }

  // ---------- 온도 그래프 ----------
  for (int i = 1; i < GRAPH_W; i++) {
    if (!isnan(tempBuf[i - 1]) && !isnan(tempBuf[i])) {
      int y1 = map(tempBuf[i - 1], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      int y2 = map(tempBuf[i], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      tft.drawLine(GRAPH_X + i - 1, y1,
                   GRAPH_X + i, y2, TFT_RED);
    }
  }

  // ---------- 습도 그래프 ----------
  for (int i = 1; i < GRAPH_W; i++) {
    if (!isnan(humiBuf[i - 1]) && !isnan(humiBuf[i])) {
      int hy1 = map(humiBuf[i - 1], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      int hy2 = map(humiBuf[i], 0, 80, GRAPH_Y + GRAPH_H, GRAPH_Y);
      tft.drawLine(GRAPH_X + i - 1, hy1,
                   GRAPH_X + i, hy2, TFT_CYAN);
    }
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
}

void loop() {
  // ---------- Encoder rotation ----------
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

  // ---------- Encoder button ----------
  if (!digitalRead(ENCODER_SW)) {
    if (millis() - lastBtnTime > debounceMs) {
      selectedLED ^= 1;
      drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
      drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);
      lastBtnTime = millis();
    }
  }

  // ---------- DHT + Graph ----------
  if (millis() - lastSample >= SAMPLE_INTERVAL) {
    lastSample = millis();

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    bool err = isnan(t) || isnan(h);

    drawTempHumi(t, h, err);

    if (!err) {
      // GRAPH_INTERVAL 적용
      if (millis() - lastGraphUpdate >= GRAPH_INTERVAL) {
        lastGraphUpdate = millis();
        pushGraphData(t, h);
        drawGraph();
      }
    }
  }
}
