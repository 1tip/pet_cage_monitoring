/**************************************************************
 * Lizard Cage Monitoring (Stable + Scroll Graph + External AP + Long-term Graph)
 * ESP32 + TFT_eSPI + DHT22 + Rotary Encoder
 * LED UI + REAL PWM LED control (2ch)
 * GRAPH_INTERVAL 적용 + WebServer 외부 AP 설정
 * 장시간 평균 기반 그래프 (1~24시간 X축) + 가변 타임스탬프
 * 그래프모드 추가(1,6,12,24H)
 * 플래시롬에 저장기능 추가 (임시파일사용 안전저장기능 추가)
 * 웹페이지 변경(모바일화면) : 사용자로그인, 외부AP설정, NTP설정, 로그파일 다운로드
 * 시스템 정보표시 추가, 버튼누름 세분화(싱글, 더블, 롱클릭)
 *************************************************************/
// 업로드 실패시 ESP32 BOOT 버튼을 누른 상태로 업로드 하면 됨

#include <Arduino.h>
#include <LittleFS.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <Wire.h>

using namespace fs;     // File 사용 가능

// ================= Sensor Select =================   (택1 : USE_SHT41 / USE_AHT20_BMP280)
//#define USE_SHT41
#define USE_AHT20_BMP280

#if defined(USE_SHT41) && defined(USE_AHT20_BMP280)
#error "센서는 하나만 선택해야 합니다."
#endif

#if defined(USE_SHT41)
  #include <SensirionI2cSht4x.h>
  #define SHT41_I2C_ADDR 0x44
#elif defined(USE_AHT20_BMP280)
  #include <Adafruit_AHTX0.h>
  #include <Adafruit_BMP280.h>
#endif

#if defined(USE_SHT41)
  SensirionI2cSht4x sht4;
#elif defined(USE_AHT20_BMP280)
  Adafruit_AHTX0 aht20;
  Adafruit_BMP280 bmp280;
#endif

//#define INVALID_VALUE  -9999.0f         // float 자료형 사용시
#define INVALID_VALUE  -9999              // int16_t 자료형 사용시

#define SAMPLE_INTERVAL 1000          // 온/습도 센서
#define GRAPH_SAMPLE_INTERVAL_SEC 12  // 그래프용 샘플 간격 (10,15,20 선택 가능)
unsigned long lastGraphSampleMs = 0;   // 그래프용 마지막 push 시간


#define MAX_GRAPH_SAMPLES (24*3600/GRAPH_SAMPLE_INTERVAL_SEC)  // 24H 기준 최대 샘플 수     // 24 이라호 하면 그래프모드가 정상동작안함(system halt)

// ===== Flash Log interval (minutes) =====
#define FLASH_LOG_INTERVAL_MIN   3   // 10~1440 권장 (저장주기)

// ===== Flash Log Retention =====
#define FLASH_LOG_MAX_DAYS   30   // 보관 일수 (예: 30일)

#define FLASH_LOG_MAX_RECORDS ((FLASH_LOG_MAX_DAYS * 1 * 60) / FLASH_LOG_INTERVAL_MIN)
#define MAX_LOG_RECORDS ((FLASH_LOG_MAX_DAYS * 1 * 60) / FLASH_LOG_INTERVAL_MIN)

unsigned long lastFlashSaveMs = 0;
unsigned long lastFlashLogMs = 0;

#define LOG_FILE "/log.bin"
struct LogRecord {                        // 플래시메모리에 수집데이터 저장
  uint32_t ts;   // epoch (UTC)
  int16_t temp;    // 10배 확대된 값 (ex: 25.3°C → 253)  float -> int16_t
  int16_t humi;    // 10배 확대된 값 (ex: 60.2% → 602)   float -> int16_t
};
LogRecord logBuf[MAX_GRAPH_SAMPLES];

float currentTemp = 0;
float currentHumi = 0;

bool timeSynced = false;
bool isBufferFull = false; // New global variable


// ---------- Graph 좌표, outline 크기 등 ----------
#define GRAPH_X 40
#define GRAPH_Y 66  //64
#define GRAPH_W 240 //260           // 그래프박스 너비
#define GRAPH_H 90

#define Y_MIN 0
#define Y_MAX 80



//#define DISPLAY_HOURS 1   // 1~24시간   (그래프 X축 폭 [GRAPH_W 260] 전체 채움 소요 시간)
int displayHours = 1;
int displayHoursIndex = 0;   // 현재 선택 인덱스

unsigned long labelScrollIntervalMs;  // 초기값은 setup()에서 계산


#define NUM_LABELS 7      // X축 레이블 개수(칸갯수: n-1)

#define NUM_GRAPH_MODES 4           // 1,6,12,24H

const int displayHoursOptions[NUM_GRAPH_MODES] = {1, 6, 12, 24};


int graphIndex = 0;     // 다음 데이터 입력 위치
float accTemp = 0;
float accHumi = 0;
int accCount = 0;


unsigned long lastNtpSync = 0;                                            // ntpUpdate()
const unsigned long NTP_SYNC_INTERVAL = 24UL * 60UL * 60UL * 1000UL;      // ntpUpdate(),  1일 (ms)


#define DOUBLE_CLICK_MS 400             // 더블클릭, 일반적인 더블클릭 판정 시간
unsigned long lastReleaseTime = 0;      // 더블클릭
bool waitingSecondClick = false;        // 더블클릭




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

// ---------- Encoder 버튼 길게 누르기 감지 ----------
unsigned long encoderPressStart = 0;
const unsigned long LONG_PRESS_MS = 10000; // 10초 이상 길게 누르면 초기화


// ---------- ESP32 AP ----------
#define AP_SSID "Cage2"
#define AP_PASSWORD "cage1234"
IPAddress apIP(192,168,4,1);


// ---------- NTP ----------
char ntpServer[64]; // NTP 서버 주소를 저장할 버퍼
const long gmtOffset_sec = 9*3600;
const int daylightOffset_sec = 0;


// ======= Global Variables =======
unsigned long lastTimeAxisTick = 0;   // 그래프 + X축 레이블 공용


#define GRID_COLOR  0x4208   // 매우 어두운 회색 (RGB565)  (TFT_DARKGREY보다 약 50% 정도 어두움)


// ---------- UI layout ----------
#define TITLE_Y 5
#define INFO_Y  40
#define LED_UI_Y1 180
#define LED_UI_Y2 210

TFT_eSPI tft;

float lastTemp = NAN;
float lastHumi = NAN;

///////////////////////////////////////////////////////////////////////////////////////
// 모드별 누적 변수 (10초 단위 계산용)
float accTempAll[NUM_GRAPH_MODES] = {0};
float accHumiAll[NUM_GRAPH_MODES] = {0};
int accCountAll[NUM_GRAPH_MODES] = {0};// 모드별 1픽셀 단위 push 카운트
int pixelAccCountAll[NUM_GRAPH_MODES] = {0};
///////////////////////////////////////////////////////////////////////////////////////


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



bool readSensor(float &temp, float &humi) {

#if defined(USE_SHT41)

  uint16_t error = sht4.measureHighPrecision(temp, humi);
  return (error == 0);

#elif defined(USE_AHT20_BMP280)

  sensors_event_t humEvent, tempEvent;
  if (!aht20.getEvent(&humEvent, &tempEvent)) {
    return false;
  }

  temp = tempEvent.temperature;
  humi = humEvent.relative_humidity;
  return true;

#else
  return false;
#endif
}



float secondsPerPixel = 0;        // drawGraphData() 관련







// =========================================
// UI Functions
// =========================================
void drawTitle() {
  const int TITLE_H = 28;   // 타이틀 영역 높이
  tft.fillRect(0, TITLE_Y, tft.width(), TITLE_H, TFT_NAVY);    // 타이틀 배경 (파란색 음영)
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

  // getLocalTime() 성공은 NTP 동기화가 완료되었음을 의미하므로, 플래그를 설정합니다.
  if (!timeSynced) timeSynced = true;

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

struct TimeAxisState {
  float totalMin;
  float minPerPx;
  float labelStepMin;

  int nowMin;
  int anchorMin;
  int anchorX;
};




void computeTimeAxis(TimeAxisState &axis) {
  //axis.totalMin = DISPLAY_HOURS * 60.0f;
  axis.totalMin = displayHours * 60.0f;
  axis.minPerPx = axis.totalMin / GRAPH_W;
  axis.labelStepMin = axis.totalMin / (NUM_LABELS - 1);

  time_t nowSec;
  time(&nowSec);
  struct tm timeinfo;
  localtime_r(&nowSec, &timeinfo);

  axis.nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  axis.anchorMin = (axis.nowMin / 10) * 10;

  float anchorOffsetPx = (axis.nowMin - axis.anchorMin) / axis.minPerPx;
  axis.anchorX = GRAPH_X + GRAPH_W - (int)(anchorOffsetPx + 0.5f);
  //axis.anchorX = GRAPH_X + GRAPH_W;


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
void drawGraphFrame() {

  tft.drawRect(GRAPH_X - 1, GRAPH_Y - 1,
               GRAPH_W + 2, GRAPH_H + 2, TFT_WHITE);
}


void initLogBuffer() {
    for (int i = 0; i < MAX_GRAPH_SAMPLES; i++) {
        logBuf[i].ts = 0;
        logBuf[i].temp = INVALID_VALUE;
        logBuf[i].humi = INVALID_VALUE;
    }
    graphIndex = 0;
    accTemp = 0;
    accHumi = 0;
    accCount = 0;
    isBufferFull = false; // Initialize buffer full flag
}


int getSecondsPerPixel() {
    switch (displayHours) {
        case 1:  return 15;
        case 6:  return 90;
        case 12: return 180;
        case 24: return 360;
    }
    return 15;
}



void pushGraphData(float t, float h) {
    // NTP 시각 동기화 전에는 데이터를 저장하지 않음
    if (!timeSynced) return;

    logBuf[graphIndex].ts = time(nullptr);

    // float → int16_t 변환 (10배 확대)
    logBuf[graphIndex].temp = (t != INVALID_VALUE && !isnan(t)) ? (int16_t)(t * 10.0f) : (int16_t)INVALID_VALUE;
    logBuf[graphIndex].humi = (h != INVALID_VALUE && !isnan(h)) ? (int16_t)(h * 10.0f) : (int16_t)INVALID_VALUE;

    graphIndex = (graphIndex + 1) % MAX_GRAPH_SAMPLES;
    if (graphIndex == 0) { // Buffer wrapped around
        isBufferFull = true;
    }
}



//---------------------------------2026.1.1------------------------------

void drawGraphGrid(const TimeAxisState &axis) {
  
  const int Y_GRID_STEP = 20;   // 기존 값 유지
  // Y축
  for (int v = Y_MIN + Y_GRID_STEP; v <= Y_MAX; v += Y_GRID_STEP) {
    int y = map(v, Y_MIN, Y_MAX,
                GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.drawLine(GRAPH_X, y,
                GRAPH_X + GRAPH_W - 1, y,
                GRID_COLOR);
  }


  // X축
  for (int i = 0; i < NUM_LABELS; i++) {
    float offsetMin = (NUM_LABELS - 1 - i) * axis.labelStepMin;
    int x = axis.anchorX -
            (int)((offsetMin / axis.minPerPx) + 0.5f);

    if (x < GRAPH_X || x > GRAPH_X + GRAPH_W) continue;
    tft.drawFastVLine(x, GRAPH_Y, GRAPH_H, GRID_COLOR);
  }
}





void drawGraphLabels(const TimeAxisState &axis) {

  // 그래프 모드 라벨
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKCYAN, BG_COLOR);
  tft.setCursor(GRAPH_X + 10, GRAPH_Y + 10);
  tft.printf("%dH", displayHoursOptions[displayHoursIndex]);
  
  // Y축 라벨
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKCYAN, BG_COLOR);

  for (int v = Y_MIN + 20; v <= Y_MAX; v += 20) {
    int y = map(v, Y_MIN, Y_MAX,
                GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.setCursor(GRAPH_X - 18, y - 4);
    tft.printf("%d", v);
  }

  // Y_MIN 라벨
  int y0 = GRAPH_Y + GRAPH_H;
  tft.setCursor(GRAPH_X - 18, y0 - 4);
  tft.printf("%d", Y_MIN);


  // X축 라벨
  tft.fillRect(GRAPH_X - 12,
               GRAPH_Y + GRAPH_H + 6,
               GRAPH_W + 44,
               14,
               BG_COLOR);

  for (int i = 0; i < NUM_LABELS; i++) {
    float offsetMin = (NUM_LABELS - 1 - i) * axis.labelStepMin;
    int labelMin = axis.anchorMin - (int)(offsetMin + 0.5f);

    labelMin = (labelMin + 1440) % 1440;                // 성능 최적화

    int x = axis.anchorX -
            (int)((offsetMin / axis.minPerPx) + 0.5f);

    if (x < GRAPH_X || x > GRAPH_X + GRAPH_W) continue;

    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d",
             labelMin / 60, labelMin % 60);

    tft.setTextColor(TFT_DARKGREEN, BG_COLOR);
    tft.setCursor(x - 12, GRAPH_Y + GRAPH_H + 6);
    tft.print(buf);
  }

  tft.drawRect(GRAPH_X - 1, GRAPH_Y - 1,
               GRAPH_W + 2, GRAPH_H + 2, TFT_WHITE);
}




void drawGraphData() {
    if (!timeSynced) return;

    time_t nowTs = time(nullptr);
    int graphBottom = GRAPH_Y + GRAPH_H;
    float scale = (float)GRAPH_H / (Y_MAX - Y_MIN);

    // 이전 데이터 포인트 초기화
    int prevTempX = -1, prevTempY = -1;
    int prevHumiX = -1, prevHumiY = -1;
    uint32_t prevValidTs = 0; // 이전에 유효했던 데이터의 타임스탬프

    // logBuf를 시간 순서대로 (가장 오래된 것부터 최신까지) 순회
    // graphIndex는 다음 저장될 위치를 가리키므로, 실제 데이터는 graphIndex의 이전부터 시작
    for (int i = 0; i < MAX_GRAPH_SAMPLES; i++) {
        int idx = (graphIndex + i) % MAX_GRAPH_SAMPLES; // 가장 오래된 데이터부터 시작 (0 또는 graphIndex 바로 다음부터)
        LogRecord rec = logBuf[idx];

        // 타임스탬프가 유효하지 않거나, 이전 데이터와의 시간 간격이 너무 크면 연결을 끊음
        bool isDataGap = false;
        if (rec.ts == 0 || rec.ts == 0xFFFFFFFF) {
            isDataGap = true;
        } else if (prevValidTs != 0 && (rec.ts - prevValidTs) > (GRAPH_SAMPLE_INTERVAL_SEC * 1.5)) {
            isDataGap = true;
        }

        if (isDataGap) {
            prevTempX = -1; // 연결 끊기
            prevHumiX = -1; // 연결 끊기
            prevValidTs = 0; // 이전 유효 타임스탬프 초기화
            continue;
        }

        unsigned long secondsAgo = nowTs - rec.ts;
        if (secondsAgo > displayHours * 3600UL) { // 화면에 표시할 시간 범위를 넘어선 데이터
            prevTempX = -1; // 연결 끊기
            prevHumiX = -1; // 연결 끊기
            prevValidTs = 0; // 이전 유효 타임스탬프 초기화
            continue;
        }

        int currX = GRAPH_X + GRAPH_W - (int)((float)secondsAgo / secondsPerPixel);

        // X축 범위 밖의 데이터는 그리지 않음
        if (currX < GRAPH_X || currX >= GRAPH_X + GRAPH_W) {
            prevTempX = -1; // 연결 끊기
            prevHumiX = -1; // 연결 끊기
            prevValidTs = 0; // 이전 유효 타임스탬프 초기화
            continue;
        }

        // 온도 데이터 처리
        if (rec.temp != INVALID_VALUE) {
            int currTempY = graphBottom - (int)(((rec.temp / 10.0f) - Y_MIN) * scale + 0.5f);
            currTempY = constrain(currTempY, GRAPH_Y, graphBottom);

            if (prevTempX != -1) { // 이전 유효한 데이터가 있다면 선으로 연결
                tft.drawLine(prevTempX, prevTempY, currX, currTempY, TFT_YELLOW);
            }
            // 현재 유효한 데이터를 이전 데이터로 저장
            prevTempX = currX;
            prevTempY = currTempY;
        } else {
            prevTempX = -1; // 현재 데이터가 유효하지 않으므로 연결 끊기
        }

        // 습도 데이터 처리
        if (rec.humi != INVALID_VALUE) {
            int currHumiY = graphBottom - (int)(((rec.humi / 10.0f) - Y_MIN) * scale + 0.5f);
            currHumiY = constrain(currHumiY, GRAPH_Y, graphBottom);

            if (prevHumiX != -1) { // 이전 유효한 데이터가 있다면 선으로 연결
                tft.drawLine(prevHumiX, prevHumiY, currX, currHumiY, TFT_GREEN);
            }
            // 현재 유효한 데이터를 이전 데이터로 저장
            prevHumiX = currX;
            prevHumiY = currHumiY;
        } else {
            prevHumiX = -1; // 현재 데이터가 유효하지 않으므로 연결 끊기
        }

        // 현재 데이터가 유효했다면 prevValidTs 업데이트
        if (rec.ts != 0 && rec.ts != 0xFFFFFFFF && rec.temp != INVALID_VALUE && rec.humi != INVALID_VALUE) {
             prevValidTs = rec.ts;
        }
  }
}




void drawGraph() {
  TimeAxisState axis;
  computeTimeAxis(axis);

  tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, BG_COLOR);

  drawGraphGrid(axis);
  drawGraphData();
  drawGraphLabels(axis);
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

    // analogWrite → LEDC (PWM)
    ledcWrite(0, pwmValue[0]);
    ledcWrite(1, pwmValue[1]);


    drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
    drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);
  }

  lastEncState = encState;
}




///////////////////////// @param msg /////////////////////////////
void lcdPrint(const char* msg) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, BG_COLOR);
    tft.setCursor(50, 100);
    tft.printf(msg);
    delay(2000);
}


void updateGraphTimeScale() {
    displayHours = displayHoursOptions[displayHoursIndex];
    labelScrollIntervalMs = (displayHours * 3600UL * 1000UL) / GRAPH_W;
    secondsPerPixel = (float)(displayHours * 3600) / GRAPH_W;
}



// 1, 6, 12, 24H 그래프
void changeGraphMode() {
   displayHoursIndex = (displayHoursIndex + 1) % NUM_GRAPH_MODES;
   updateGraphTimeScale();
   drawGraph();
}



// =========================================
// System Info UI
// =========================================
void drawSystemInfo() {
  tft.fillRect(0, TITLE_Y + 28, tft.width(), tft.height() - (TITLE_Y + 28), BG_COLOR); // Clear main content area
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setCursor(10, INFO_Y);
  tft.print("System Info:");

  int y_offset = INFO_Y + 30;

  // SSID
  tft.setCursor(10, y_offset);
  tft.printf("SSID: %s", WiFi.SSID().c_str());
  y_offset += 20;

  // External IP (from connected AP)
  tft.setCursor(10, y_offset);
  tft.printf("Ext IP: %s", WiFi.localIP().toString().c_str());
  y_offset += 20;

  // Internal IP (if acting as AP, this is the softAP IP)
  tft.setCursor(10, y_offset);
  tft.printf("Int IP: %s", WiFi.softAPIP().toString().c_str());
  y_offset += 20;

  // Uptime
  unsigned long upTimeMillis = millis();
  unsigned long seconds = upTimeMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;

  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  tft.setCursor(10, y_offset);
  tft.printf("Uptime: %luD %02luH %02luM %02luS", days, hours, minutes, seconds);
  y_offset += 20;

  // NTP Sync Status
  tft.setCursor(10, y_offset);
  tft.printf("NTP Sync: %s", timeSynced ? "Synced" : "Not Synced");
  delay(3000);

  tft.fillScreen(BG_COLOR);
  drawTitle();
  drawGraphFrame();
  drawLEDUI(1, LED_UI_Y1, brightnessStep[0], true);
  drawLEDUI(2, LED_UI_Y2, brightnessStep[1], false);
  drawGraph();
}





// ================================================================
// Encoder Button : Graph Time Toggle (1,6,12,24H)
// ================================================================



/*
void handleEncoderButton() {
    bool btnCurrState = digitalRead(ENCODER_SW);
    unsigned long now = millis();

    // 버튼 누름 시작
    if (btnPrevState == HIGH && btnCurrState == LOW) {
        encoderPressStart = now;
    }

    // 버튼이 눌려있는 동안 LONG_PRESS_MS 경과 시 바로 처리
    if (btnCurrState == LOW && (now - encoderPressStart >= LONG_PRESS_MS)) {
        // 길게 누름 → 플래시 초기화
        LittleFS.remove("/log.bin");
        lcdPrint("/log.bin - removed");
        ESP.restart();                            ////////////////////// MCU리셋 추가 //  이하 의미없음
        initLogBuffer();
        graphIndex = 0;
        drawGraph();  // 그래프 초기화 표시

        // 초기화 후 encoderPressStart를 현재 시간으로 갱신해서 중복 실행 방지
        encoderPressStart = now + 1000000UL; // 충분히 큰 값으로 다음 처리까지 대기
    }

    // 버튼 떼어짐 → 짧게 누름 처리
    if (btnPrevState == LOW && btnCurrState == HIGH) {
        unsigned long pressDuration = now - encoderPressStart;
        if (pressDuration < LONG_PRESS_MS) {
            // 짧게 누름 → 기존 기능 (그래프 모드 전환)
            displayHoursIndex = (displayHoursIndex + 1) % NUM_GRAPH_MODES;
            updateGraphTimeScale();
            drawGraph();
        }
    }

    btnPrevState = btnCurrState;
}
*/


void handleEncoderButton() {
    bool btnCurrState = digitalRead(ENCODER_SW);
    unsigned long now = millis();

    // 버튼 눌림 시작
    if (btnPrevState == HIGH && btnCurrState == LOW) {
        encoderPressStart = now;
    }

    // LONG PRESS (10초 이상 눌림)
    if (btnCurrState == LOW && (now - encoderPressStart >= LONG_PRESS_MS)) {
        //longPressed();
        lcdPrint("LOG FILE ERASED");
        waitingSecondClick = false;   // 롱클릭이 발생하면 더블클릭 후보 취소
    }

    // 버튼 release
    if (btnPrevState == LOW && btnCurrState == HIGH) {
        unsigned long pressDuration = now - encoderPressStart;

        // LONG PRESS가 아니면 (즉, short or double 후보)
        if (pressDuration < LONG_PRESS_MS) {

            // 두 번째 클릭인지 확인
            if (waitingSecondClick &&
                (now - lastReleaseTime <= DOUBLE_CLICK_MS)) {

                // === DOUBLE CLICK ===
                drawSystemInfo();
                waitingSecondClick = false;

            } else {
                // 첫 번째 클릭 → 더블클릭 대기 상태로 진입
                waitingSecondClick = true;
                lastReleaseTime = now;
            }
        }
    }

    // 더블클릭 대기 시간이 지났으면 SHORT CLICK 확정
    if (waitingSecondClick &&
        (now - lastReleaseTime > DOUBLE_CLICK_MS)) {
        // SHORT CLICK
        changeGraphMode();
        waitingSecondClick = false;
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

void handleRoot(bool error = false) {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Login</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, system-ui, BlinkMacSystemFont, "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; }
        .error { color: #d9534f; text-align: center; margin-bottom: 1em; font-weight: bold; }
        label { display: block; margin-bottom: 0.5em; font-weight: bold; }
        input[type='text'], input[type='password'] { width: 100%; padding: 0.8em; margin-bottom: 1em; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
        input[type='submit'] { width: 100%; background-color: #007bff; color: white; padding: 1em; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; font-weight: bold; }
        input[type='submit']:hover { background-color: #0056b3; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Device Login</h2>
)rawliteral";
  if (error) {
    page += "<p class='error'>Invalid username or password.</p>";
  }
  page += R"rawliteral(
        <form method='POST' action='/login'>
            <label for='username'>Username:</label>
            <input id='username' name='username' type='text' value='cage' required>
            <label for='password'>Password:</label>
            <input id='password' name='password' type='password' required>
            <br><br>
            <input type='submit' value='Login'>
        </form>
    </div>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", page);
}

void handleLogin() {
    if (server.hasArg("username") && server.arg("username") == "cage" && server.arg("password") == "cage1234") {
        String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, system-ui, BlinkMacSystemFont, "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }H2 { text-align: center; color: #007bff; margin-bottom: 1.5em; }
        .menu-button { display: block; width: calc(100% - 2em); padding: 1em; margin: 1em auto; background-color: #007bff; color: white; text-decoration: none; border-radius: 4px; text-align: center; font-weight: bold; font-size: 1.1em; }
        .menu-button:hover { background-color: #0056b3; }
        .download-button { background-color: #28a745; }
        .download-button:hover { background-color: #218838; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Device Dashboard</h2>
        <a href="/config" class="menu-button">Configure WiFi</a>
        <a href="/ntpconfig" class="menu-button">Configure NTP</a>
        <a href="/downloadlog" class="menu-button download-button">Download Log File</a>
    </div>
</body>
</html>
)rawliteral";
        server.send(200, "text/html", page);
    } else {
        handleRoot(true); // Show login page with error
    }
}

void handleNTPConfig() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>NTP Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, system-ui, BlinkMacSystemFont, "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; }
        p.info { text-align: center; margin-bottom: 2em; color: #666; font-size: 0.9em; }
        fieldset { border: 1px solid #ddd; border-radius: 8px; padding: 1.5em; margin-bottom: 2em; }
        legend { font-size: 1.2em; font-weight: bold; color: #007bff; padding: 0 0.5em; }
        label { display: block; margin-bottom: 0.5em; font-weight: bold; }
        input[type='text'], input[type='password'] { width: 100%; padding: 0.8em; margin-bottom: 1em; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
        input[type='radio'] { margin-right: 0.5em; }
        input[type='submit'] { width: 100%; background-color: #28a745; color: white; padding: 1em; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; font-weight: bold; }
        input[type='submit']:hover { background-color: #218838; }
        #customNtpServerDiv { margin-top: 1em; }
    </style>
</head>
<body>
    <div class="container">
        <h2>NTP Server Configuration</h2>
        <form method='POST' action='/ntpsave'>
            <fieldset>
                <legend>NTP Server Settings</legend>
                <p class="info">Choose your NTP server:</p>
                <div style="display: flex; align-items: center; margin-bottom: 1em;">
                    <input type="radio" id="ntp_builtin1" name="ntpMode" value="builtin1" %BUILTIN1_CHECKED%>
                    <label for="ntp_builtin1" style="margin-bottom: 0;">time.bora.net (default)</label>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 1em;">
                    <input type="radio" id="ntp_builtin2" name="ntpMode" value="builtin2" %BUILTIN2_CHECKED%>
                    <label for="ntp_builtin2" style="margin-bottom: 0;">time.kriss.re.kr</label>
                </div>
                <div style="display: flex; align-items: center; margin-top: 1em;">
                    <input type="radio" id="ntp_custom" name="ntpMode" value="custom" %CUSTOM_CHECKED%>
                    <input id='customNtpServer' name='customNtpServer' type='text' placeholder="e.g., pool.ntp.org" style="flex-grow: 1; margin-left: 0.5em;">
                </div>
            </fieldset>
            <br><br>
            <input type='submit' value='Save & Reboot'>
        </form>

    </div>
</body>
</html>
)rawliteral";

  preferences.begin("wifi", true); // 읽기 전용
  String savedNtpMode = preferences.getString("ntpMode", "builtin1"); // 기본값 "builtin1"
  String savedCustomNtp = preferences.getString("ntpServer", ""); // 기본값 없음
  preferences.end();

  page.replace("%CUSTOM_NTP_VALUE%", savedCustomNtp);

  if (savedNtpMode == "builtin1") {
    page.replace("%BUILTIN1_CHECKED%", "checked");
    page.replace("%BUILTIN2_CHECKED%", "");
    page.replace("%CUSTOM_CHECKED%", "");
  } else if (savedNtpMode == "builtin2") {
    page.replace("%BUILTIN1_CHECKED%", "");
    page.replace("%BUILTIN2_CHECKED%", "checked");
    page.replace("%CUSTOM_CHECKED%", "");
  } else { // custom
    page.replace("%BUILTIN1_CHECKED%", "");
    page.replace("%BUILTIN2_CHECKED%", "");
    page.replace("%CUSTOM_CHECKED%", "checked");
  }

  server.send(200, "text/html", page);
}

void handleConfig() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, system-ui, BlinkMacSystemFont, "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; }
        p.info { text-align: center; margin-bottom: 2em; color: #666; font-size: 0.9em; }
        fieldset { border: 1px solid #ddd; border-radius: 8px; padding: 1.5em; margin-bottom: 2em; }
        legend { font-size: 1.2em; font-weight: bold; color: #007bff; padding: 0 0.5em; }
        label { display: block; margin-bottom: 0.5em; font-weight: bold; }
        input[type='text'], input[type='password'] { width: 100%; padding: 0.8em; margin-bottom: 1em; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
        input[type='radio'] { margin-right: 0.5em; }
        input[type='submit'] { width: 100%; background-color: #28a745; color: white; padding: 1em; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; font-weight: bold; }
        input[type='submit']:hover { background-color: #218838; }
        #customNtpServerDiv { margin-top: 1em; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Device Configuration</h2>
        <form method='POST' action='/save'>
            <fieldset>
                <legend>WiFi Settings</legend>
                <p class="info">Enter the details for your WiFi network to allow this device to connect to the internet.</p>
                <label for='ssid'>Your WiFi Name (SSID):</label>
                <input id='ssid' name='ssid' type='text' required value='%SSID_VALUE%'>
                <label for='password'>Your WiFi Password:</label>
                <input id='password' name='password' type='password' value='%PASSWORD_VALUE%'>
            </fieldset>

            <br><br>
            <input type='submit' value='Save & Reboot'>
        </form>
    </div>
</body>
</html>
)rawliteral";

  preferences.begin("wifi", true); // 읽기 전용

  String savedSsid = preferences.getString("ssid", "");
  String savedPass = preferences.getString("password", "");

  preferences.end();

  page.replace("%SSID_VALUE%", savedSsid);
  page.replace("%PASSWORD_VALUE%", savedPass);

  server.send(200, "text/html", page);
}

void handleNTPSave() {
  String ntpMode = server.arg("ntpMode");
  String customNtp = server.arg("customNtpServer");

  preferences.begin("wifi", false); // 쓰기 모드로 열기
  preferences.putString("ntpMode", ntpMode); // NTP 모드 저장
  if (ntpMode == "custom") {
    preferences.putString("ntpServer", customNtp); // 커스텀 NTP 서버 주소 저장
  } else {
    // 커스텀 모드가 아니면 저장된 커스텀 NTP 서버 주소는 지우는게 깔끔함
    preferences.remove("ntpServer");
  }
  preferences.end(); // Preferences 닫기

  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Saved</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, system-ui, BlinkMacSystemFont, "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; text-align: center; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        p { font-size: 1.2em; }
    </style>
</head>
<body>
    <div class="container">
        <p>Saved successfully!</p>
        <p>The device will now reboot...</p>
    </div>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", page);
  delay(2000);
  ESP.restart();
}

void handleSave() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String ssid = server.arg("ssid");

    if (ssid.length() == 0) {
      String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Error</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, system-ui, BlinkMacSystemFont, "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; text-align: center; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        p { font-size: 1.2em; color: #d9534f; }
        a { color: #007bff; text-decoration: none; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <p>SSID cannot be empty.</p>
        <p><a href="/">Please go back and try again.</a></p>
    </div>
</body>
</html>
)rawliteral";
      server.send(400, "text/html", page);
      return;
    }

    String pass = server.arg("password");

    preferences.begin("wifi", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", pass);
    preferences.end();

    String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Saved</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, system-ui, BlinkMacSystemFont, "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; text-align: center; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        p { font-size: 1.2em; }
    </style>
</head>
<body>
    <div class="container">
        <p>Saved successfully!</p>
        <p>The device will now reboot...</p>
    </div>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", page);
    delay(2000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "400: Invalid Request");
  }
}


// ================== Save/Load Data (Flash) ==================

void saveLogToFlash() {
    // NTP 동기화가 안됐으면 저장하지 않음
    if (!timeSynced) {
        return;
    }

    const char* tmpLogFile = "/log.bin.tmp";

    // 1. 임시 파일에 현재 버퍼 상태를 쓴다.
    File f = LittleFS.open(tmpLogFile, "w");
    if (!f) {
        return;
    }

    size_t bytesWritten = f.write((uint8_t*)logBuf, sizeof(logBuf));
    bytesWritten += f.write((uint8_t*)&graphIndex, sizeof(graphIndex));
    bytesWritten += f.write((uint8_t*)&isBufferFull, sizeof(isBufferFull)); // Save the new flag
    f.close();

    // 2. 쓰기가 성공했는지 확인한다.
    // Updated expected size
    if (bytesWritten != sizeof(logBuf) + sizeof(graphIndex) + sizeof(isBufferFull)) {
        LittleFS.remove(tmpLogFile); // 실패한 임시 파일 삭제
        return;
    }

    // 3. 기존 로그 파일을 삭제하고 임시 파일의 이름을 변경한다. (원자적 업데이트)
    LittleFS.remove(LOG_FILE);
    LittleFS.rename(tmpLogFile, LOG_FILE);
}

void loadLogFromFlash() {
    const char* tmpLogFile = "/log.bin.tmp";
    // 부팅 시 임시 파일이 남아있다면 이전 쓰기 작업이 실패한 것이므로 삭제
    if (LittleFS.exists(tmpLogFile)) {
        LittleFS.remove(tmpLogFile);
    }

    if (!LittleFS.exists(LOG_FILE)) {
        initLogBuffer();
        return;
    }

    File f = LittleFS.open(LOG_FILE, "r");
    if (!f) {
        initLogBuffer();
        return;
    }

    // 파일 크기 검사 - Updated expected size
    if (f.size() != sizeof(logBuf) + sizeof(graphIndex) + sizeof(isBufferFull)) {
        f.close();
        initLogBuffer();
        // 손상된 파일일 수 있으므로 삭제하여 다음 부팅 시 새로 시작하도록 함
        LittleFS.remove(LOG_FILE);
        return;
    }
    
    size_t bytesRead = f.read((uint8_t*)logBuf, sizeof(logBuf));
    bytesRead += f.read((uint8_t*)&graphIndex, sizeof(graphIndex));
    bytesRead += f.read((uint8_t*)&isBufferFull, sizeof(isBufferFull)); // Load the new flag
    f.close();

    // 읽기 실패 시 버퍼 초기화
    // Updated expected size
    if (bytesRead != sizeof(logBuf) + sizeof(graphIndex) + sizeof(isBufferFull)) {
        initLogBuffer();
    }
}



void handleDownloadLog() {
  // It's possible that LittleFS.exists(LOG_FILE) is true but the logBuf is empty
  // if initLogBuffer() was called due to file corruption/size mismatch.
  // The CSV generation logic handles empty buffers correctly by only outputting the header.

  // Create CSV content
  String csvContent = "Timestamp,Temperature(C),Humidity(%),Epoch\n";

  int startIndex;
  int count;

  // Determine the valid range in the circular buffer
  if (isBufferFull) {
    startIndex = graphIndex; // Start from the oldest entry
    count = MAX_GRAPH_SAMPLES;
  } else {
    startIndex = 0;
    count = graphIndex; // Up to the current index
  }

  for (int i = 0; i < count; i++) {
    int currentIdx = (startIndex + i) % MAX_GRAPH_SAMPLES;
    LogRecord rec = logBuf[currentIdx];

    // Only include valid records (ts != 0, temp/humi != INVALID_VALUE)
    if (rec.ts == 0 || rec.temp == INVALID_VALUE || rec.humi == INVALID_VALUE) {
        continue;
    }

    struct tm *timeinfo;
    time_t rawtime = rec.ts;
    timeinfo = localtime(&rawtime);

    char timeStr[20]; // YYYY-MM-DD HH:MM:SS
    // strftime returns 0 on failure, so ensure it worked before using timeStr
    if (strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", timeinfo) == 0) {
        strcpy(timeStr, "Invalid Time");
    }

    csvContent += String(timeStr);
    csvContent += ",";
    csvContent += String(rec.temp / 10.0f, 1); // 1 decimal place
    csvContent += ",";
    csvContent += String(rec.humi / 10.0f, 1); // 1 decimal place
    csvContent += ",";
    csvContent += String(rec.ts); // Epoch timestamp
    csvContent += "\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=\"log.csv\"");
  server.sendHeader("Content-Type", "text/csv; charset=utf-8"); // Added charset
  server.sendHeader("Content-Length", String(csvContent.length()));
  server.send(200, "text/csv", csvContent);
}

void ntpUpdate() {
    unsigned long now = millis();
    if (now - lastNtpSync < NTP_SYNC_INTERVAL) return;  // 1일이 안되면 return
    lastNtpSync = now;
    if (WiFi.status() != WL_CONNECTED) {
        // WiFi 없음 → loop 계속
        return;
    }

    // NTP 시각 동기 (실패시 자동리셋)
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    unsigned long start = millis();
    while (!getLocalTime(&timeinfo)) {
        if (millis() - start > 5000) {  // 5초 시각동기 timeout
            ESP.restart();  // 자동 리셋
        }
        delay(100);
    }
  timeSynced = true;
}

// =========================================
void setup() {
  preferences.begin("wifi", false); // Preferences를 읽기/쓰기 모드로 열기

  String savedSsid = preferences.getString("ssid", "");
  String savedPass = preferences.getString("password", "");
  String savedNtpMode = preferences.getString("ntpMode", "builtin1"); // 기본값 "builtin1"
  String savedCustomNtp = preferences.getString("ntpServer", "time.bora.net"); // 기본값 "time.bora.net"
  preferences.end(); // Preferences 닫기 (setup에서 한번만 열고 닫도록)

  // NTP 서버 설정
  if (savedNtpMode == "builtin1") {
    strcpy(ntpServer, "time.bora.net");
  } else if (savedNtpMode == "builtin2") {
    strcpy(ntpServer, "time.kriss.re.kr");
  } else { // custom
    savedCustomNtp.toCharArray(ntpServer, sizeof(ntpServer));
  }

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);

  //LittleFS.begin(true);
  if (!LittleFS.begin(true)) {
    lcdPrint("LittleFS mount failed");
    ESP.restart();
  }

  // 초기 인코더 상태 (CLK + DT)
  lastEncState = (digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  // PWM 설정(Rotyry Encoder)
  ledcSetup(0,5000,8); ledcAttachPin(LED1_PIN,0);
  ledcSetup(1,5000,8); ledcAttachPin(LED2_PIN,1);

  updateGraphTimeScale();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(BG_COLOR);

  //----------------------------------------------------------------------
  Wire.begin(I2C_SDA, I2C_SCL);
  #if defined(USE_SHT41)
    sht4.begin(Wire, SHT41_I2C_ADDR);
    delay(10);
    sht4.softReset();

  #elif defined(USE_AHT20_BMP280)
    if (!aht20.begin()) {
      //Serial.println("AHT20 not found");
    }
    if (!bmp280.begin(0x76)) {   // 보드에 따라 0x76 / 0x77
      //Serial.println("BMP280 not found");
    }

  #endif
  //----------------------------------------------------------------------

  drawTitle();
  initLogBuffer();        // RAM 초기화
  loadLogFromFlash();  // ← Flash에 저장된 데이터로 버퍼 초기화
  drawGraphFrame();
  drawGraph();

  drawLEDUI(1, LED_UI_Y1, brightnessStep[0], true);
  drawLEDUI(2, LED_UI_Y2, brightnessStep[1], false);


  startAP();
  server.on("/", HTTP_GET, [](){ handleRoot(false); });
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/config", HTTP_GET, handleConfig); // 추가
  server.on("/ntpconfig", HTTP_GET, handleNTPConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/ntpsave", HTTP_POST, handleNTPSave);
  server.on("/downloadlog", HTTP_GET, handleDownloadLog);
  server.begin();

  // connectExternalAP() 내부에서 preferences를 다시 열지 않도록 수정
  // ssid, pass는 savedSsid, savedPass를 사용
  if (savedSsid.length() > 0) {
    WiFi.begin(savedSsid.c_str(), savedPass.c_str());
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
void loop() {
  server.handleClient();

  handleEncoderRotation();
  handleEncoderButton();

  ntpUpdate();

  unsigned long nowMs = millis();

  // ================== Sensor Sampling (1초) ==================
  if (nowMs - lastSample >= SAMPLE_INTERVAL) {
      lastSample = nowMs;

      float temperature = NAN;
      float humidity = NAN;

      if (readSensor(temperature, humidity)) {
          lastTemp = temperature;
          lastHumi = humidity;

          accTemp += temperature;
          accHumi += humidity;
          accCount++;

      } else {
          lastTemp = NAN;
          lastHumi = NAN;
      }

      drawTimeTempHumi(lastTemp, lastHumi, true);
  }

  if (nowMs - lastGraphSampleMs >= GRAPH_SAMPLE_INTERVAL_SEC * 1000UL) {
      lastGraphSampleMs = nowMs;

      float avgTemp = (accCount > 0) ? (accTemp / accCount) : NAN;
      float avgHumi = (accCount > 0) ? (accHumi / accCount) : NAN;

      // 1) 그래프 버퍼에 저장
      pushGraphData(avgTemp, avgHumi);

      // 3) 누적값 초기화
      accTemp = 0;
      accHumi = 0;
      accCount = 0;

      // 현재 display 모드 redraw
      drawGraph();
  }
  
  /*
  // ================== Flash 저장 주기 ==================
  if (millis() - lastFlashSaveMs >= SAVE_INTERVAL_MIN * 60UL * 1000UL) {
      lastFlashSaveMs = millis();
      writeFlashLog(currentTemp, currentHumi);
      //writeFlashLog();  // ← 여기: RAM logBuf → Flash 저장
  }
  */


  // ================== Save Data ==================
  if (nowMs - lastFlashSaveMs >= FLASH_LOG_INTERVAL_MIN * 60UL * 1000UL) {
    lastFlashSaveMs = nowMs;
    // 2) Flash 저장
    saveLogToFlash();  // ← 여기서 Flash에 현재 logBuf[]를 저장, timeSynced가 true일 때 저장가능
  }

}
