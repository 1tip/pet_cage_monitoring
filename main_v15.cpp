/**************************************************************	
 * Lizard Cage Monitoring (Stable + Scroll Graph + External AP + Long-term Graph)	
 * ESP32 + TFT_eSPI + DHT22 + Rotary Encoder	
 * 온습도 센서 선택가능 : SHT41, AHT20_BMP280	
 * LED UI + REAL PWM LED control (2ch)	
 * GRAPH_INTERVAL 적용 + WebServer 외부 AP 설정	
 * 장시간 평균 기반 그래프 (1~24시간 X축) + 가변 타임스탬프	
 * 그래프모드 추가(1,6,12,24H)	
 * 플래시롬에 저장기능 추가 (Append 방식, Flash 수명 개선)	
 * 웹페이지 변경(모바일화면) : 사용자로그인, 외부AP설정, NTP설정, 로그파일(CSV) 다운로드	
 * 시스템 정보표시 추가, 버튼누름 세분화(싱글, 더블, 롱클릭)	
 * NTP설정 버그 해결	
 * lcd LANDSCAPE/PORTRAIT 설정	
 * I2C 센서 통신 블로킹 해소 - Wire.setTimeout(1000) 설정
 * 웹페이지 원격제어 항목 추가(가습기 운전모드 AUTO, ON, OFF)
 * 시스템 정보에 사용자 설정값(웹페이지) 보기 추가
 * 웹 비동기통신(AJAX) 방식을 이용해 실시간 온,습도를 보여줌
 * 로그파일 다운로드 버그 수정 - yield()함수 사용
 * 가습기 제어 상태값 저장(비휘발성메모리)
 ************************************************************/	


#include <Arduino.h>
#include <LittleFS.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <Wire.h>

using namespace fs;     // File 사용 가능

enum HumidifierMode { AUTO, ON, OFF };
HumidifierMode humidifierMode = AUTO;
unsigned long manualHumidifierStartTime = 0;

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

int humiMax = 60;                   // 최고습도 조건(가습기 정지)
int humiMin = 45;                   // 최저습도 조건(가습기 동작)

#define HUMIDIFIER_AUTO_OFF_MINUTES 1 // 가습기 수동 ON 후 자동 OFF 시간 (분)

#if defined(USE_SHT41)
  SensirionI2cSht4x sht4;
#elif defined(USE_AHT20_BMP280)
  Adafruit_AHTX0 aht20;
  Adafruit_BMP280 bmp280;
#endif

#define INVALID_VALUE  -9999              // int16_t 자료형 사용시

#define SAMPLE_INTERVAL 1000          // 온/습도 센서
#define GRAPH_SAMPLE_INTERVAL_SEC 12  // 그래프용 샘플 간격(초) (10,15,20 선택 가능)
unsigned long lastGraphSampleMs = 0;   // 그래프용 마지막 push 시간

// ================== Log Configuration ==================
// ===== RAM Buffer for TFT Display (24 Hours) =====
#define DISPLAY_MAX_HOURS 24
#define DISPLAY_MAX_SAMPLES (DISPLAY_MAX_HOURS * 3600 / GRAPH_SAMPLE_INTERVAL_SEC)

// ===== Flash Log Retention (15 Days) =====
#define FLASH_LOG_MAX_DAYS 15
#define FLASH_MAX_RECORDS (FLASH_LOG_MAX_DAYS * 24 * 3600 / GRAPH_SAMPLE_INTERVAL_SEC)

#define LOG_FILE "/log.bin"
#define META_FILE "/log.meta"

struct LogRecord {
  uint32_t ts;   // epoch (UTC)
  int16_t temp;    // 10배 확대된 값 (ex: 25.3°C → 253)
  int16_t humi;    // 10배 확대된 값 (ex: 60.2% → 602)
};

struct LogMeta {
  uint32_t head_index;     // 다음 레코드가 기록될 위치 (포인터)
  uint32_t record_count;   // 현재 저장된 총 레코드 수
};

// In-memory buffer for display
LogRecord displayLogBuf[DISPLAY_MAX_SAMPLES];
int displayLogIndex = 0;
bool isDisplayBufferFull = false;

// Flash metadata
LogMeta logMeta;


float currentTemp = 0;
float currentHumi = 0;

bool timeSynced = false;

// ---------- ESP32 AP ----------
#define AP_SSID "Cage2"
#define AP_PASSWORD "cage1234"
IPAddress apIP(192,168,2,1);

/////////////////////////////////////////////////////////////////////////////////////
//#define LANDSCAPE           // portrait(0),  landscape(1)
#define PORTRAIT           // portrait(0),  landscape(1)
/////////////////////////////////////////////////////////////////////////////////////

// ---------- Graph 좌표(Landscape), outline 크기 등 ----------
#if defined(LANDSCAPE)            // LCD (landscape)
#define SCREEN_ROTATION 1
#define GRAPH_X 40
#define GRAPH_Y 66  //64
#define GRAPH_W 240 //260           // 그래프박스 너비
#define GRAPH_H 90
#define HEADLINE "  Lizard Cage Monitoring"
#define NUM_LABELS 7      // X축 레이블 개수(칸갯수: n-1)

// ---------- Graph 좌표(Portrait), outline 크기 등 ----------
#elif defined(PORTRAIT)           // LCD (portrait)
#define SCREEN_ROTATION 0
#define GRAPH_X 18
#define GRAPH_Y 66  //64
#define GRAPH_W 210//240 //260           // 그래프박스 너비
#define GRAPH_H 90
#define HEADLINE "Lizard Cage Monitor"
#define NUM_LABELS 6      // X축 레이블 개수(칸갯수: n-1)

#else
#error "PORTRAIT, LANDSCAPE 중 하나를 정의하시오"
#endif

#define Y_MIN 0
#define Y_MAX 80

int displayHours = 1;
int displayHoursIndex = 0;   // 현재 선택 인덱스

unsigned long labelScrollIntervalMs;  // 초기값은 setup()에서 계산

#define NUM_GRAPH_MODES 4           // 1,6,12,24H
const int displayHoursOptions[NUM_GRAPH_MODES] = {1, 6, 12, 24};

int graphIndex = 0;     // 다음 데이터 입력 위치
float accTemp = 0;
float accHumi = 0;
int accCount = 0;

unsigned long lastNtpSync = 0;                                            // ntpUpdate()
const unsigned long NTP_SYNC_INTERVAL = 24UL * 60UL * 60UL * 1000UL;      // ntpUpdate(),  1일 (ms)

#define DOUBLE_CLICK_MS 300             // 더블클릭, 일반적인 더블클릭 판정 시간 (400ms)
unsigned long sysInfoDisplayUntil = 0;
byte btnClickCount = 0;
unsigned long btnReleaseTime = 0;
bool longPressActive = false;

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
#define HUMIDIFIER_PWR 15       // 가습기 전원
#define HUMIDIFIER_RUN 2        // 가습기 작동버튼

#else
#error "HW_VER_1 또는 HW_VER_2 중 하나를 정의하시오"
#endif


int brightnessStep[2] = {0, 0};
int selectedLED = 0;
uint8_t lastEncState = 0;
bool btnPrevState = HIGH;
unsigned long encoderPressStart = 0;
const unsigned long LONG_PRESS_MS = 10000;

// ---------- NTP ----------
char ntpServer[64];
const long gmtOffset_sec = 9*3600;
const int daylightOffset_sec = 0;

unsigned long lastTimeAxisTick = 0;
#define GRID_COLOR  0x4208
#define TITLE_Y 5
#define INFO_Y  40
#define LED_UI_Y1 180
#define LED_UI_Y2 210

TFT_eSPI tft;

float lastTemp = NAN;
float lastHumi = NAN;
int pwmValue[2] = {0, 0};
unsigned long lastSample = 0;

WebServer server(80);
Preferences preferences;
bool externalAPConnected = false;

// =========================================
// Forward Declarations
// =========================================
void lcdPrint(const char* msg);
void drawGraph();
void checkHumidity();
void handleDashboard();

// =========================================
// Logging Functions (New Append-based Logic)
// =========================================

void writeMeta() {
  File metaFile = LittleFS.open(META_FILE, "w");
  if (metaFile) {
    metaFile.write((uint8_t*)&logMeta, sizeof(LogMeta));
    metaFile.close();
  }
}

void initFlashStorage() {
  if (!LittleFS.begin(true)) {
    lcdPrint("LittleFS Mount Failed!");
    delay(5000);
    ESP.restart();
  }

  // Check and initialize metadata file
  if (LittleFS.exists(META_FILE)) {
    File metaFile = LittleFS.open(META_FILE, "r");
    if (metaFile && metaFile.size() == sizeof(LogMeta)) {
      metaFile.read((uint8_t*)&logMeta, sizeof(LogMeta));
    } else {
      // File is corrupted or wrong size, re-initialize
      logMeta.head_index = 0;
      logMeta.record_count = 0;
    }
    metaFile.close();
  } else {
    logMeta.head_index = 0;
    logMeta.record_count = 0;
    writeMeta();
  }
  
  // Check log file, if it doesn't exist, create an empty one.
  // The file will grow as records are appended.
  // For a strict circular buffer, pre-allocating would be an option,
  // but letting it grow and managing it via head_index is simpler.
  if (!LittleFS.exists(LOG_FILE)) {
    File logFile = LittleFS.open(LOG_FILE, "w");
    if (logFile) {
      logFile.close();
    }
  }
}

void appendLogRecord(LogRecord& newRecord) {
  if (!timeSynced) return;

  File logFile = LittleFS.open(LOG_FILE, "r+");
  if (!logFile) {
    logFile = LittleFS.open(LOG_FILE, "w+");
  }

  if (logFile) {
    logFile.seek(logMeta.head_index * sizeof(LogRecord));
    logFile.write((uint8_t*)&newRecord, sizeof(LogRecord));
    logFile.close();

    logMeta.head_index = (logMeta.head_index + 1) % FLASH_MAX_RECORDS;
    if (logMeta.record_count < FLASH_MAX_RECORDS) {
      logMeta.record_count++;
    }
    writeMeta();
  }
}

void readLogRecord(LogRecord& record, uint32_t index) {
  File logFile = LittleFS.open(LOG_FILE, "r");
  if (logFile) {
    if (index < logMeta.record_count) {
      logFile.seek(index * sizeof(LogRecord));
      logFile.read((uint8_t*)&record, sizeof(LogRecord));
    } else {
      record.ts = 0; // Invalid record
    }
    logFile.close();
  } else {
    record.ts = 0; // Invalid record
  }
}

void initDisplayBuffer() {
    for (int i = 0; i < DISPLAY_MAX_SAMPLES; i++) {
        displayLogBuf[i].ts = 0;
        displayLogBuf[i].temp = INVALID_VALUE;
        displayLogBuf[i].humi = INVALID_VALUE;
    }
    displayLogIndex = 0;
    isDisplayBufferFull = false;
}

void loadDataForDisplay() {
  initDisplayBuffer();
  
  uint32_t records_to_read = min((uint32_t)DISPLAY_MAX_SAMPLES, logMeta.record_count);
  if (records_to_read == 0) return;

  uint32_t start_index = (logMeta.head_index + FLASH_MAX_RECORDS - records_to_read) % FLASH_MAX_RECORDS;

  File logFile = LittleFS.open(LOG_FILE, "r");
  if (!logFile) return;

  for (uint32_t i = 0; i < records_to_read; i++) {
    uint32_t current_flash_index = (start_index + i) % FLASH_MAX_RECORDS;
    logFile.seek(current_flash_index * sizeof(LogRecord));

    LogRecord rec;
    size_t read_size = logFile.read((uint8_t*)&rec, sizeof(LogRecord));

    if (read_size == sizeof(LogRecord) && rec.ts != 0) {
      displayLogBuf[displayLogIndex] = rec;
      displayLogIndex = (displayLogIndex + 1) % DISPLAY_MAX_SAMPLES;
      if (displayLogIndex == 0) isDisplayBufferFull = true;
    }
  }
  logFile.close();
}

void pushToDisplayBuffer(float t, float h) {
    if (!timeSynced) return;

    displayLogBuf[displayLogIndex].ts = time(nullptr);
    displayLogBuf[displayLogIndex].temp = (t != INVALID_VALUE && !isnan(t)) ? (int16_t)(t * 10.0f) : (int16_t)INVALID_VALUE;
    displayLogBuf[displayLogIndex].humi = (h != INVALID_VALUE && !isnan(h)) ? (int16_t)(h * 10.0f) : (int16_t)INVALID_VALUE;
    
    // Append to flash
    appendLogRecord(displayLogBuf[displayLogIndex]);

    displayLogIndex = (displayLogIndex + 1) % DISPLAY_MAX_SAMPLES;
    if (displayLogIndex == 0) {
        isDisplayBufferFull = true;
    }
}


// =========================================
// UI Functions
// =========================================
int stepToPWM(int step) { return map(step, 0, BRIGHTNESS_STEPS, 0, 255); }

bool readSensor(float &temp, float &humi) {

#if defined(USE_SHT41)
  uint16_t error = sht4.measureHighPrecision(temp, humi);
  return (error == 0);

#elif defined(USE_AHT20_BMP280)
  sensors_event_t humEvent, tempEvent;
  if (!aht20.getEvent(&humEvent, &tempEvent)) return false;
  temp = tempEvent.temperature;
  humi = humEvent.relative_humidity;
  return true;

#else
  return false;

#endif
}

float secondsPerPixel = 0;

void drawTitle() {
  tft.fillScreen(BG_COLOR);                                       ///////
  const int TITLE_H = 28;
  tft.fillRect(0, TITLE_Y, tft.width(), TITLE_H, TFT_NAVY);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextSize(2);
  tft.setCursor(0, TITLE_Y + 6);
  tft.print(HEADLINE);
  tft.drawFastHLine(0, TITLE_Y + TITLE_H - 1, tft.width(), TFT_DARKGREY);
}

void drawTimeTempHumi(float t, float h, bool timeAvailable) {
  tft.fillRect(0, INFO_Y, 240, 20, BG_COLOR);
  struct tm timeinfo;
  bool isTimeValid = getLocalTime(&timeinfo, 0) && timeinfo.tm_year > (2020 - 1900);

  if (isTimeValid) {
    if (!timeSynced) { timeSynced = true; lastNtpSync = millis(); }
    tft.setTextSize(2);
    tft.setCursor(0, INFO_Y);
    tft.setTextColor(TFT_BLUE, BG_COLOR);
    tft.printf("%02d:%02d:%02d ", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    tft.setTextColor(TFT_YELLOW, BG_COLOR);
    tft.printf("%4.1fC ", t);
    tft.setTextColor(TFT_GREEN, BG_COLOR);
    tft.printf("%4.1f%%", h);
  } else {
    if (timeSynced) { timeSynced = false; }
    tft.setTextSize(1);
    tft.setCursor(5, INFO_Y + 4);
    tft.setTextColor(TFT_ORANGE, BG_COLOR);
    if (WiFi.status() != WL_CONNECTED) {
        tft.print("WiFi Connecting...");
    } else {
        char msg[40];
        snprintf(msg, sizeof(msg), "NTP Syncing via %s", ntpServer);
        tft.print(msg);
    }
  }
}

struct TimeAxisState {
  float totalMin, minPerPx, labelStepMin;
  int nowMin, anchorMin, anchorX;
};

void computeTimeAxis(TimeAxisState &axis) {
  axis.totalMin = displayHours * 60.0f;
  axis.minPerPx = axis.totalMin / GRAPH_W;
  axis.labelStepMin = axis.totalMin / (NUM_LABELS - 1);
  time_t nowSec; time(&nowSec); struct tm timeinfo; localtime_r(&nowSec, &timeinfo);
  axis.nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  axis.anchorMin = (axis.nowMin / 10) * 10;
  float anchorOffsetPx = (axis.nowMin - axis.anchorMin) / axis.minPerPx;
  axis.anchorX = GRAPH_X + GRAPH_W - (int)(anchorOffsetPx + 0.5f);
}

void drawLEDUI(int led, int y, int level, bool selected) {
  int barX = 90, barW = 120, barH = 16;

#if defined(LANDSCAPE)
  tft.fillRect(0, y, 240, 28, BG_COLOR);
  tft.setTextSize(2); tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setCursor(10, y);
  if (selected) tft.print("* "); else tft.print("  ");
  tft.printf("LED%d", led);
  tft.drawRect(barX, y, barW, barH, TFT_WHITE);
  int fillW = map(level, 0, BRIGHTNESS_STEPS, 0, barW - 2);
  if (fillW > 0) tft.fillRect(barX + 1, y + 1, fillW, barH - 2, TFT_GREEN);

#elif defined(PORTRAIT)
  tft.fillRect(0, y+90, 240, 28, BG_COLOR);
  tft.setTextSize(2); tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setCursor(10, y+90);
  if (selected) tft.print("* "); else tft.print("  ");
  tft.printf("LED%d", led);
  tft.drawRect(barX, y+90, barW, barH, TFT_WHITE);
  int fillW = map(level, 0, BRIGHTNESS_STEPS, 0, barW - 2);
  if (fillW > 0) tft.fillRect(barX + 1, y+90 + 1, fillW, barH - 2, TFT_GREEN);
#endif
}

void drawGraphFrame() { tft.drawRect(GRAPH_X - 1, GRAPH_Y - 1, GRAPH_W + 2, GRAPH_H + 2, TFT_WHITE); }

int getSecondsPerPixel() {
    switch (displayHours) {
        case 1:  return 15; case 6:  return 90;
        case 12: return 180; case 24: return 360;
    }
    return 15;
}

void drawGraphGrid(const TimeAxisState &axis) {
  const int Y_GRID_STEP = 20;
  for (int v = Y_MIN + Y_GRID_STEP; v <= Y_MAX; v += Y_GRID_STEP) {
    int y = map(v, Y_MIN, Y_MAX, GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_W - 1, y, GRID_COLOR);
  }
  for (int i = 0; i < NUM_LABELS; i++) {
    float offsetMin = (NUM_LABELS - 1 - i) * axis.labelStepMin;
    int x = axis.anchorX - (int)((offsetMin / axis.minPerPx) + 0.5f);
    if (x < GRAPH_X || x > GRAPH_X + GRAPH_W) continue;
    tft.drawFastVLine(x, GRAPH_Y, GRAPH_H, GRID_COLOR);
  }
}

void drawGraphLabels(const TimeAxisState &axis) {
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKCYAN, BG_COLOR);
  tft.setCursor(GRAPH_X + 8, GRAPH_Y + 8);
  tft.printf("%dH", displayHoursOptions[displayHoursIndex]);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKCYAN, BG_COLOR);
#if defined(LANDSCAPE)
  for (int v = Y_MIN + 20; v <= Y_MAX; v += 20) {
    int y = map(v, Y_MIN, Y_MAX, GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.setCursor(GRAPH_X - 18, y - 4); tft.printf("%d", v);
  }
  int y0 = GRAPH_Y + GRAPH_H; tft.setCursor(GRAPH_X - 12, y0 - 6); tft.printf("%d", Y_MIN);
#elif defined(PORTRAIT)
  for (int v = Y_MIN + 20; v <= Y_MAX; v += 20) {
    int y = map(v, Y_MIN, Y_MAX, GRAPH_Y + GRAPH_H, GRAPH_Y);
    tft.setCursor(GRAPH_X - 16, y - 2); tft.printf("%d", v);
  }
  int y0 = GRAPH_Y + GRAPH_H; tft.setCursor(GRAPH_X - 8, y0 - 4); tft.printf("%d", Y_MIN);
#endif
  tft.fillRect(0, GRAPH_Y + GRAPH_H + 4, GRAPH_W + 44, 14, BG_COLOR);
  for (int i = 0; i < NUM_LABELS; i++) {
    float offsetMin = (NUM_LABELS - 1 - i) * axis.labelStepMin;
    int labelMin = axis.anchorMin - (int)(offsetMin + 0.5f);
    labelMin = (labelMin + 1440) % 1440;
    int x = axis.anchorX - (int)((offsetMin / axis.minPerPx) + 0.5f);
    if (x < GRAPH_X || x > GRAPH_X + GRAPH_W) continue;
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", labelMin / 60, labelMin % 60);
    tft.setTextColor(TFT_DARKGREEN, BG_COLOR);
#if defined(LANDSCAPE)
    tft.setCursor(x - 14, GRAPH_Y + GRAPH_H + 6);
#elif defined(PORTRAIT)
    tft.setCursor(x - 20, GRAPH_Y + GRAPH_H + 5);
#endif
    tft.print(buf); tft.print("     ");
  }
  tft.drawRect(GRAPH_X - 1, GRAPH_Y - 1, GRAPH_W + 2, GRAPH_H + 2, TFT_WHITE);
}

void drawGraphData() {
    if (!timeSynced) return;
    time_t nowTs = time(nullptr);
    int graphBottom = GRAPH_Y + GRAPH_H;
    float scale = (float)GRAPH_H / (Y_MAX - Y_MIN);
    int prevTempX = -1, prevTempY = -1;
    int prevHumiX = -1, prevHumiY = -1;
    uint32_t prevValidTs = 0;

    int record_count = isDisplayBufferFull ? DISPLAY_MAX_SAMPLES : displayLogIndex;

    for (int i = 0; i < record_count; i++) {
        int idx = (displayLogIndex - record_count + i + DISPLAY_MAX_SAMPLES) % DISPLAY_MAX_SAMPLES;
        LogRecord rec = displayLogBuf[idx];
        bool isDataGap = false;
        if (rec.ts == 0 || rec.ts == 0xFFFFFFFF) isDataGap = true;
        else if (prevValidTs != 0 && (rec.ts - prevValidTs) > (GRAPH_SAMPLE_INTERVAL_SEC * 1.5)) isDataGap = true;
        
        if (isDataGap) {
            prevTempX = -1; prevHumiX = -1; prevValidTs = 0; continue;
        }

        unsigned long secondsAgo = nowTs - rec.ts;
        if (secondsAgo > displayHours * 3600UL) {
             prevTempX = -1; prevHumiX = -1; prevValidTs = 0; continue;
        }

        int currX = GRAPH_X + GRAPH_W - (int)((float)secondsAgo / secondsPerPixel);
        if (currX < GRAPH_X || currX >= GRAPH_X + GRAPH_W) {
             prevTempX = -1; prevHumiX = -1; prevValidTs = 0; continue;
        }

        if (rec.temp != INVALID_VALUE) {
            int currTempY = graphBottom - (int)(((rec.temp / 10.0f) - Y_MIN) * scale + 0.5f);
            currTempY = constrain(currTempY, GRAPH_Y, graphBottom);
            if (prevTempX != -1) tft.drawLine(prevTempX, prevTempY, currX, currTempY, TFT_YELLOW);
            prevTempX = currX; prevTempY = currTempY;
        } else { prevTempX = -1; }

        if (rec.humi != INVALID_VALUE) {
            int currHumiY = graphBottom - (int)(((rec.humi / 10.0f) - Y_MIN) * scale + 0.5f);
            currHumiY = constrain(currHumiY, GRAPH_Y, graphBottom);
            if (prevHumiX != -1) tft.drawLine(prevHumiX, prevHumiY, currX, currHumiY, TFT_GREEN);
            prevHumiX = currX; prevHumiY = currHumiY;
        } else { prevHumiX = -1; }

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
  uint8_t encState = (digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT);
  if (encState == lastEncState) return;
  int delta = 0;
  if ((lastEncState == 0b00 && encState == 0b01) || (lastEncState == 0b01 && encState == 0b11) ||
      (lastEncState == 0b11 && encState == 0b10) || (lastEncState == 0b10 && encState == 0b00)) {
    delta = +1;
  } else if ((lastEncState == 0b00 && encState == 0b10) || (lastEncState == 0b10 && encState == 0b11) ||
             (lastEncState == 0b11 && encState == 0b01) || (lastEncState == 0b01 && encState == 0b00)) {
    delta = -1;
  }
  if (delta != 0) {
    brightnessStep[selectedLED] += delta * ENCODER_DIR;
    brightnessStep[selectedLED] = constrain(brightnessStep[selectedLED], 0, BRIGHTNESS_STEPS);
    pwmValue[0] = stepToPWM(brightnessStep[0]);
    pwmValue[1] = stepToPWM(brightnessStep[1]);
    ledcWrite(0, pwmValue[0]); ledcWrite(1, pwmValue[1]);

    drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
    drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);

  }
  lastEncState = encState;
}



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



void changeGraphMode() {
   displayHoursIndex = (displayHoursIndex + 1) % NUM_GRAPH_MODES;
   updateGraphTimeScale();
   drawGraph();
}




void humidifierToggle() {
  if (digitalRead(HUMIDIFIER_PWR) == HIGH) {
    digitalWrite(HUMIDIFIER_RUN, HIGH);              // 가습기 버튼 Released
    digitalWrite(HUMIDIFIER_PWR, LOW);               // 가습기 전원 OFF


  } else  {
    digitalWrite(HUMIDIFIER_PWR, HIGH);              // 가습기 전원 ON
    delay(100);
    digitalWrite(HUMIDIFIER_RUN, LOW);               // 가습기 버튼 Pressed  (RUN)
    delay(100);
    digitalWrite(HUMIDIFIER_RUN, HIGH);              // 가습기 버튼 Released 
  }
}

// Helper function for hardware ON sequence
void setHumidifierHwOn() {
  if (digitalRead(HUMIDIFIER_PWR) == HIGH) return; // Already on
  digitalWrite(HUMIDIFIER_PWR, HIGH);
  delay(100);
  digitalWrite(HUMIDIFIER_RUN, LOW);               // 가습기 버튼 Pressed (RUN)
  delay(100);
  digitalWrite(HUMIDIFIER_RUN, HIGH);              // 가습기 버튼 Released
}

void humidifierOn() {
  humidifierMode = ON;
  setHumidifierHwOn();
  manualHumidifierStartTime = millis();
}

void humidifierOff() {
  if (digitalRead(HUMIDIFIER_PWR) == LOW) return; // Already off
  digitalWrite(HUMIDIFIER_RUN, HIGH);              // 가습기 버튼 Released
  digitalWrite(HUMIDIFIER_PWR, LOW);               // 가습기 전원 OFF
  manualHumidifierStartTime = 0;
}



/*
void humidifierToggle2() {
  digitalWrite(HUMIDIFIER, LOW);                // 가습기 버튼 Pressed    강도 2단(Continue)
  delay(80);
  digitalWrite(HUMIDIFIER, HIGH);               // 가습기 버튼 Released
  delay(80);
  digitalWrite(HUMIDIFIER, LOW);                // 가습기 버튼 Pressed    강도 1단(Pulse)
  delay(80);
  digitalWrite(HUMIDIFIER, HIGH);               // 가습기 버튼 Released
  delay(80);
  digitalWrite(HUMIDIFIER, LOW);                // 가습기 버튼 Pressed    정지(STOP)
  delay(80);
  digitalWrite(HUMIDIFIER, HIGH);               // 가습기 버튼 Released
}*/


void checkHumidity() {
  if (isnan(lastHumi)) return;

  if (lastHumi > humiMax) {
    humidifierOff();
  } else if (lastHumi < humiMin) {
    setHumidifierHwOn();
  }
}


void drawSystemInfo() {
  tft.fillRect(0, TITLE_Y + 28, tft.width(), tft.height() - (TITLE_Y + 28), GRID_COLOR);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, GRID_COLOR);
  tft.setCursor(10, INFO_Y);
  tft.print("System Info:");
  int y_offset = INFO_Y + 30;
  tft.setCursor(10, y_offset); tft.printf("SSID: %s", WiFi.SSID().c_str()); y_offset += 20;
  tft.setCursor(10, y_offset); tft.printf("Ext.IP: %s", WiFi.localIP().toString().c_str()); y_offset += 20;
  tft.setCursor(10, y_offset); tft.printf("Int.IP: %s", WiFi.softAPIP().toString().c_str()); y_offset += 20;
  unsigned long s = millis()/1000;
  tft.setCursor(10, y_offset); tft.printf("Uptime: %luD %02luH %02luM %02luS", s/86400, (s%86400)/3600, (s%3600)/60, s%60); y_offset += 20;
  tft.setCursor(10, y_offset); tft.printf("NTP Sync: %s", timeSynced ? "Synced" : "Not Synced"); y_offset += 20;

  String modeStr;
  if (humidifierMode == AUTO) modeStr = "AUTO";
  else if (humidifierMode == ON) modeStr = "ON";
  else modeStr = "OFF";
  tft.setCursor(10, y_offset); tft.printf("Humi Mode: %s", modeStr.c_str()); y_offset += 20;
  tft.setCursor(10, y_offset); tft.printf("Humi Range: %d%% - %d%%", humiMin, humiMax);

}



void removeLogFile(){

    LittleFS.remove(LOG_FILE);
    LittleFS.remove(META_FILE);
    initFlashStorage();
    initDisplayBuffer();
    drawGraph();
}



void handleEncoderButton() {
    bool btnCurrState = digitalRead(ENCODER_SW);
    unsigned long now = millis();

    // Button press detection
    if (btnPrevState == HIGH && btnCurrState == LOW) {
        encoderPressStart = now;
        longPressActive = false;
    }
    // Button release detection
    else if (btnPrevState == LOW && btnCurrState == HIGH) {
        if (!longPressActive) { // It's a click, not the end of a long press
            unsigned long pressDuration = now - encoderPressStart;
            if (pressDuration < LONG_PRESS_MS) {
                btnClickCount++;
                btnReleaseTime = now;
            }
        }
    }
    // Long press detection (while holding)
    else if (btnCurrState == LOW && !longPressActive) {
        if (now - encoderPressStart > LONG_PRESS_MS) {
            removeLogFile();
            longPressActive = true; // Mark that long press action was taken
        }
    }
    
    // Process clicks after a timeout
    if (btnClickCount > 0 && now - btnReleaseTime > DOUBLE_CLICK_MS) {
        if (btnClickCount == 1) {
            // Single click action
            changeGraphMode();
        } else if (btnClickCount == 2) {
            // Double click action
            sysInfoDisplayUntil = millis() + 3000;
            drawSystemInfo();
        }
        btnClickCount = 0;

    }

    btnPrevState = btnCurrState;
}



// =========================================
// WiFi / NTP / WebServer
// =========================================
void startAP() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
}

// --- Webserver handlers ---
void handleRoot(bool error = false) { String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Login</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; }
        .error { color: #d9534f; text-align: center; margin-bottom: 1em; font-weight: bold; }
        label { display: block; margin-bottom: 0.5em; font-weight: bold; }
        input[type='text'], input[type='password'] { width: 100%; padding: 0.8em; margin-bottom: 1em; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
        input[type='submit'] { width: 100%; background-color: #007bff; color: #fff; padding: 1em; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; font-weight: bold; }
        input[type='submit']:hover { background-color: #0056b3; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Device Login</h2>)rawliteral"; if(error){page+="<p class='error'>Invalid username or password.</p>";} page+=R"rawliteral(
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
</html>)rawliteral"; server.send(200, "text/html; charset=UTF-8", page); }
void handleDashboard() {
    String page=R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Dashboard</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; margin-bottom: 1.5em; }
        .menu-button { display: block; width: calc(100% - 2em); padding: 1em; margin: 1em auto; background-color: #007bff; color: #fff; text-decoration: none; border-radius: 4px; text-align: center; font-weight: bold; font-size: 1.1em; }
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
        <a href="/remote" class="menu-button">원격제어</a>
        <a href="/downloadlog" class="menu-button download-button">Download Log File</a>
    </div>
</body>
</html>)rawliteral";
    server.send(200, "text/html; charset=UTF-8", page);
}

void handleLogin() {
    if (server.hasArg("username") && server.arg("username") == "cage" && server.arg("password") == "cage1234") {
        server.sendHeader("Location", "/dashboard", true);
        server.send(302, "text/plain", "");
    } else {
        handleRoot(true);
    }
}

void handleRemote() {
    String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>원격제어</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; }
        fieldset { border: 1px solid #ddd; border-radius: 8px; padding: 1.5em; margin-bottom: 2em; }
        legend { font-size: 1.2em; font-weight: bold; color: #007bff; padding: 0 0.5em; }
        .radio-group div { margin-bottom: 0.75em; }
        label { margin-left: 0.5em; font-weight: normal; }
        input[type='submit'] { width: 100%; background-color: #28a745; color: #fff; padding: 1em; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; font-weight: bold; }
        input[type='submit']:hover { background-color: #218838; }
        a.back-link { display: inline-block; margin-top: 1em; color: #007bff; text-decoration: none; }
    </style>
</head>
<body>
    <div class="container">
        <h2>원격제어</h2>
        <form method='POST' action='/sethumidifier'>
            <fieldset>
                <legend>습도 범위 설정 (가습기 자동제어)</legend>
                <p style="text-align: center; font-weight: bold; color: #337ab7;">
                  현재 온도: <span id="temp_val">%CURRENT_TEMP%</span>&deg;C / 현재 습도: <span id="humi_val">%CURRENT_HUMI%</span>%
                </p>
                <div style="margin-bottom: 1em;">
                  <label for="humi_max" style="display: inline-block; width: 120px;">최고 습도 (%):</label>
                  <input type="number" id="humi_max" name="humi_max" value="%HUMI_MAX_VALUE%" min="0" max="100" style="width: 80px;">
                </div>
                <div>
                  <label for="humi_min" style="display: inline-block; width: 120px;">최저 습도 (%):</label>
                  <input type="number" id="humi_min" name="humi_min" value="%HUMI_MIN_VALUE%" min="0" max="100" style="width: 80px;">
                </div>
            </fieldset>
            <fieldset>
                <legend>가습기 제어</legend>
                <div class="radio-group">
                    <div><input type="radio" id="h_auto" name="h_mode" value="auto" %AUTO_CHECKED%><label for="h_auto">AUTO (자동제어)</label></div>
                    <div><input type="radio" id="h_on" name="h_mode" value="on" %ON_CHECKED%><label for="h_on">ON (수동제어)</label></div>
                    <div><input type="radio" id="h_off" name="h_mode" value="off" %OFF_CHECKED%><label for="h_off">OFF (수동제어)</label></div>

                </div>
            </fieldset>
            <input type='submit' value='설정 저장'>
        </form>
        <a href="/dashboard" class="back-link"> &larr; Dashboard로 돌아가기</a>
    </div>
    <script>
      setInterval(function() {
        fetch('/sensordata')
          .then(response => response.json())
          .then(data => {
            document.getElementById('temp_val').innerText = data.temp;
            document.getElementById('humi_val').innerText = data.humi;
          })
          .catch(error => console.error('Error fetching sensor data:', error));
      }, 2000); // 2초마다 데이터 갱신
    </script>
</body>
</html>)rawliteral";

    if (humidifierMode == ON) page.replace("%ON_CHECKED%", "checked"); else page.replace("%ON_CHECKED%", "");
    if (humidifierMode == OFF) page.replace("%OFF_CHECKED%", "checked"); else page.replace("%OFF_CHECKED%", "");
    if (humidifierMode == AUTO) page.replace("%AUTO_CHECKED%", "checked"); else page.replace("%AUTO_CHECKED%", "");

    page.replace("%HUMI_MIN_VALUE%", String(humiMin));
    page.replace("%HUMI_MAX_VALUE%", String(humiMax));
    
    String tempStr = isnan(lastTemp) ? "N/A" : String(lastTemp, 1);
    String humiStr = isnan(lastHumi) ? "N/A" : String(lastHumi, 1);
    page.replace("%CURRENT_TEMP%", tempStr);
    page.replace("%CURRENT_HUMI%", humiStr);

    server.send(200, "text/html; charset=UTF-8", page);
}

void handleSensorData() {
  String json = "{";
  json += "\"temp\":\"" + (isnan(lastTemp) ? "N/A" : String(lastTemp, 1)) + "\",";
  json += "\"humi\":\"" + (isnan(lastHumi) ? "N/A" : String(lastHumi, 1)) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetHumidifier() {
    bool valuesChanged = false;
    if (server.hasArg("humi_min")) {
        int newMin = server.arg("humi_min").toInt();
        if (newMin >= 0 && newMin <= 100 && newMin != humiMin) {
            humiMin = newMin;
            valuesChanged = true;
        }
    }
    if (server.hasArg("humi_max")) {
        int newMax = server.arg("humi_max").toInt();
        if (newMax >= 0 && newMax <= 100 && newMax != humiMax) {
            humiMax = newMax;
            valuesChanged = true;
        }
    }

    if (valuesChanged) {
        preferences.begin("humidifier", false);
        preferences.putInt("humiMin", humiMin);
        preferences.putInt("humiMax", humiMax);
        preferences.end();
    }

    if (server.hasArg("h_mode")) {
        String mode = server.arg("h_mode");
        HumidifierMode oldMode = humidifierMode;

        if (mode == "on") {
            humidifierOn(); // This sets humidifierMode to ON
        } else if (mode == "off") {
            humidifierMode = OFF;
            humidifierOff();
        } else { // "auto"
            humidifierMode = AUTO;
            humidifierOff(); 
        }

        // If mode changed, save it to Preferences
        if (oldMode != humidifierMode) {
            preferences.begin("humidifier", false);
            preferences.putInt("humiMode", static_cast<int>(humidifierMode));
            preferences.end();
        }
    }
    // After setting, show the page again to confirm the change
    handleRemote();
}

void handleConfig() { String page=R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Setup</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; }
        p.info { text-align: center; margin-bottom: 2em; color: #666; font-size: 0.9em; }
        fieldset { border: 1px solid #ddd; border-radius: 8px; padding: 1.5em; margin-bottom: 2em; }
        legend { font-size: 1.2em; font-weight: bold; color: #007bff; padding: 0 0.5em; }
        label { display: block; margin-bottom: 0.5em; font-weight: bold; }
        input[type='text'], input[type='password'] { width: 100%; padding: 0.8em; margin-bottom: 1em; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
        input[type='submit'] { width: 100%; background-color: #28a745; color: #fff; padding: 1em; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; font-weight: bold; }
        input[type='submit']:hover { background-color: #218838; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Device Configuration</h2>
        <form method='POST' action='/save'>
            <fieldset>
                <legend>WiFi Settings</legend>
                <p class="info">Enter WiFi details for internet access.</p>
                <label for='ssid'>SSID:</label>
                <input id='ssid' name='ssid' type='text' required value='%SSID_VALUE%'>
                <label for='password'>Password:</label>
                <input id='password' name='password' type='password' value='%PASSWORD_VALUE%'>
            </fieldset>
            <br><br>
            <input type='submit' value='Save & Reboot'>
        </form>
    </div>
</body>
</html>)rawliteral"; preferences.begin("wifi", true); String savedSsid=preferences.getString("ssid", ""); String savedPass=preferences.getString("password", ""); preferences.end(); page.replace("%SSID_VALUE%", savedSsid); page.replace("%PASSWORD_VALUE%", savedPass); server.send(200, "text/html; charset=UTF-8", page); }
void handleNTPConfig() { String page=R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>NTP Setup</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; color: #333; }
        .container { max-width: 600px; margin: 2em auto; background-color: #fff; padding: 2em; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        h2 { text-align: center; color: #007bff; }
        p.info { text-align: center; margin-bottom: 2em; color: #666; font-size: 0.9em; }
        fieldset { border: 1px solid #ddd; border-radius: 8px; padding: 1.5em; margin-bottom: 2em; }
        legend { font-size: 1.2em; font-weight: bold; color: #007bff; padding: 0 0.5em; }
        label { display: block; margin-bottom: 0.5em; font-weight: bold; }
        input[type='text'] { width: 100%; padding: 0.8em; margin-bottom: 1em; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
        input[type='radio'] { margin-right: 0.5em; }
        input[type='submit'] { width: 100%; background-color: #28a745; color: #fff; padding: 1em; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; font-weight: bold; }
        input[type='submit']:hover { background-color: #218838; }
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
                    <input id='customNtpServer' name='customNtpServer' type='text' placeholder="e.g., pool.ntp.org" style="flex-grow: 1; margin-left: 0.5em;" value="%CUSTOM_NTP_VALUE%">
                </div>
            </fieldset>
            <br><br>
            <input type='submit' value='Save & Reboot'>
        </form>

    </div>
</body>
</html>)rawliteral"; preferences.begin("wifi", true); String savedNtpMode=preferences.getString("ntpMode", "builtin1"); String savedCustomNtp=preferences.getString("ntpServer", ""); preferences.end(); page.replace("%CUSTOM_NTP_VALUE%", savedCustomNtp); if(savedNtpMode=="builtin1"){page.replace("%BUILTIN1_CHECKED%","checked");}else{page.replace("%BUILTIN1_CHECKED%","");} if(savedNtpMode=="builtin2"){page.replace("%BUILTIN2_CHECKED%","checked");}else{page.replace("%BUILTIN2_CHECKED%","");} if(savedNtpMode=="custom"){page.replace("%CUSTOM_CHECKED%","checked");}else{page.replace("%CUSTOM_CHECKED%","");} server.send(200, "text/html; charset=UTF-8", page); }
void handleSave() { if (server.hasArg("ssid") && server.hasArg("password")) { String ssid = server.arg("ssid"); if(ssid.length()==0){String page=R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Error</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; text-align: center; }
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
</html>)rawliteral"; server.send(400,"text/html; charset=UTF-8",page); return;} String pass=server.arg("password"); preferences.begin("wifi",false); preferences.putString("ssid",ssid); preferences.putString("password",pass); preferences.end(); String page=R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Saved</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; text-align: center; }
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
</html>)rawliteral"; server.send(200,"text/html; charset=UTF-8",page); delay(2000); ESP.restart(); } else { server.send(400, "text/plain", "400: Invalid Request"); } }
void handleNTPSave() { String ntpMode=server.arg("ntpMode"); String customNtp=server.arg("customNtpServer"); preferences.begin("wifi",false); preferences.putString("ntpMode",ntpMode); if(ntpMode=="custom"){preferences.putString("ntpServer",customNtp);} else {preferences.remove("ntpServer");} preferences.end(); String page=R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Saved</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif; background-color: #f4f4f4; margin: 0; padding: 1em; text-align: center; }
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
</html>)rawliteral"; server.send(200,"text/html; charset=UTF-8",page); delay(2000); ESP.restart(); }






void handleDownloadLog() {
    File logFile = LittleFS.open(LOG_FILE, "r");
    if (!logFile) {
        server.send(404, "text/plain", "Log file not found.");
        return;
    }

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/csv", ""); // Send header
    
    server.sendContent("Timestamp,Temperature(C),Humidity(%),Epoch\n");

    uint32_t records_to_read = logMeta.record_count;
    uint32_t start_index = (logMeta.head_index + FLASH_MAX_RECORDS - records_to_read) % FLASH_MAX_RECORDS;

    LogRecord rec;
    char line_buffer[128]; // Use char buffer to avoid String fragmentation

    for (uint32_t i = 0; i < records_to_read; i++) {
        uint32_t current_flash_index = (start_index + i) % FLASH_MAX_RECORDS;
        logFile.seek(current_flash_index * sizeof(LogRecord));
        
        if (logFile.read((uint8_t*)&rec, sizeof(LogRecord)) == sizeof(LogRecord)) {
            if (rec.ts == 0 || rec.temp == INVALID_VALUE || rec.humi == INVALID_VALUE) {
                continue;
            }
            struct tm *timeinfo; time_t rawtime = rec.ts; timeinfo = localtime(&rawtime);
            char timeStr[20];
            if (strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", timeinfo) == 0) {
                strcpy(timeStr, "Invalid Time");
            }
            
            snprintf(line_buffer, sizeof(line_buffer), "%s,%.1f,%.1f,%lu\n",
                     timeStr,
                     rec.temp / 10.0f,
                     rec.humi / 10.0f,
                     (unsigned long)rec.ts);
            server.sendContent(line_buffer, strlen(line_buffer));
            yield(); // Allow system tasks to run, preventing watchdog timeout
        }
    }
    logFile.close();
    server.sendContent(""); // End of stream
}







void ntpUpdate() {
    unsigned long now = millis();
    if (!timeSynced || (now - lastNtpSync >= NTP_SYNC_INTERVAL)) {
        if (WiFi.status() != WL_CONNECTED) return;
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "pool.ntp.org");
        struct tm timeinfo;
        if (getLocalTime(&timeinfo) && timeinfo.tm_year > (2020-1900)) {
            timeSynced = true;
            lastNtpSync = now;
        }
    }
}




// =========================================
void setup() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);
  lastEncState = (digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT);
  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  ledcSetup(0,5000,8); ledcAttachPin(LED1_PIN,0);
  ledcSetup(1,5000,8); ledcAttachPin(LED2_PIN,1);

  pinMode(HUMIDIFIER_PWR, OUTPUT);          // 가습기 전원
  pinMode(HUMIDIFIER_RUN, OUTPUT);          // 가습기 작동버튼
  digitalWrite(HUMIDIFIER_PWR, LOW);        // 가습기 전원차단(OFF)
  digitalWrite(HUMIDIFIER_RUN, HIGH);       // 가습기 작동버튼 Release


  tft.init();
  tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(BG_COLOR);

  initFlashStorage();
  
  preferences.begin("humidifier", true); // Open in read-only mode
  humiMin = preferences.getInt("humiMin", 45);
  humiMax = preferences.getInt("humiMax", 65);
  humidifierMode = static_cast<HumidifierMode>(preferences.getInt("humiMode", static_cast<int>(AUTO)));
  preferences.end();

  // Apply loaded humidifier state after reboot
  if (humidifierMode == ON) {
    // If last state was ON, restart the timed cycle
    humidifierOn();
  } else if (humidifierMode == OFF) {
    // If last state was OFF, ensure it's off
    humidifierOff();
  }
  // If last state was AUTO, the main loop will handle it

  preferences.begin("wifi", true);
  String savedSsid = preferences.getString("ssid", "");
  String savedPass = preferences.getString("password", "");
  String savedNtpMode = preferences.getString("ntpMode", "builtin1");
  String savedCustomNtp = preferences.getString("ntpServer", "time.bora.net");
  preferences.end();

  if (savedNtpMode == "builtin1") strcpy(ntpServer, "time.bora.net");
  else if (savedNtpMode == "builtin2") strcpy(ntpServer, "time.kriss.re.kr");
  else if (savedCustomNtp.length() > 0) savedCustomNtp.toCharArray(ntpServer, sizeof(ntpServer));
  else strcpy(ntpServer, "time.bora.net");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeout(1000); // Set 1ms timeout for I2C communication
#if defined(USE_SHT41)
  sht4.begin(Wire, SHT41_I2C_ADDR); delay(10); sht4.softReset();
#elif defined(USE_AHT20_BMP280)
  if (!aht20.begin()) {} // Error handling omitted for brevity
  if (!bmp280.begin(0x76)) {} // Error handling omitted for brevity
#endif

  drawTitle();
  loadDataForDisplay();
  updateGraphTimeScale();
  drawGraphFrame();
  drawGraph();

  drawLEDUI(1, LED_UI_Y1, brightnessStep[0], true);
  drawLEDUI(2, LED_UI_Y2, brightnessStep[1], false);



  startAP();
  server.on("/", HTTP_GET, [](){ handleRoot(false); });
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/dashboard", HTTP_GET, handleDashboard);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/ntpconfig", HTTP_GET, handleNTPConfig);
  server.on("/remote", HTTP_GET, handleRemote);
  server.on("/sethumidifier", HTTP_POST, handleSetHumidifier);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/ntpsave", HTTP_POST, handleNTPSave);
  server.on("/downloadlog", HTTP_GET, handleDownloadLog);
  server.on("/sensordata", HTTP_GET, handleSensorData);
  server.begin();

  if (savedSsid.length() > 0) {
    WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) delay(500);
  }
  externalAPConnected = (WiFi.status() == WL_CONNECTED);
}

// =========================================
void loop() {
  unsigned long nowMs = millis();
  
  server.handleClient();
  handleEncoderButton(); // Handle button input regardless of display mode

  // Check for manual humidifier auto-off
  if (humidifierMode == ON && manualHumidifierStartTime != 0) {
    if (millis() - manualHumidifierStartTime >= (HUMIDIFIER_AUTO_OFF_MINUTES * 60 * 1000UL)) {
      humidifierOff();
      humidifierMode = OFF; // 수동 OFF 상태로 변경
    }
  }

  // Handle system info display overlay
  if (sysInfoDisplayUntil > 0) {
    if (nowMs >= sysInfoDisplayUntil) {
      // Time to hide sysInfo and redraw main screen
      sysInfoDisplayUntil = 0;
      // Redraw the screen elements instead of a full clear to reduce flicker
      drawTitle();
      // Need to redraw graph frame as well as it might be overwritten.
      drawGraphFrame();
      drawGraph();
      drawLEDUI(1, LED_UI_Y1, brightnessStep[0], selectedLED == 0);
      drawLEDUI(2, LED_UI_Y2, brightnessStep[1], selectedLED == 1);
      drawTimeTempHumi(lastTemp, lastHumi, true);
    }
    return; // Do nothing else while info is displayed
  }
  
  handleEncoderRotation();
  ntpUpdate();

  if (nowMs - lastSample >= SAMPLE_INTERVAL) {
      lastSample = nowMs;
      float temperature, humidity;
      if (readSensor(temperature, humidity)) {
          lastTemp = temperature; lastHumi = humidity;
          accTemp += temperature; accHumi += humidity; accCount++;
          if (humidifierMode == AUTO) {
            checkHumidity();
          }
      } else { lastTemp = NAN; lastHumi = NAN; }
      drawTimeTempHumi(lastTemp, lastHumi, true);
  }

  if (nowMs - lastGraphSampleMs >= GRAPH_SAMPLE_INTERVAL_SEC * 1000UL) {
      lastGraphSampleMs = nowMs;
      float avgTemp = (accCount > 0) ? (accTemp / accCount) : NAN;
      float avgHumi = (accCount > 0) ? (accHumi / accCount) : NAN;
      
      pushToDisplayBuffer(avgTemp, avgHumi);

      accTemp = 0; accHumi = 0; accCount = 0;
      drawGraph();
  }
}
