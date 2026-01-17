/**************************************************************
 * Lizard Cage Monitoring (Stable + Scroll Graph + External AP + Long-term Graph)
 * ESP32 + TFT_eSPI + DHT22 + Rotary Encoder
 * LED UI + REAL PWM LED control (2ch)
 * 웹에서 사용자로그인 후, 외부AP설정, NTP설정, 로그파일(CSV) 다운로드
 * 웹에서 온습도 센서 선택가능 : SHT41, AHT20_BMP280 , 비동기통신(AJAX) 방식을 이용해 실시간 온,습도 표시
 * 웹에서 장치별운전모드 설정 : AUTO, ON, OFF
 * 웹에서 LCD LANDSCAPE/PORTRAIT 선택가능
 * 장시간 평균 기반 그래프 (1~24시간 X축) + 가변 타임스탬프
 * 그래프모드 추가(1,6,12,24H)
 * 시스템 정보표시 추가, 버튼누름 세분화(싱글, 더블, 롱클릭)
 * 플래시 메모리 수명 무한대: 메타데이터 저장로직 제거, 부팅시 마지막기록포인트 검색후 진행처리
 * 플래시 메모리 보호: 잦은 기록으로 인한 보드 고장 방지 (1단계)
 ************************************************************/

  // 트루타입 폰트 설정 및 해제
  // tft.setFreeFont(&FreeSansBold12pt7b); 
  // tft.setFreeFont(NULL); 


#include <Arduino.h>
#include <LittleFS.h>
#include <TFT_eSPI.h>
//#include "Fonts/FreeSansBold12pt7b.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <Wire.h>


using namespace fs;     // File 사용 가능

int tempMin = 22;                   // 최소 온도 조건(히터 동작)
int tempMax = 32;                   // 최대 온도 조건(히터 정지, tempMax 초과시 FAN 동작)
int humiMax = 60;                   // 최고습도 조건(가습기 정지)
int humiMin = 45;                   // 최저습도 조건(가습기 동작)



// 아이콘 30x30 Hot Springs (♨?) style icon

const uint8_t hotSpring30[120] PROGMEM = {
  // --- Steam (Top) ---
  0b00000000, 0b00000000, 0b00000000, 0b00000000,   // Row 0
  0b00000000, 0b00000011, 0b00000000, 0b00000000,
  0b00000000, 0b00000001, 0b10000000, 0b00000000,
  0b00000000, 0b00000001, 0b10000000, 0b00000000,
  0b00000001, 0b10000011, 0b00000110, 0b00000000,
  0b00000000, 0b11000110, 0b00001100, 0b00000000,
  0b00000000, 0b11000110, 0b00001100, 0b00000000,
  0b00000001, 0b10000011, 0b00000110, 0b00000000,
  0b00000011, 0b00000001, 0b10000011, 0b00000000,
  0b00000011, 0b00000001, 0b10000011, 0b00000000,
  0b00000001, 0b10000011, 0b00000110, 0b00000000,

  0b00000000, 0b00000011, 0b00000000, 0b00000000,
  0b00000000, 0b00000001, 0b10000000, 0b00000000,

  0b00000011, 0b00000000, 0b00000011, 0b11100000,   // Top edges of tub
  0b00111100, 0b00000000, 0b00000000, 0b11111000,
  0b01111110, 0b00000000, 0b00000000, 0b11111100,
  0b01111110, 0b00000000, 0b00000001, 0b11111100,
  0b01111111, 0b00000000, 0b00001111, 0b11111000,
  0b00111111, 0b11111111, 0b11111111, 0b11110000,
  0b00011111, 0b11111111, 0b11111111, 0b11000000,
  0b00000111, 0b11111111, 0b11111111, 0b10000000,
  0b00000000, 0b00111111, 0b01110000, 0b00000000,

  0b00000000, 0b00000000, 0b00000000, 0b00000000,   // Bottom
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000
};



enum OperMode { AUTO, ON, OFF };              // 가습기, 히터, 팬 동작모드
OperMode humidifierMode = AUTO;
OperMode heaterMode = AUTO;
OperMode fanMode = AUTO;

unsigned long manualHumidifierStartTime = 0;
unsigned long manualHeaterStartTime = 0;
unsigned long manualFanStartTime = 0;

#define HUMIDIFIER_AUTO_OFF_MINUTES 1 // 가습기 수동 ON 후 자동 OFF 시간 (분)
#define HEATER_AUTO_OFF_MINUTES 1 // 히터 수동 ON 후 자동 OFF 시간 (분)
#define FAN_AUTO_OFF_MINUTES 1 // 팬 수동 ON 후 자동 OFF 시간 (분)



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// --- 센서(SHT41, AHT20_BMP280) 라이브러리 모두 포함 ---
#include <SensirionI2cSht4x.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// --- 센서 객체 모두 생성 ---
SensirionI2cSht4x sht4;
Adafruit_AHTX0 aht20;
Adafruit_BMP280 bmp280;

#define SHT41_I2C_ADDR 0x44

// --- 센서 타입 관리용 변수 ---
int currentSensorType = 0;               // 0: SHT41, 1: AHT20+BMP280

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



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
#define AP_SSID "Cage"
#define AP_PASSWORD "cage1234"
IPAddress apIP(192,168,2,1);



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int screenRotation = 0; // 0: Portrait(세로), 1: Landscape(가로)
int layout_face_y = 0;
int layout_face_x = 0;        // 가로모드일때 얼굴좌표
int layout_icons_y = 0;
int layout_graph_x = 0;
int layout_graph_y = 0;
int layout_graph_w = 0;
int layout_graph_h = 0;
int layout_num_labels = 0;


// 헤드라인 텍스트도 방향에 따라 다를 수 있음
String headlineText = "Lizard Monitor";






#define Y_MIN 0
#define Y_MAX 100

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
#define BRIGHTNESS_STEPS 9
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
#define HEATER_PIN 14           // 히터 전원
#define FAN_PIN 22              // FAN 전원

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
TFT_eSprite graphSprite = TFT_eSprite(&tft); // [추가] 그래프용 스프라이트 선언


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
void checkTemperature();
void handleDashboard();



#define TFT_WIDTH_OFFSET  20          // User_Setup.h 에 정의되어 있음 //  텍스트 자동 줄바뀜 방지를 위한 가상공간 확보

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// [수정 2] 회전 모드에 따른 좌표 설정 함수

void updateLayout() {
  if (screenRotation == 0) { 
    // --- 세로 모드 (Portrait) ---
    layout_face_y = 115-16;
    layout_icons_y = 160;// + 120;
    layout_graph_x = 18;
    layout_graph_y = 190;
    layout_graph_w = 210+10;
    layout_graph_h = 100;
    layout_num_labels = 7;                  // X축 그리드
    headlineText = "   Lizard Guidian";
  } 
  else { 

    // [수정] 얼굴 아이콘 배치 (숨김 해제 -> 우측 상단 배치)
    layout_face_x = 285;                // [중요] 화면 우측 끝부분으로 X좌표 설정
    layout_face_y = 72; //80; //60;     // [중요] 화면 상단(정보창 옆)으로 Y좌표 설정 (-100 아님)

    // [유지] 아래는 사용자가 지정한 가로모드 배치값 그대로 유지
    layout_icons_y = 180 - 100 + 8; 
    layout_graph_x = 20;
    layout_graph_y = 66 + 50;
    layout_graph_w = 240 + 40 + 10 + 6;
    layout_graph_h = 100 + 10;
    layout_num_labels = 7;
    headlineText = "  Lizard Cage Monitoring";
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// =========================================
// Logging Functions (New Append-based Logic)
// =========================================



// [수정] 부팅 시 마지막 기록 위치를 자동으로 스캔하는 로직   (플레시메모리 수명 향상)
void initFlashStorage() {
  if (!LittleFS.begin(true)) {
    lcdPrint("LittleFS Mount Failed!");
    delay(5000);
    ESP.restart();
  }

  // 로그 파일이 없으면 새로 생성
  if (!LittleFS.exists(LOG_FILE)) {
    File logFile = LittleFS.open(LOG_FILE, "w");
    if (logFile) logFile.close();
    logMeta.head_index = 0;
    logMeta.record_count = 0;
    return;
  }

  // 로그 파일이 있으면 스캔 시작
  File logFile = LittleFS.open(LOG_FILE, "r");
  size_t fileSize = logFile.size();
  size_t maxRecords = fileSize / sizeof(LogRecord);
  
  logMeta.head_index = 0;
  logMeta.record_count = 0;

  if (maxRecords > 0) {
    LogRecord tempRec;
    uint32_t lastTs = 0;
    
    // 파일 전체를 훑어서 마지막 기록 위치 찾기
    for (size_t i = 0; i < maxRecords; i++) {
        logFile.read((uint8_t*)&tempRec, sizeof(LogRecord));
        
        // 1. 타임스탬프가 0이면(빈 데이터) 여기가 끝
        if (tempRec.ts == 0) {
            logMeta.head_index = i;
            break; 
        }
        
        // 2. 현재 시간이 이전 시간보다 과거라면? (순환 버퍼가 한 바퀴 돌았다는 뜻)
        // 예: [ ... 10:00, 10:01, 09:00 ... ] -> 09:00 자리가 현재 쓸 위치(head)
        if (i > 0 && tempRec.ts < lastTs) {
            logMeta.head_index = i;
            // 순환된 경우 레코드 수는 꽉 찬 것으로 간주
            logMeta.record_count = FLASH_MAX_RECORDS; 
            break; 
        }
        
        lastTs = tempRec.ts;
        logMeta.head_index = i + 1; // 계속 다음 칸으로 이동
        logMeta.record_count++;
    }
    
    // 인덱스가 범위를 넘어가면 0으로 초기화
    if (logMeta.head_index >= FLASH_MAX_RECORDS) {
        logMeta.head_index = 0;
    }
  }
  logFile.close();
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
  }
}



void readLogRecord(LogRecord& record, uint32_t index) {
  File logFile = LittleFS.open(LOG_FILE, "r");
  if (logFile) {
    if (index < logMeta.record_count) {
      logFile.seek(index * sizeof(LogRecord));
      logFile.read((uint8_t*)&record, sizeof(LogRecord));
    } else {
      record.ts = 0;                                            // Invalid record
    }
    logFile.close();
  } else {
    record.ts = 0;                                              // Invalid record
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



// readSensor 함수  SHT41, AHT20
bool readSensor(float &temp, float &humi) {
  if (currentSensorType == 0) {
    uint16_t error = sht4.measureHighPrecision(temp, humi);         // --- SHT41 읽기 ---
    return (error == 0);
  } 
  else {
    sensors_event_t humEvent, tempEvent;                            // --- AHT20 읽기 ---
    if (!aht20.getEvent(&humEvent, &tempEvent)) return false;
    temp = tempEvent.temperature;
    humi = humEvent.relative_humidity;
    return true;
  }
}




float secondsPerPixel = 0;


void drawTitle() {
  // 1. 배경 지우기
  // tft.fillScreen(BG_COLOR); // (화면 전체 지우기는 loop 로직에 따라 깜빡임 유발 가능하므로 주석 유지 or 필요시 사용)
  
  const int TITLE_H = 28;
  tft.fillRect(0, TITLE_Y, tft.width(), TITLE_H, TFT_NAVY);
  
  // 2. 미려한 폰트 적용
  // FreeFont는 기본 폰트보다 크므로 Size를 1로 설정합니다.
  tft.setFreeFont(&FreeSansBold12pt7b); 
  tft.setTextSize(1);
  
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  
  // 3. 텍스트 위치 보정
  // FreeFont는 기준점(Baseline)이 달라서 y좌표를 조금 더 내려야 중앙에 옵니다.
  // TITLE_Y(5) + 20 정도가 적당합니다.
  tft.setCursor(5, TITLE_Y + 20);
  tft.print(headlineText.c_str());
  
  // 4. 하단 라인 그리기
  tft.drawFastHLine(0, TITLE_Y + TITLE_H - 1, tft.width(), TFT_DARKGREY);

  // 5. [중요] 다른 텍스트에 영향이 없도록 기본 폰트로 복구
  tft.setFreeFont(NULL);
}




void drawTimeTempHumi(float t, float h, bool timeAvailable) {
  // 1. 기존 영역 지우기 (높이를 20 -> 24로 살짝 키워 여유를 줌)
  //tft.fillRect(0, INFO_Y, 240, 24, BG_COLOR);
  
  if (screenRotation == 0) {                              // 세로모드
      tft.fillRect(0, INFO_Y, 240, 24, BG_COLOR); 
  } else {                                                // 가로모드
      tft.fillRect(0, INFO_Y, 320, 24, BG_COLOR);
  }


  struct tm timeinfo;
  bool isTimeValid = getLocalTime(&timeinfo, 0) && timeinfo.tm_year > (2020 - 1900);

  // 2. 9포인트 폰트 적용
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextSize(1); // FreeFont는 기본 크기 사용

  // 3. 텍스트 기준선(Baseline) 보정
  // FreeFont는 y좌표가 글자 밑부분이므로, 기존보다 아래로 내려야 합니다.
  // INFO_Y(40) + 18 정도가 적당합니다.
  int textY = INFO_Y + 14;           // 18보다 14가 적당함

  if (isTimeValid) {
    if (!timeSynced) { timeSynced = true; lastNtpSync = millis(); }

    if (screenRotation == 0) {            // 세로모드
        // 시간 표시 (파란색)
        tft.setCursor(5, textY);
        tft.setTextColor(TFT_BLUE, BG_COLOR);
        //tft.printf("%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        tft.printf("%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        
        // 온도 표시 (노란색)
        tft.setCursor(95, textY); // x좌표 조정 (시간 길이 고려)
        tft.setTextColor(TFT_YELLOW, BG_COLOR);
        tft.printf("%4.1fC", t);
        
        // 습도 표시 (초록색)
        tft.setCursor(170, textY); // x좌표 조정
        tft.setTextColor(TFT_GREEN, BG_COLOR);
        tft.printf("%4.1f%%", h);

    } else {                                  // 가로모드
        int x= 140;                           // X축 좌표 보정
        // 시간 표시 (파란색)
        tft.setCursor(x + 5, textY);
        tft.setTextColor(TFT_BLUE, BG_COLOR);
        //tft.printf("%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        tft.printf("%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        
        // 온도 표시 (노란색)
        tft.setCursor(x + 60, textY); // x좌표 조정 (시간 길이 고려)
        tft.setTextColor(TFT_YELLOW, BG_COLOR);
        tft.printf("%4.1fC", t);
        
        // 습도 표시 (초록색)
        tft.setCursor(x + 120, textY); // x좌표 조정
        tft.setTextColor(TFT_GREEN, BG_COLOR);
        tft.printf("%4.1f%%", h);
    }

  //tft.setFreeFont(NULL);
  } else {
    
    if (timeSynced) timeSynced = false;            // 시간이 유효하지 않을 때 (WiFi 접속 중 등)

    if (screenRotation == 0) {                     // 세로모드
        tft.setCursor(5, textY);
        tft.setTextColor(TFT_ORANGE, BG_COLOR);

        if (WiFi.status() != WL_CONNECTED) tft.print("WiFi Connecting...");
        else tft.print("NTP Syncing...");

    } else {                                      // 가로모드
        int x= 140;                               // X축 좌표 보정
        tft.setCursor(x + 5, textY);
        tft.setTextColor(TFT_ORANGE, BG_COLOR);

        if (WiFi.status() != WL_CONNECTED) tft.print("WiFi Connecting...");
        else tft.print("NTP Syncing...");
    }
    tft.setCursor(5, textY);
    //tft.print("                   ");
  }
  // 4. 폰트 복구
  tft.setFreeFont(NULL);
}







struct TimeAxisState {
  float totalMin, minPerPx, labelStepMin;
  int nowMin, anchorMin, anchorX;
};

void computeTimeAxis(TimeAxisState &axis) {
  axis.totalMin = displayHours * 60.0f;
  axis.minPerPx = axis.totalMin / layout_graph_w;
  axis.labelStepMin = axis.totalMin / (layout_num_labels - 1);
  time_t nowSec; time(&nowSec); struct tm timeinfo; localtime_r(&nowSec, &timeinfo);
  axis.nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  axis.anchorMin = (axis.nowMin / 10) * 10;
  float anchorOffsetPx = (axis.nowMin - axis.anchorMin) / axis.minPerPx;
  axis.anchorX = layout_graph_x + layout_graph_w - (int)(anchorOffsetPx + 0.5f);
}




// --------------------------------------------------------- New UI Functions -----------------------------------------------------------------

void drawConditionFace() {    
    // -값일 경우 그리기 생략 
    if (layout_face_y < 0) return; 

    int centerX = (tft.width() - TFT_WIDTH_OFFSET) / 2;

    // --- 1. 상태 결정 로직 (기존 유지) ---
    uint16_t face_color;
    enum { SHAPE_SMILE, SHAPE_STRAIGHT, SHAPE_FROWN } mouth_shape;
    String temp_status, humi_status; 

    // Determine Temperature status
    if (isnan(lastTemp)) temp_status = "Err";
    else if (lastTemp < tempMin) temp_status = "Low";
    else if (lastTemp > tempMax) temp_status = "High";
    else temp_status = "Good";

    // Determine Humidity status
    if (isnan(lastHumi)) humi_status = "Err";
    else if (lastHumi < humiMin) humi_status = "Low";
    else if (lastHumi > humiMax) humi_status = "High";
    else humi_status = "Good";

    // 1a. Determine color
    if (isnan(lastTemp)) face_color = TFT_RED;
    else if (lastTemp < tempMin) face_color = TFT_BLUE;
    else if (lastTemp > tempMax) face_color = TFT_RED;
    else face_color = TFT_YELLOW;

    // 1b. Determine mouth shape
    bool temp_ok = !isnan(lastTemp) && (lastTemp >= tempMin && lastTemp <= tempMax);
    bool humi_ok = !isnan(lastHumi) && (lastHumi >= humiMin && lastHumi <= humiMax);

    if (temp_ok && humi_ok) mouth_shape = SHAPE_SMILE;                                  // T:Good, H:Good -> Smile (U)
    else if (temp_ok != humi_ok) mouth_shape = SHAPE_STRAIGHT;                          // One is good, one is bad -> Straight (-)
    else mouth_shape = SHAPE_FROWN;                                                     // Both are bad -> Frown (∩)


    // --- 2. Draw the Face ---
    int face_r = 28; // Large, round face

    // Clear area for the new larger face
    // layout_face_y 변수 사용

    if (screenRotation == 0) {         // --- 세로 모드 (Portrait)
        tft.fillRect(0, layout_face_y - face_r, tft.width(), face_r * 2, BG_COLOR);

        // Head
        tft.fillCircle(centerX, layout_face_y, face_r, face_color);
        tft.drawCircle(centerX, layout_face_y, face_r, TFT_BLACK);

        // Eyes
        tft.fillEllipse(centerX - 10, layout_face_y - 8, 4, 8, TFT_BLACK); // Left eye
        tft.fillEllipse(centerX + 10, layout_face_y - 8, 4, 8, TFT_BLACK); // Right eye

        // Mouth
        if (mouth_shape == SHAPE_SMILE) { // U shape (inverted Y)
            // Draw a thick parabolic smile
            for (int i = -12; i <= 12; i++) {
                int x = centerX + i;
                int y = layout_face_y + 18 - (int)(i * i * 0.05);
                tft.drawFastVLine(x, y, 2, TFT_BLACK);
            }
        } else if (mouth_shape == SHAPE_STRAIGHT) { // - shape
            tft.fillRect(centerX - 12, layout_face_y + 13, 25, 3, TFT_BLACK);
        }
        else { // SHAPE_FROWN (∩ shape) (inverted Y)
            // Draw a thick parabolic frown
            for (int i = -12; i <= 12; i++) {
                int x = centerX + i;
                int y = layout_face_y + 10 + (int)(i * i * 0.05);
                tft.drawFastVLine(x, y, 2, TFT_BLACK);
            }
        }
    }
    else {      // 가로화면
        //centerX += 130;                     // 기준 좌표를 우측으로 100px 이동
        centerX = 40;                         // 기준 좌표를 우측으로 10px  이동



        //tft.fillRect(0, layout_face_y - face_r, tft.width(), face_r * 2, BG_COLOR);
        tft.fillRect(centerX - face_r - 10, layout_face_y - face_r, (face_r * 2) + 20, face_r * 2, BG_COLOR);

        // Head
        tft.fillCircle(centerX, layout_face_y, face_r, face_color);
        tft.drawCircle(centerX, layout_face_y, face_r, TFT_BLACK);

        // Eyes
        tft.fillEllipse(centerX - 10, layout_face_y - 8, 4, 8, TFT_BLACK); // Left eye
        tft.fillEllipse(centerX + 10, layout_face_y - 8, 4, 8, TFT_BLACK); // Right eye

        // Mouth
        if (mouth_shape == SHAPE_SMILE) { // U shape (inverted Y)
            // Draw a thick parabolic smile
            for (int i = -12; i <= 12; i++) {
                int x = centerX + i;
                int y = layout_face_y + 18 - (int)(i * i * 0.05);
                tft.drawFastVLine(x, y, 2, TFT_BLACK);
            }
        } else if (mouth_shape == SHAPE_STRAIGHT) { // - shape
            tft.fillRect(centerX - 12, layout_face_y + 13, 25, 3, TFT_BLACK);
        }
        else { // SHAPE_FROWN (∩ shape) (inverted Y)
            // Draw a thick parabolic frown
            for (int i = -12; i <= 12; i++) {
                int x = centerX + i;
                int y = layout_face_y + 10 + (int)(i * i * 0.05);
                tft.drawFastVLine(x, y, 2, TFT_BLACK);
            }
        }
    }


    // --- 3. Draw Status Text ---
    tft.setTextDatum(TL_DATUM);                 // Top-Left datum
    tft.setTextSize(1);
    tft.setTextColor(TFT_SILVER, BG_COLOR);
    

    if (screenRotation == 0) {                                                        // --- 세로 모드 (Portrait)
      int text_x = 5;
      tft.drawString("Temp: " + temp_status, text_x, layout_face_y - 15);
      tft.drawString("Humi: " + humi_status, text_x, layout_face_y + 5);
    } else {                                                                          // --- 가로 모드 (Landscape)
      int text_x = 5;
      int offset_x = 70;
      tft.drawString("Temp: " + temp_status, text_x + offset_x - 2, layout_face_y - 20 - 18 + 4+10);
      tft.drawString("Humi: " + humi_status, text_x + offset_x - 2, layout_face_y - 10 - 18 + 4+10);
    }

}






// ----------------------------------- 바람개비 아이콘(회전동작) -------------------------------------------
// 애니메이션을 위한 변수 (루프 밖이나 클래스 멤버로 선언)
float fan_angle = 0; 

void drawSpinningFan(int centerX, int centerY, int radius, uint16_t color) {
    // 1. 이전 잔상을 지우기 (배경색이 검정색이라고 가정)
    tft.fillCircle(centerX, centerY, radius + 2, TFT_BLACK);

    // 2. 4개의 날개 그리기 (90도 간격)
    for (int i = 0; i < 4; i++) {
        float baseAngle = fan_angle + (i * PI / 2.0); // 90도(PI/2) 간격

        // 날개의 세 꼭짓점 계산
        // 중심점
        int x0 = centerX;
        int y0 = centerY;

        // 바깥쪽 끝점
        int x1 = centerX + radius * cos(baseAngle);
        int y1 = centerY + radius * sin(baseAngle);

        // 옆으로 퍼지는 점 (바람개비 날개 폭 결정)
        // baseAngle에서 약 30도(PI/6) 정도 옆으로 벌어진 지점
        int x2 = centerX + (radius * 0.8) * cos(baseAngle + PI / 6.0);
        int y2 = centerY + (radius * 0.8) * sin(baseAngle + PI / 6.0);

        // 삼각형 날개 채우기
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);
    }

    // 3. 중앙 축 (원본 이미지의 점 표현)
    tft.fillCircle(centerX, centerY, 2, TFT_WHITE);

    // 4. 회전 속도 조절
    fan_angle += 0.15; // 숫자가 클수록 빨리 회전합니다.
    if (fan_angle >= 2 * PI) fan_angle = 0;
}
// ----------------------------------- 바람개비 아이콘(회전동작) -------------------------------------------



void drawStatusIcons() {   
    int icon_x_start = 30;
    if (screenRotation == 1) icon_x_start += 75;       // 가로모드에서 화면배치 수정(우측으로 이동)

    // 전역 변수 layout_icons_y 사용 (화면 방향에 따라 높이 자동 조절)
    int icon_y = layout_icons_y + 6; 
    int icon_gap = 60;

    // 영역 지우기 - 화면 전체 폭(tft.width())만큼 지워서 회전 시 잔상 방지
    //tft.fillRect(0, icon_y - 15, tft.width(), 32, BG_COLOR);
    

    // [수정 후] 가로/세로 모드에 따라 지우는 영역 분리
    if (screenRotation == 1) {
        // 가로 모드: 얼굴이 있는 좌측(X=0~90)은 건드리지 않고, 아이콘 시작 지점 근처(90)부터 끝까지만 지움
        tft.fillRect(90, icon_y - 15, tft.width() - 90, 32, BG_COLOR);
    } else {
        // 세로 모드: 기존처럼 전체 너비를 지움
        tft.fillRect(0, icon_y - 15, tft.width(), 32, BG_COLOR);
    }



    tft.setTextSize(1);
    tft.setTextColor(TFT_SILVER, BG_COLOR);

    // ---------------------- 1. LED Icon ----------------------
    bool led_on = (brightnessStep[0] > 0 || brightnessStep[1] > 0);
    uint16_t led_color = led_on ? TFT_YELLOW : TFT_DARKGREY;
    
    tft.fillCircle(icon_x_start, icon_y, 8, led_color);
    for (int i=0; i<8; i++) { 
        float angle = i * PI / 4;
        int x1 = icon_x_start + 10 * cos(angle);
        int y1 = icon_y + 10 * sin(angle);
        int x2 = icon_x_start + 13 * cos(angle);
        int y2 = icon_y + 13 * sin(angle);
        tft.drawLine(x1, y1, x2, y2, led_color);
    }
    tft.drawString((String)brightnessStep[0], icon_x_start - 1, icon_y - 22);   // LED 밝기값 표시
    
    // ---------------------- 2. Humidifier Icon ----------------------
    icon_x_start += icon_gap;
    bool humi_on = (digitalRead(HUMIDIFIER_PWR) == HIGH);
    uint16_t humi_color = humi_on ? TFT_CYAN : TFT_DARKGREY;
  
    tft.fillCircle(icon_x_start, icon_y - 3, 8, humi_color);
    tft.fillTriangle(icon_x_start - 8, icon_y - 3, icon_x_start + 8, icon_y - 3, icon_x_start, icon_y + 9, humi_color);

    String modeHumiStr;
    if (humidifierMode == AUTO) modeHumiStr = "AUTO";
    else if (humidifierMode == ON) modeHumiStr = " ON ";
    else modeHumiStr = " OFF";

    tft.drawString(modeHumiStr, icon_x_start - 12, icon_y - 21);
    tft.drawString("      ", icon_x_start - 10, icon_y - 31); 

    // 가습기가 ON일때 카운트다운
    if (humidifierMode == ON && manualHumidifierStartTime > 0) {
      unsigned long elapsedSec = (millis() - manualHumidifierStartTime) / 1000;
      unsigned long limitSec = HUMIDIFIER_AUTO_OFF_MINUTES * 60;
      long remain = (long)limitSec - (long)elapsedSec;
      if (remain < 0) remain = 0; 
      tft.drawString((String)remain, icon_x_start - 6, icon_y - 31);
    }

    // ---------------------- 3. Heater Icon ----------------------
    icon_x_start += icon_gap;
    bool heater_on = (digitalRead(HEATER_PIN) == HIGH); 
    uint16_t heater_color = heater_on ? TFT_RED : TFT_DARKGREY;
    
    int heater_x = icon_x_start;
    int heater_y = icon_y + 4;

    tft.drawBitmap(heater_x - 15, heater_y - 15, hotSpring30, 30, 30, heater_color);

    String modeHeatStr;
    if (heaterMode == AUTO) modeHeatStr = "AUTO";
    else if (heaterMode == ON) modeHeatStr = " ON ";
    else modeHeatStr = " OFF";

    tft.drawString(modeHeatStr, icon_x_start - 12, icon_y - 21);
    tft.drawString("      ", icon_x_start - 10, icon_y - 31); 

    // 히터 ON일때 카운트다운
    if (heaterMode == ON && manualHeaterStartTime > 0) {
      unsigned long elapsedSec = (millis() - manualHeaterStartTime) / 1000;
      unsigned long limitSec = HEATER_AUTO_OFF_MINUTES * 60;
      long remain = (long)limitSec - (long)elapsedSec;
      if (remain < 0) remain = 0;
      tft.drawString((String)remain, icon_x_start - 6, icon_y - 31);
    }

    // ---------------------- 4. Fan Icon ----------------------
    /*
    icon_x_start += icon_gap;
    bool fan_on = (digitalRead(FAN_PIN) == HIGH);
    uint16_t fan_color = fan_on ? TFT_GREEN : TFT_DARKGREY;
    
    int fan_x = icon_x_start;
    int fan_y = icon_y + 2;
    int fan_radius = 12;

    // 팬이 꺼져있을 때만 정지 아이콘 그리기 (켜져있으면 loop에서 애니메이션 처리)
    if (!fan_on) {
         tft.drawCircle(fan_x, fan_y, fan_radius, TFT_DARKGREY);
         tft.fillCircle(fan_x, fan_y, 2, TFT_DARKGREY);
    }

    String modeFanStr = {};
    if (fanMode == AUTO) modeFanStr = "AUTO";
    else if (fanMode == ON) modeFanStr = " ON ";
    else modeFanStr = " OFF";

    tft.drawString(modeFanStr, icon_x_start - 12, icon_y - 21);
    tft.drawString("      ", icon_x_start - 10, icon_y - 31); 

    // 팬이 ON일때 카운트다운
    if (fanMode == ON && manualFanStartTime > 0) {
      unsigned long elapsedSec = (millis() - manualFanStartTime) / 1000;
      unsigned long limitSec = FAN_AUTO_OFF_MINUTES * 60;
      long remain = (long)limitSec - (long)elapsedSec;
      if (remain < 0) remain = 0; 
      tft.drawString((String)remain, icon_x_start - 6, icon_y - 31);
    }
    */

    // ---------------------- 4. Fan Icon (수정됨) ----------------------
    icon_x_start += icon_gap;
    bool fan_on = (digitalRead(FAN_PIN) == HIGH);
    
    int fan_x = icon_x_start;
    int fan_y = icon_y + 2;
    int fan_radius = 12;

    if (!fan_on) {
         // 꺼져있으면 정지 아이콘 그리기
         tft.drawCircle(fan_x, fan_y, fan_radius, TFT_DARKGREY);
         tft.fillCircle(fan_x, fan_y, 2, TFT_DARKGREY);
    } else {
         // [핵심 수정] 켜져있으면, 방금 지워진 배경을 메우기 위해 즉시 한 번 그려줍니다.
         // 이렇게 하면 1초마다 깜빡이는 현상이 사라집니다.
         drawSpinningFan(fan_x, fan_y, fan_radius, TFT_GREEN);
    }

    String modeFanStr = {};
    if (fanMode == AUTO) modeFanStr = "AUTO";
    else if (fanMode == ON) modeFanStr = " ON ";
    else modeFanStr = " OFF";

    tft.drawString(modeFanStr, icon_x_start - 12, icon_y - 21);
    tft.drawString("      ", icon_x_start - 10, icon_y - 31); 

    if (fanMode == ON && manualFanStartTime > 0) {
      unsigned long elapsedSec = (millis() - manualFanStartTime) / 1000;
      unsigned long limitSec = FAN_AUTO_OFF_MINUTES * 60;
      long remain = (long)limitSec - (long)elapsedSec;
      if (remain < 0) remain = 0; 
      tft.drawString((String)remain, icon_x_start - 6, icon_y - 31);
    }

}



void drawLEDUI(int led, int y, int level, bool selected) {
  int barX = 90; int barW = 120; int barH = 16;
  tft.fillRect(0, y, 240, 28, BG_COLOR);
  tft.setTextSize(2); tft.setTextColor(TFT_WHITE, BG_COLOR);
  tft.setCursor(10, y);
  if (selected) tft.print("* "); else tft.print("  ");
  tft.printf("LED%d", led);
  tft.drawRect(barX, y, barW, barH, TFT_WHITE);
  int fillW = map(level, 0, BRIGHTNESS_STEPS, 0, barW - 2);
  if (fillW > 0) tft.fillRect(barX + 1, y + 1, fillW, barH - 2, TFT_GREEN);
}


void drawGraphFrame() { tft.drawRect(layout_graph_x - 1, layout_graph_y - 1, layout_graph_w + 2, layout_graph_h + 2, TFT_WHITE); }

int getSecondsPerPixel() {
    switch (displayHours) {
        case 1:  return 15; case 6:  return 90;
        case 12: return 180; case 24: return 360;
    }
    return 15;
}



// 스프라이트에 격자 그리기 (좌표 보정 적용)
void drawGraphGrid(const TimeAxisState &axis) {
  const int Y_GRID_STEP = 20;
  for (int v = Y_MIN + Y_GRID_STEP; v <= Y_MAX; v += Y_GRID_STEP) {
    int y = map(v, Y_MIN, Y_MAX, layout_graph_h, 0);                                          // [보정] layout_graph_y+layout_graph_h -> layout_graph_h, layout_graph_y -> 0
    // tft.drawLine(...) 대신 graphSprite.drawLine(...) 사용
    // X좌표도 절대좌표가 아닌 스프라이트 내부 좌표(0 ~ layout_graph_w-1) 사용
    graphSprite.drawLine(0, y, layout_graph_w - 1, y, GRID_COLOR);
  }
  for (int i = 0; i < layout_num_labels; i++) {
    float offsetMin = (layout_num_labels - 1 - i) * axis.labelStepMin;
    // [보정] anchorX는 절대좌표이므로 layout_graph_x를 빼서 스프라이트 상대좌표로 변환
    int x = (axis.anchorX - layout_graph_x) - (int)((offsetMin / axis.minPerPx) + 0.5f);
    if (x < 0 || x >= layout_graph_w) continue;                                               // [보정] 범위 체크도 0 ~ layout_graph_w 기준
    graphSprite.drawFastVLine(x, 0, layout_graph_h, GRID_COLOR);                              // [보정] y시작점 0
  }
}




void drawGraphLabels(const TimeAxisState &axis) {
  // 1. 시간 스케일 표시 (1H, 6H 등)
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKCYAN, BG_COLOR);
  tft.setCursor(layout_graph_x + 8, layout_graph_y + 8);
  tft.printf("%dH", displayHoursOptions[displayHoursIndex]);

  // 2. Y축 라벨 (온도/습도 숫자) 그리기
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKCYAN, BG_COLOR);

  for (int v = Y_MIN + 20; v <= Y_MAX; v += 20) {
    int y = map(v, Y_MIN, Y_MAX, layout_graph_y + layout_graph_h, layout_graph_y);
    
    if (screenRotation == 1) {
      tft.setCursor(layout_graph_x - 18, y - 4);                         // LANDSCAPE (가로) 좌표 보정
    } else { 
      tft.setCursor(layout_graph_x - 16, y - 2);                         // PORTRAIT (세로) 좌표 보정
    }
    tft.printf("%d", v);
  }

  // Y축 최솟값(0) 그리기
  int y0 = layout_graph_y + layout_graph_h;
  if (screenRotation == 1) {                                              // LANDSCAPE
     tft.setCursor(layout_graph_x - 12, y0 - 6);
  } else {                                                                // PORTRAIT
     tft.setCursor(layout_graph_x - 8, y0 - 4);
  }
  tft.printf("%d", Y_MIN);

  // 3. X축 라벨 (시간) 그리기
  // 그래프 아래쪽 영역 지우기 (잔상 제거)

  tft.setTextSize(1);                                                       // FreeFont는 기본 크기 사용

  tft.fillRect(0, layout_graph_y + layout_graph_h + 4, layout_graph_w + 44, 14, BG_COLOR);

  for (int i = 0; i < layout_num_labels; i++) {
    float offsetMin = (layout_num_labels - 1 - i) * axis.labelStepMin;
    int labelMin = axis.anchorMin - (int)(offsetMin + 0.5f);
    labelMin = (labelMin + 1440) % 1440;
    
    int x = axis.anchorX - (int)((offsetMin / axis.minPerPx) + 0.5f);
    
    // 그래프 범위를 벗어나면 그리지 않음
    if (x < layout_graph_x || x > layout_graph_x + layout_graph_w) continue;
    
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", labelMin / 60, labelMin % 60);
    tft.setTextColor(TFT_DARKGREEN, BG_COLOR);

    // #if defined 대신 변수로 분기
    if (screenRotation == 1) {                                                      // LANDSCAPE
        tft.setCursor(x - 14 + 4, layout_graph_y + layout_graph_h + 6);
    } else {                                                                        // PORTRAIT
        tft.setCursor(x - 20 + 4, layout_graph_y + layout_graph_h + 5);
    }
    
    tft.print(buf); 
    // tft.print("     "); // (선택) 공백 출력은 상황에 따라 제거 가능
  }

  // 4. 그래프 외곽선 그리기
  tft.drawRect(layout_graph_x - 1, layout_graph_y - 1, layout_graph_w + 2, layout_graph_h + 2, TFT_WHITE);
}









// 스프라이트에 데이터 선 그리기 (좌표 보정 적용)
void drawGraphData() {
    if (!timeSynced) return;
    time_t nowTs = time(nullptr);
    // int graphBottom = layout_graph_y + layout_graph_h; // [삭제] 안 씀
    float scale = (float)layout_graph_h / (Y_MAX - Y_MIN);
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

        // [보정] 절대 X좌표 계산 후 layout_graph_x를 빼서 스프라이트 좌표로 변환
        int currAbsX = layout_graph_x + layout_graph_w - (int)((float)secondsAgo / secondsPerPixel);
        int currX = currAbsX - layout_graph_x;

        if (currX < 0 || currX >= layout_graph_w) { // [보정] 범위 체크 0 ~ layout_graph_w
             prevTempX = -1; prevHumiX = -1; prevValidTs = 0; continue;
        }

        if (rec.temp != INVALID_VALUE) {
            // [보정] Y좌표 계산: layout_graph_h(바닥) 기준으로 계산
            int currTempY = layout_graph_h - (int)(((rec.temp / 10.0f) - Y_MIN) * scale + 0.5f);
            currTempY = constrain(currTempY, 0, layout_graph_h - 1); // [보정] 0 ~ layout_graph_h-1
            if (prevTempX != -1) {
                // [보정] graphSprite에 그리기
                graphSprite.drawLine(prevTempX, prevTempY, currX, currTempY, TFT_YELLOW);
            }
            prevTempX = currX; prevTempY = currTempY;
        } else { prevTempX = -1; }

        if (rec.humi != INVALID_VALUE) {
            // [보정] Y좌표 계산
            int currHumiY = layout_graph_h - (int)(((rec.humi / 10.0f) - Y_MIN) * scale + 0.5f);
            currHumiY = constrain(currHumiY, 0, layout_graph_h - 1); // [보정]
            if (prevHumiX != -1) {
                // [보정] graphSprite에 그리기
                graphSprite.drawLine(prevHumiX, prevHumiY, currX, currHumiY, TFT_GREEN);
            }
            prevHumiX = currX; prevHumiY = currHumiY;
        } else { prevHumiX = -1; }

        if (rec.ts != 0 && rec.ts != 0xFFFFFFFF && rec.temp != INVALID_VALUE && rec.humi != INVALID_VALUE) {
             prevValidTs = rec.ts;
        }
  }
}




// 메인 그래프 그리기 함수
void drawGraph() {
  TimeAxisState axis;
  computeTimeAxis(axis);

  // 1. 스프라이트를 배경색으로 채움 (메모리상에서 지우기)
  graphSprite.fillSprite(BG_COLOR); 
  
  // 2. 스프라이트에 격자와 데이터 그림
  drawGraphGrid(axis);
  drawGraphData();
  
  // 3. 완성된 스프라이트를 실제 화면의 지정된 위치에 전송 (깜빡임 없이 표시됨)
  graphSprite.pushSprite(layout_graph_x, layout_graph_y);

  // 4. 라벨과 외곽선은 스프라이트 바깥이므로 기존 tft 객체로 그림
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

    drawStatusIcons();

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
    labelScrollIntervalMs = (displayHours * 3600UL * 1000UL) / layout_graph_w;
    secondsPerPixel = (float)(displayHours * 3600) / layout_graph_w;
}



void changeGraphMode() {
   displayHoursIndex = (displayHoursIndex + 1) % NUM_GRAPH_MODES;
   updateGraphTimeScale();
   drawGraph();
}


/*
void setHumidifierHwToggle() {
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
*/




// hardware ON sequence
void setHumidifierHwOn() {
  //if (digitalRead(HUMIDIFIER_PWR) == HIGH) return; // Already on
  digitalWrite(HUMIDIFIER_PWR, HIGH);
  delay(100);
  digitalWrite(HUMIDIFIER_RUN, LOW);               // 가습기 버튼 Pressed (RUN)
  delay(100);
  digitalWrite(HUMIDIFIER_RUN, HIGH);              // 가습기 버튼 Released
}

// hardware OFF sequence
void setHumidifierHwOff() {
  digitalWrite(HUMIDIFIER_RUN, HIGH);              // 가습기 버튼 Released
  digitalWrite(HUMIDIFIER_PWR, LOW);               // 가습기 전원 OFF
}





void humidifierOn() {           // AUTO, ON 상태 구분필요 // humiMode 변수값 수정금지 -> handleSetEnvironment() 여기서 수정함
  if (humidifierMode != OFF) setHumidifierHwOn();
  if (humidifierMode == ON && manualHumidifierStartTime == 0) manualHumidifierStartTime = millis();    // 선택1(함수호출해도 타이머 유지됨)
  //if (humidifierMode != AUTO) manualHumidifierStartTime = millis();                                  // 선택2(함수호출할때마다 타이머 초기화)
}


void humidifierOff() {
  if (digitalRead(HUMIDIFIER_PWR) == LOW) return; // Already off
  setHumidifierHwOff();
  manualHumidifierStartTime = 0;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void heaterOn() {           // AUTO, ON 상태 구분필요      // heaterMode 변수값 수정금지 -> handleSetEnvironment() 여기서 수정함, ON→OFF 전환은 loop()에서 처리함
  digitalWrite(HEATER_PIN, HIGH);
  if (heaterMode == ON && manualHeaterStartTime == 0) manualHeaterStartTime = millis();    // 선택1(함수호출해도 타이머 유지됨)
  //if (heaterMode != AUTO) manualHeaterStartTime = millis();                              // 선택2(함수호출할때마다 타이머 초기화)
  //if (heaterMode == ON) manualHeaterStartTime = millis();                                // 선택3(함수호출할때마다 타이머 초기화)
}

void heaterOff() {
  digitalWrite(HEATER_PIN, LOW);
  manualHeaterStartTime = 0;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void fanOn() {
  if (fanMode != OFF) digitalWrite(FAN_PIN, HIGH);
  if (fanMode == ON && manualFanStartTime == 0) manualFanStartTime = millis();     // 선택1(함수호출해도 타이머 유지됨)
  //if (fanMode != AUTO) manualFanStartTime = millis();                            // 선택2(함수호출할때마다 타이머 초기화)
}

void fanOff() {
  digitalWrite(FAN_PIN, LOW);
  manualFanStartTime = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////





void checkHumidity() {
  // [수정] 습도 센서 값을 못 읽으면(NaN), 가습기 AUTO 모드는 무조건 끕니다.
  // (습도가 계속 올라가 물바다가 되는 것을 방지)
  if (isnan(lastHumi)) {
    if (humidifierMode == AUTO) humidifierOff();
    return;
  }

  // --- 기존 로직 ---
  if (lastHumi < humiMin && humidifierMode == AUTO)
    humidifierOn();
  else if (lastHumi > humiMax && humidifierMode == AUTO)
    humidifierOff();
  else if (humidifierMode == ON)
    humidifierOn();
  else if (humidifierMode == OFF)
    humidifierOff();
}

void checkTemperature() {
  // [수정] 온도 센서 값을 못 읽으면(NaN), 히터/팬 AUTO 모드는 무조건 끕니다.
  // (히터가 계속 켜져서 사육장이 찜통이 되는 것을 방지 - 매우 중요!)
  if (isnan(lastTemp)) {
    if (heaterMode == AUTO) heaterOff();
    if (fanMode == AUTO) fanOff();
    return;
  }

  // --- 기존 로직 ---
  if (lastTemp < tempMin && heaterMode == AUTO)         heaterOn();
  else if (lastTemp > tempMax && heaterMode == AUTO)    heaterOff();
  else if (heaterMode == ON)                            heaterOn();

  if (lastTemp > tempMax && fanMode == AUTO)            fanOn();              // 중요, "fanMode"
  else if (lastTemp <= tempMax && fanMode == AUTO)      fanOff();             // 중요, "fanMode"
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
  tft.setCursor(10, y_offset); tft.printf("Humi Mode: %s", modeStr.c_str());

  y_offset += 20;
  tft.setCursor(10, y_offset); tft.printf("Humi Range: %d%% - %d%%", humiMin, humiMax);
  y_offset += 20;
  tft.setCursor(10, y_offset); tft.printf("Temp Range: %dC - %dC", tempMin, tempMax);


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
void handleRoot(bool error = false) {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=UTF-8", "");

    server.sendContent(F("<!DOCTYPE html><html><head><title>Login</title><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>body{font-family:-apple-system,system-ui,BlinkMacSystemFont,\"Segoe UI\",\"Roboto\",\"Helvetica Neue\",Arial,sans-serif;background-color:#f4f4f4;margin:0;padding:1em;color:#333}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}h2{text-align:center;color:#007bff}.error{color:#d9534f;text-align:center;margin-bottom:1em;font-weight:bold}label{display:block;margin-bottom:.5em;font-weight:bold}input[type='text'],input[type='password']{width:100%;padding:.8em;margin-bottom:1em;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}input[type='submit']{width:100%;background-color:#007bff;color:#fff;padding:1em;border:none;border-radius:4px;cursor:pointer;font-size:1em;font-weight:bold}input[type='submit']:hover{background-color:#0056b3}</style></head><body><div class=\"container\"><h2>Device Login</h2>"));
    if (error) {
        server.sendContent(F("<p class='error'>Invalid username or password.</p>"));
    }
    server.sendContent(F("<form method='POST' action='/login'><label for='username'>Username:</label><input id='username' name='username' type='text' value='cage' required><label for='password'>Password:</label><input id='password' name='password' type='password' required><br><br><input type='submit' value='Login'></form></div></body></html>"));
    
    server.sendContent(""); // Finalize stream
}
void handleDashboard() {
    server.sendHeader("Connection", "close"); // [추가]
    const char DASHBOARD_PAGE[] PROGMEM = R"rawliteral(
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
        <a href="/sensorconfig" class="menu-button">시스템 설정</a>
        <a href="/downloadlog" class="menu-button download-button">Download Log File</a>
    </div>
</body>
</html>)rawliteral";
    server.send_P(200, "text/html; charset=UTF-8", DASHBOARD_PAGE);
}

void handleLogin() {
    if (server.hasArg("username") && server.arg("username") == "cage" && server.arg("password") == "cage1234") {
        server.sendHeader("Location", "/dashboard", true);
        server.send(302, "text/plain", "");
    } else {
        handleRoot(true);
    }
}









/*
void handleRemote() {

    server.sendHeader("Connection", "close");             // [수정] 헤더 추가

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=UTF-8", "");

    server.sendContent(F("<!DOCTYPE html><html><head><title>원격제어</title><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>body{font-family:-apple-system,system-ui,BlinkMacSystemFont,\"Segoe UI\",\"Roboto\",\"Helvetica Neue\",Arial,sans-serif;background-color:#f4f4f4;margin:0;padding:1em;color:#333}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}h2{text-align:center;color:#007bff}fieldset{border:1px solid #ddd;border-radius:8px;padding:1.5em;margin-bottom:2em}legend{font-size:1.2em;font-weight:bold;color:#007bff;padding:0 .5em}.radio-group div{margin-bottom:.75em}label{margin-left:.5em;font-weight:normal}input[type='submit']{width:100%;background-color:#28a745;color:#fff;padding:1em;border:none;border-radius:4px;cursor:pointer;font-size:1em;font-weight:bold}input[type='submit']:hover{background-color:#218838}a.back-link{display:inline-block;margin-top:1em;color:#007bff;text-decoration:none}</style></head><body><div class=\"container\"><h2>원격제어</h2><form method='POST' action='/setterminal'><fieldset><legend>자동제어 온/습도 설정</legend>"));
    
    char buffer[256];

    char tempStr[10];
    char humiStr[10];

    if (isnan(lastTemp)) {
        strcpy(tempStr, "N/A");
    } else {
        snprintf(tempStr, sizeof(tempStr), "%.1f", lastTemp);
    }

    if (isnan(lastHumi)) {
        strcpy(humiStr, "N/A");
    } else {
        snprintf(humiStr, sizeof(humiStr), "%.1f", lastHumi);
    }


    snprintf(buffer, sizeof(buffer), "<p style=\"text-align: center; font-weight: bold; color: #337ab7;\">현재 온도: <span id=\"temp_val\">%s</span>&deg;C / 현재 습도: <span id=\"humi_val\">%s</span>%%</p>", tempStr, humiStr);
    server.sendContent(buffer);

     server.sendContent(F("<div style=\"display: grid; grid-template-columns: 1fr 1fr; gap: 1em;\">"));

    // Max temp input
    snprintf(buffer, sizeof(buffer), "<div><label for=\"temp_max\">최고 온도(°C)  </label><input type=\"number\" id=\"temp_max\" name=\"temp_max\" value=\"%d\" min=\"0\" max=\"100\"></div>", tempMax);
    server.sendContent(buffer);

    // Max humidity input
    snprintf(buffer, sizeof(buffer), "<div><label for=\"humi_max\">최고 습도(%%)  </label><input type=\"number\" id=\"humi_max\" name=\"humi_max\" value=\"%d\" min=\"0\" max=\"100\"></div>", humiMax);
    server.sendContent(buffer);

    // Min temp input
    snprintf(buffer, sizeof(buffer), "<div><label for=\"temp_min\">최저 온도(°C)  </label><input type=\"number\" id=\"temp_min\" name=\"temp_min\" value=\"%d\" min=\"0\" max=\"100\"></div>", tempMin);
    server.sendContent(buffer);

    // Min humidity input
    snprintf(buffer, sizeof(buffer), "<div><label for=\"humi_min\">최저 습도(%%)  </label><input type=\"number\" id=\"humi_min\" name=\"humi_min\" value=\"%d\" min=\"0\" max=\"100\"></div>", humiMin);
    server.sendContent(buffer);
    
    server.sendContent(F("</div></fieldset><fieldset><legend>원격제어</legend><div style=\"display: flex; justify-content: space-around;\">"));

    // Humidifier Controls
    server.sendContent(F("<div><strong>가습기</strong><div class=\"radio-group\"><div><input type=\"radio\" id=\"humi_auto\" name=\"humi_mode\" value=\"auto\" "));
    if (humidifierMode == AUTO) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"humi_auto\">AUTO</label></div><div><input type=\"radio\" id=\"humi_on\" name=\"humi_mode\" value=\"on\" "));
    if (humidifierMode == ON) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"humi_on\">ON</label></div><div><input type=\"radio\" id=\"humi_off\" name=\"humi_mode\" value=\"off\" "));
    if (humidifierMode == OFF) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"humi_off\">OFF</label></div></div></div>"));


    // Heater Controls (수정후)
    server.sendContent(F("<div><strong>히터</strong><div class=\"radio-group\"><div><input type=\"radio\" id=\"heat_auto\" name=\"heat_mode\" value=\"auto\" "));
    if (heaterMode == AUTO) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"heat_auto\">AUTO</label></div><div><input type=\"radio\" id=\"heat_on\" name=\"heat_mode\" value=\"on\" "));
    if (heaterMode == ON) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"heat_on\">ON</label></div><div><input type=\"radio\" id=\"heat_off\" name=\"heat_mode\" value=\"off\" "));
    if (heaterMode == OFF) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"heat_off\">OFF</label></div></div></div>"));


    // Fan Controls (수정후)
    server.sendContent(F("<div><strong>팬</strong><div class=\"radio-group\"><div><input type=\"radio\" id=\"fan_auto\" name=\"fan_mode\" value=\"auto\" "));
    if (fanMode == AUTO) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"fan_auto\">AUTO</label></div><div><input type=\"radio\" id=\"fan_on\" name=\"fan_mode\" value=\"on\" "));
    if (fanMode == ON) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"fan_on\">ON</label></div><div><input type=\"radio\" id=\"fan_off\" name=\"fan_mode\" value=\"off\" "));
    if (fanMode == OFF) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"fan_off\">OFF</label></div></div></div>"));



    //server.sendContent(F("</div></div></fieldset>"));
    server.sendContent(F("</div></fieldset>"));
    server.sendContent(F("<input type='submit' value='설정 저장'></form><a href=\"/dashboard\" class=\"back-link\"> &larr; Dashboard로 돌아가기</a></div>"));
    
    server.sendContent(F("<script>setInterval(function(){fetch('/sensordata').then(response=>response.json()).then(data=>{document.getElementById('temp_val').innerText=data.temp;document.getElementById('humi_val').innerText=data.humi;}).catch(error=>console.error('Error fetching sensor data:',error));},2000);
      </script><style>input[type='submit']{width:auto;}</style></body></html>"));
    server.sendContent(""); // Finalize stream
}
*/





void handleRemote() {
    // [중요 수정 1] 브라우저에게 "응답 받으면 연결 끊어라"고 지시 (소켓 고갈 방지)
    server.sendHeader("Connection", "close"); 

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=UTF-8", "");

    // --- HTML 헤더 및 스타일 ---
    server.sendContent(F("<!DOCTYPE html><html><head><title>원격제어</title><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>body{font-family:-apple-system,system-ui,BlinkMacSystemFont,\"Segoe UI\",\"Roboto\",\"Helvetica Neue\",Arial,sans-serif;background-color:#f4f4f4;margin:0;padding:1em;color:#333}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}h2{text-align:center;color:#007bff}fieldset{border:1px solid #ddd;border-radius:8px;padding:1.5em;margin-bottom:2em}legend{font-size:1.2em;font-weight:bold;color:#007bff;padding:0 .5em}.radio-group div{margin-bottom:.75em}label{margin-left:.5em;font-weight:normal}input[type='submit']{width:100%;background-color:#28a745;color:#fff;padding:1em;border:none;border-radius:4px;cursor:pointer;font-size:1em;font-weight:bold}input[type='submit']:hover{background-color:#218838}a.back-link{display:inline-block;margin-top:1em;color:#007bff;text-decoration:none}</style></head><body><div class=\"container\"><h2>원격제어</h2><form method='POST' action='/setterminal'><fieldset><legend>자동제어 온/습도 설정</legend>"));
    
    char buffer[256];
    char tempStr[10];
    char humiStr[10];

    // 현재 값 포맷팅
    if (isnan(lastTemp)) { strcpy(tempStr, "N/A"); } 
    else { snprintf(tempStr, sizeof(tempStr), "%.1f", lastTemp); }

    if (isnan(lastHumi)) { strcpy(humiStr, "N/A"); } 
    else { snprintf(humiStr, sizeof(humiStr), "%.1f", lastHumi); }

    // 현재 상태 표시
    snprintf(buffer, sizeof(buffer), "<p style=\"text-align: center; font-weight: bold; color: #337ab7;\">현재 온도: <span id=\"temp_val\">%s</span>&deg;C / 현재 습도: <span id=\"humi_val\">%s</span>%%</p>", tempStr, humiStr);
    server.sendContent(buffer);

    server.sendContent(F("<div style=\"display: grid; grid-template-columns: 1fr 1fr; gap: 1em;\">"));

    // --- 설정값 입력 필드 ---
    snprintf(buffer, sizeof(buffer), "<div><label for=\"temp_max\">최고 온도(°C)  </label><input type=\"number\" id=\"temp_max\" name=\"temp_max\" value=\"%d\" min=\"0\" max=\"100\"></div>", tempMax);
    server.sendContent(buffer);

    snprintf(buffer, sizeof(buffer), "<div><label for=\"humi_max\">최고 습도(%%)  </label><input type=\"number\" id=\"humi_max\" name=\"humi_max\" value=\"%d\" min=\"0\" max=\"100\"></div>", humiMax);
    server.sendContent(buffer);

    snprintf(buffer, sizeof(buffer), "<div><label for=\"temp_min\">최저 온도(°C)  </label><input type=\"number\" id=\"temp_min\" name=\"temp_min\" value=\"%d\" min=\"0\" max=\"100\"></div>", tempMin);
    server.sendContent(buffer);

    snprintf(buffer, sizeof(buffer), "<div><label for=\"humi_min\">최저 습도(%%)  </label><input type=\"number\" id=\"humi_min\" name=\"humi_min\" value=\"%d\" min=\"0\" max=\"100\"></div>", humiMin);
    server.sendContent(buffer);
    
    server.sendContent(F("</div></fieldset><fieldset><legend>원격제어</legend><div style=\"display: flex; justify-content: space-around;\">"));

    // --- 라디오 버튼 (가습기) ---
    server.sendContent(F("<div><strong>가습기</strong><div class=\"radio-group\"><div><input type=\"radio\" id=\"humi_auto\" name=\"humi_mode\" value=\"auto\" "));
    if (humidifierMode == AUTO) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"humi_auto\">AUTO</label></div><div><input type=\"radio\" id=\"humi_on\" name=\"humi_mode\" value=\"on\" "));
    if (humidifierMode == ON) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"humi_on\">ON</label></div><div><input type=\"radio\" id=\"humi_off\" name=\"humi_mode\" value=\"off\" "));
    if (humidifierMode == OFF) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"humi_off\">OFF</label></div></div></div>"));

    // --- 라디오 버튼 (히터) ---
    server.sendContent(F("<div><strong>히터</strong><div class=\"radio-group\"><div><input type=\"radio\" id=\"heat_auto\" name=\"heat_mode\" value=\"auto\" "));
    if (heaterMode == AUTO) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"heat_auto\">AUTO</label></div><div><input type=\"radio\" id=\"heat_on\" name=\"heat_mode\" value=\"on\" "));
    if (heaterMode == ON) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"heat_on\">ON</label></div><div><input type=\"radio\" id=\"heat_off\" name=\"heat_mode\" value=\"off\" "));
    if (heaterMode == OFF) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"heat_off\">OFF</label></div></div></div>"));

    // --- 라디오 버튼 (팬) ---
    server.sendContent(F("<div><strong>팬</strong><div class=\"radio-group\"><div><input type=\"radio\" id=\"fan_auto\" name=\"fan_mode\" value=\"auto\" "));
    if (fanMode == AUTO) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"fan_auto\">AUTO</label></div><div><input type=\"radio\" id=\"fan_on\" name=\"fan_mode\" value=\"on\" "));
    if (fanMode == ON) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"fan_on\">ON</label></div><div><input type=\"radio\" id=\"fan_off\" name=\"fan_mode\" value=\"off\" "));
    if (fanMode == OFF) server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"fan_off\">OFF</label></div></div></div>"));

    // --- 하단 버튼 및 링크 ---
    server.sendContent(F("</div></fieldset>"));
    server.sendContent(F("<input type='submit' value='설정 저장'></form><a href=\"/dashboard\" class=\"back-link\"> &larr; Dashboard로 돌아가기</a></div>"));
    
    // [중요 수정 2] 자바스크립트를 안전한 방식(재귀적 Timeout)으로 변경
    // 기존 setInterval은 응답 지연 시 요청을 폭주시켰으나, 이 방식은 응답이 오면 다음 요청을 예약하므로 안전함
    server.sendContent(F(
        "<script>"
        "function fetchSensorData() {"
            "fetch('/sensordata')"
            ".then(response => {"
                "if (!response.ok) throw new Error('Network response was not ok');"
                "return response.json();"
            "})"
            ".then(data => {"
                "document.getElementById('temp_val').innerText = data.temp;"
                "document.getElementById('humi_val').innerText = data.humi;"
            "})"
            ".catch(error => console.error('Error fetching data:', error))"
            ".finally(() => {"
                "setTimeout(fetchSensorData, 2000);" // 응답 처리 후 2초 뒤 재요청
            "});"
        "}"
        // 페이지 로드 시 최초 실행
        "fetchSensorData();" 
        "</script>"
    ));

    server.sendContent(F("<style>input[type='submit']{width:auto;}</style></body></html>"));
    server.sendContent(""); // 전송 종료
}







void handleSensorData() {
  server.sendHeader("Connection", "close");         // // [수정] 브라우저에게 연결을 계속 유지하지 말고 끊으라고 명령 (소켓 고갈 방지)

  char json_buffer[100];
  char temp_str[10];
  char humi_str[10];

  if (isnan(lastTemp)) {
    strcpy(temp_str, "N/A");
  } else {
    snprintf(temp_str, sizeof(temp_str), "%.1f", lastTemp);
  }

  if (isnan(lastHumi)) {
    strcpy(humi_str, "N/A");
  } else {
    snprintf(humi_str, sizeof(humi_str), "%.1f", lastHumi);
  }

  snprintf(json_buffer, sizeof(json_buffer), "{\"temp\":\"%s\",\"humi\":\"%s\"}", temp_str, humi_str);
  server.send(200, "application/json", json_buffer);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void handleSetTerminal() {
    bool valuesChanged = false;

     if (server.hasArg("temp_min")) {
        int newMin = server.arg("temp_min").toInt();
        if (newMin >= 0 && newMin <= 100 && newMin != tempMin) {
            tempMin = newMin;
            valuesChanged = true;
        }
    }
    if (server.hasArg("temp_max")) {
        int newMax = server.arg("temp_max").toInt();
        if (newMax >= 0 && newMax <= 100 && newMax != tempMax) {
            tempMax = newMax;
            valuesChanged = true;
        }
    }

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
        preferences.begin("Storage", false);
        preferences.putInt("humiMin", humiMin);
        preferences.putInt("tempMin", tempMin);
        preferences.putInt("tempMax", tempMax);
        preferences.putInt("humiMax", humiMax);
        preferences.end();
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (server.hasArg("humi_mode")) {
        String humiMode = server.arg("humi_mode");      // humi (지역변수), humidifierMode (전역변수, OperMode 타입)
        OperMode oldHumidifierMode = humidifierMode;

        if (humiMode == "on") {
            humidifierMode = ON;
            humidifierOn();                 // This sets humidifierMode to ON
        } else if (humiMode == "off") {
            humidifierMode = OFF;
            humidifierOff();
        } else {
            humidifierMode = AUTO;
            //humidifierOff();
        }

        if (oldHumidifierMode != humidifierMode) {                               // If humiMode changed, save it to Preferences
            preferences.begin("Storage", false);                                 // 저장소를 공통으로 사용함(온도,습도,가습기,히터,팬) (-> Terminal)
            preferences.putInt("humiMode", static_cast<int>(humidifierMode));
            preferences.end();
        }
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (server.hasArg("heat_mode")) {
      String heatMode = server.arg("heat_mode");          // heatMode 지역변수, heaterMode 전역변수
      OperMode oldHeaterMode = heaterMode;

        if (heatMode == "on") {
            heaterMode = ON;
            heaterOn();                 // This sets heaterMode to ON
        } else if (heatMode == "off") {
            heaterMode = OFF;
            heaterOff();
        } else {
            heaterMode = AUTO;
            //heaterOff();
        }

      if (oldHeaterMode != heaterMode) {                                 // If humiMode changed, save it to Preferences
          preferences.begin("Storage", false);                           // 저장소를 공통으로 사용함 (-> Terminal)
          preferences.putInt("heatMode", static_cast<int>(heaterMode));
          preferences.end();
      }
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (server.hasArg("fan_mode")) {
        String _fanMode = server.arg("fan_mode");          // _fanMode 지역변수, fanMode 전역변수
        OperMode oldFanMode = fanMode;

        if (_fanMode == "on") {
            fanMode = ON;
            fanOn();
        } else if (_fanMode == "off") {
            fanMode = OFF;
            fanOff();
        } else {
            fanMode = AUTO;
            //fanOff(); 
        }

        if (oldFanMode != fanMode) {                                       // If humiMode changed, save it to Preferences
            preferences.begin("Storage", false);                           // 저장소를 공통으로 사용함 (-> Terminal)
            preferences.putInt("fanMode", static_cast<int>(fanMode));
            preferences.end();
        }
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // After setting, show the page again to confirm the change
    handleRemote();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void handleConfig() {
    server.sendHeader("Connection", "close"); // [추가]

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=UTF-8", "");

    preferences.begin("wifi", true);
    String savedSsid = preferences.getString("ssid", "");
    String savedPass = preferences.getString("password", "");
    preferences.end();

    server.sendContent(F("<!DOCTYPE html><html><head><title>WiFi Setup</title><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>body{font-family:-apple-system,system-ui,BlinkMacSystemFont,\"Segoe UI\",\"Roboto\",\"Helvetica Neue\",Arial,sans-serif;background-color:#f4f4f4;margin:0;padding:1em;color:#333}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}h2{text-align:center;color:#007bff}p.info{text-align:center;margin-bottom:2em;color:#666;font-size:.9em}fieldset{border:1px solid #ddd;border-radius:8px;padding:1.5em;margin-bottom:2em}legend{font-size:1.2em;font-weight:bold;color:#007bff;padding:0 .5em}label{display:block;margin-bottom:.5em;font-weight:bold}input[type='text'],input[type='password']{width:100%;padding:.8em;margin-bottom:1em;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}input[type='submit']{width:100%;background-color:#28a745;color:#fff;padding:1em;border:none;border-radius:4px;cursor:pointer;font-size:1em;font-weight:bold}input[type='submit']:hover{background-color:#218838}</style></head><body><div class=\"container\"><h2>Device Configuration</h2><form method='POST' action='/save'><fieldset><legend>WiFi Settings</legend><p class=\"info\">Enter WiFi details for internet access.</p>"));
    
    char buffer[512];
    snprintf(buffer, sizeof(buffer), "<label for='ssid'>SSID:</label><input id='ssid' name='ssid' type='text' required value='%s'><label for='password'>Password:</label><input id='password' name='password' type='password' value='%s'></fieldset><br><br><input type='submit' value='Save & Reboot'></form></div></body></html>", savedSsid.c_str(), savedPass.c_str());
    server.sendContent(buffer);

    server.sendContent("");       // Finalize stream
}
void handleNTPConfig() { 
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=UTF-8", "");

    preferences.begin("wifi", true);
    String savedNtpMode = preferences.getString("ntpMode", "builtin1");
    String savedCustomNtp = preferences.getString("ntpServer", "");
    preferences.end();

    server.sendContent(F("<!DOCTYPE html><html><head><title>NTP Setup</title><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>body{font-family:-apple-system,system-ui,BlinkMacSystemFont,\"Segoe UI\",\"Roboto\",\"Helvetica Neue\",Arial,sans-serif;background-color:#f4f4f4;margin:0;padding:1em;color:#333}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}h2{text-align:center;color:#007bff}p.info{text-align:center;margin-bottom:2em;color:#666;font-size:.9em}fieldset{border:1px solid #ddd;border-radius:8px;padding:1.5em;margin-bottom:2em}legend{font-size:1.2em;font-weight:bold;color:#007bff;padding:0 .5em}label{display:block;margin-bottom:.5em;font-weight:bold}input[type='text']{width:100%;padding:.8em;margin-bottom:1em;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}input[type='radio']{margin-right:.5em}input[type='submit']{width:100%;background-color:#28a745;color:#fff;padding:1em;border:none;border-radius:4px;cursor:pointer;font-size:1em;font-weight:bold}input[type='submit']:hover{background-color:#218838}</style></head><body><div class=\"container\"><h2>NTP Server Configuration</h2><form method='POST' action='/ntpsave'><fieldset><legend>NTP Server Settings</legend><p class=\"info\">Choose your NTP server:</p><div style=\"display: flex; align-items: center; margin-bottom: 1em;\"><input type=\"radio\" id=\"ntp_builtin1\" name=\"ntpMode\" value=\"builtin1\" "));
    if (savedNtpMode == "builtin1") server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"ntp_builtin1\" style=\"margin-bottom: 0;\">time.bora.net (default)</label></div><div style=\"display: flex; align-items: center; margin-bottom: 1em;\"><input type=\"radio\" id=\"ntp_builtin2\" name=\"ntpMode\" value=\"builtin2\" "));
    if (savedNtpMode == "builtin2") server.sendContent(F("checked"));
    server.sendContent(F("><label for=\"ntp_builtin2\" style=\"margin-bottom: 0;\">time.kriss.re.kr</label></div><div style=\"display: flex; align-items: center; margin-top: 1em;\"><input type=\"radio\" id=\"ntp_custom\" name=\"ntpMode\" value=\"custom\" "));
    if (savedNtpMode == "custom") server.sendContent(F("checked"));
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "><input id='customNtpServer' name='customNtpServer' type='text' placeholder=\"e.g., pool.ntp.org\" style=\"flex-grow: 1; margin-left: 0.5em;\" value=\"%s\"></div></fieldset><br><br><input type='submit' value='Save & Reboot'></form></div></body></html>", savedCustomNtp.c_str());
    server.sendContent(buffer);

    server.sendContent("");      // Finalize stream
}
const char SAVE_SUCCESS_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Saved</title><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{font-family:-apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif;background-color:#f4f4f4;margin:0;padding:1em;text-align:center}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}p{font-size:1.2em}</style></head><body><div class="container"><p>Saved successfully!</p><p>The device will now reboot...</p></div></body></html>)rawliteral";

const char SSID_ERROR_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Error</title><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{font-family:-apple-system,system-ui,BlinkMacSystemFont,"Segoe UI","Roboto","Helvetica Neue",Arial,sans-serif;background-color:#f4f4f4;margin:0;padding:1em;text-align:center}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}p{font-size:1.2em;color:#d9534f}a{color:#007bff;text-decoration:none;font-weight:bold}</style></head><body><div class="container"><p>SSID cannot be empty.</p><p><a href="/">Please go back and try again.</a></p></div></body></html>)rawliteral";

void handleSave() { 
    if (server.hasArg("ssid") && server.hasArg("password")) { 
        String ssid = server.arg("ssid"); 
        if(ssid.length()==0){
            server.send_P(400,"text/html; charset=UTF-8", SSID_ERROR_PAGE); 
            return;
        }
        String pass = server.arg("password"); 
        preferences.begin("wifi",false); 
        preferences.putString("ssid",ssid); 
        preferences.putString("password",pass); 
        preferences.end(); 
        server.send_P(200,"text/html; charset=UTF-8", SAVE_SUCCESS_PAGE); 
        delay(2000); 
        ESP.restart(); 
    } else { 
        server.send(400, "text/plain", "400: Invalid Request"); 
    } 
}

void handleNTPSave() { 
    String ntpMode = server.arg("ntpMode"); 
    String customNtp = server.arg("customNtpServer"); 
    preferences.begin("wifi",false); 
    preferences.putString("ntpMode",ntpMode); 
    if(ntpMode=="custom"){
        preferences.putString("ntpServer",customNtp);
    } else {
        preferences.remove("ntpServer");
    } 
    preferences.end(); 
    server.send_P(200,"text/html; charset=UTF-8", SAVE_SUCCESS_PAGE); 
    delay(2000); 
    ESP.restart(); 
}


// 웹 핸들러
void handleSensorConfig() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=UTF-8", "");

    String sensorSel = (currentSensorType == 0) ? "SHT41" : "AHT20";
    // 현재 회전 모드 확인
    String rotSel = (screenRotation == 0) ? "PORTRAIT" : "LANDSCAPE";

    server.sendContent(F("<!DOCTYPE html><html><head><title>System Setup</title><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>body{font-family:sans-serif;background-color:#f4f4f4;padding:1em;color:#333}.container{max-width:600px;margin:2em auto;background-color:#fff;padding:2em;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.1)}h2{color:#007bff;text-align:center}fieldset{border:1px solid #ddd;border-radius:8px;padding:1em;margin-bottom:1em}legend{font-weight:bold;color:#007bff}label{font-weight:bold;margin-right:1em}input[type='submit']{width:100%;padding:1em;background-color:#28a745;color:#fff;border:none;border-radius:4px;font-weight:bold;cursor:pointer}</style></head><body><div class=\"container\"><h2>System Configuration</h2><form method='POST' action='/sensorsave'>"));

    // --- 센서 설정 ---
    server.sendContent(F("<fieldset><legend>Sensor Select</legend>"));
    server.sendContent(F("<input type='radio' id='s1' name='sensorType' value='SHT41' "));
    if (sensorSel == "SHT41") server.sendContent(F("checked"));
    server.sendContent(F("><label for='s1'>SHT41</label> "));
    
    server.sendContent(F("<input type='radio' id='s2' name='sensorType' value='AHT20' "));
    if (sensorSel == "AHT20") server.sendContent(F("checked"));
    server.sendContent(F("><label for='s2'>AHT20 + BMP280</label></fieldset>"));

    // --- [추가] 화면 회전 설정 ---
    server.sendContent(F("<fieldset><legend>Screen Rotation</legend>"));
    server.sendContent(F("<input type='radio' id='r1' name='rotation' value='0' "));
    if (screenRotation == 0) server.sendContent(F("checked"));
    server.sendContent(F("><label for='r1'>Portrait (세로)</label> "));

    server.sendContent(F("<input type='radio' id='r2' name='rotation' value='1' "));
    if (screenRotation == 1) server.sendContent(F("checked"));
    server.sendContent(F("><label for='r2'>Landscape (가로)</label></fieldset>"));

    server.sendContent(F("<br><input type='submit' value='Save & Reboot'></form></div></body></html>"));
    server.sendContent(""); 
}








/*
void handleSensorSave() {
    bool changed = false;

    preferences.begin("Storage", false);                          // 쓰기 모드
    
    // 1. 센서 저장
    if (server.hasArg("sensorType")) {
        String sType = server.arg("sensorType");
        int newType = (sType == "AHT20") ? 1 : 0;
        if (currentSensorType != newType) {
            preferences.putInt("sensorType", newType);
            changed = true;
        }
    }

    // 2. 회전 저장
    if (server.hasArg("rotation")) {
        int newRot = server.arg("rotation").toInt();
        if (screenRotation != newRot) {
            preferences.putInt("rotation", newRot);
            changed = true;
        }
    }
    
    preferences.end();

    server.send_P(200, "text/html; charset=UTF-8", SAVE_SUCCESS_PAGE);
    delay(2000);
    ESP.restart();                                                 // 변경 사항 적용을 위해 재부팅
}
*/



void handleSensorSave() {                                           // 화면회전시 재부팅 없이 즉시 적용함
    bool sensorChanged = false;
    bool rotationChanged = false;
    
    preferences.begin("Storage", false); // 쓰기 모드

    // 1. 센서 변경 확인
    if (server.hasArg("sensorType")) {
        int newType = (server.arg("sensorType") == "AHT20") ? 1 : 0;
        if (currentSensorType != newType) {
            preferences.putInt("sensorType", newType);
            sensorChanged = true; // 센서가 바뀌면 재부팅 필요
        }
    }

    // 2. 화면 회전 변경 확인
    if (server.hasArg("rotation")) {
        int newRot = server.arg("rotation").toInt();
        if (screenRotation != newRot) {
            preferences.putInt("rotation", newRot);
            screenRotation = newRot; // 전역 변수 즉시 업데이트
            rotationChanged = true;  // 회전만 바뀌면 즉시 적용 가능
        }
    }
    
    preferences.end();

    // 3. 상황에 따른 처리
    if (sensorChanged) {
        // 센서가 바뀌었으면 하드웨어 초기화를 위해 재부팅 필수
        server.send(200, "text/html", "Sensor changed. Rebooting...");
        delay(1000);
        ESP.restart();
    } 
    else if (rotationChanged) {
        // 화면만 돌아갔으면 재부팅 없이 즉시 적용
        
        // (1) 레이아웃 좌표 재계산
        updateLayout(); 
        
        // (2) LCD 회전 적용
        tft.setRotation(screenRotation); 
        tft.fillScreen(BG_COLOR); // 화면 깨짐 방지를 위해 한번 지움

        // (3) 그래프 스프라이트 크기 재설정 (가로/세로 비율이 바뀌므로 필수)
        graphSprite.deleteSprite();
        graphSprite.setColorDepth(16);
        graphSprite.createSprite(layout_graph_w, layout_graph_h);

        // (4) 화면 요소 다시 그리기
        drawTitle();
        // loadDataForDisplay(); // 데이터는 그대로 있으므로 로드 불필요
        updateGraphTimeScale();
        drawGraphFrame();
        drawGraph();
        drawConditionFace();
        drawStatusIcons(); // 아이콘 위치도 바뀜

        // (5) 웹페이지 응답 (재부팅 안 함)
        server.send(200, "text/html", "Rotation Saved! Applied immediately. <br><a href='/dashboard'>Back to Dashboard</a>");
    } 
    else {
        // 변경된 게 없으면 그냥 복귀
        server.send(200, "text/html", "Nothing changed. <br><a href='/dashboard'>Back to Dashboard</a>");
    }
}





void handleDownloadLog() {
    File logFile = LittleFS.open(LOG_FILE, "r");
    if (!logFile) {
        server.send(404, "text/plain", "Log file not found.");
        return;
    }

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/csv", "");                               // Send header
    
    server.sendContent("Timestamp,Temperature(C),Humidity(%),Epoch\n");

    uint32_t records_to_read = logMeta.record_count;
    uint32_t start_index = (logMeta.head_index + FLASH_MAX_RECORDS - records_to_read) % FLASH_MAX_RECORDS;

    LogRecord rec;
    char line_buffer[128];                                          // Use char buffer to avoid String fragmentation

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
            yield();                                              // Allow system tasks to run, preventing watchdog timeout
        }
    }
    logFile.close();
    server.sendContent("");                                       // End of stream
}



void ntpUpdate() {
    unsigned long now = millis();
    // 1. 아직 시간이 설정 안 됐거나, 
    // 2. 마지막 동기화로부터 하루(NTP_SYNC_INTERVAL)가 지났거나,
    // 3. 시간이 설정 안 된 상태에서 인터넷이 막 연결되었을 때 (재연결 직후 빠른 동기화)
    if (!timeSynced || (now - lastNtpSync >= NTP_SYNC_INTERVAL)) {
        if (WiFi.status() != WL_CONNECTED) return; // 인터넷 없으면 포기
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "pool.ntp.org");
        struct tm timeinfo;
        // 시간 가져오기 성공 시
        if (getLocalTime(&timeinfo, 500)) {                       // 500ms 대기
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


  pinMode(HEATER_PIN, OUTPUT);              // 히터핀 설정
  digitalWrite(HEATER_PIN, LOW);            // 히터 전원차단(OFF)

  pinMode(FAN_PIN, OUTPUT);                 // FAN핀 설정
  digitalWrite(FAN_PIN, LOW);               // FAN 전원차단(OFF)

 
  initFlashStorage();

  // --- [수정] 설정값 불러오기 및 센서 초기화 ---
  preferences.begin("Storage", true);                       // Open in read-only mode
  humiMin = preferences.getInt("humiMin", 45);
  humiMax = preferences.getInt("humiMax", 65);
  tempMin = preferences.getInt("tempMin", 22);
  tempMax = preferences.getInt("tempMax", 32);
  humidifierMode = static_cast<OperMode>(preferences.getInt("humiMode", static_cast<int>(AUTO)));
  heaterMode     = static_cast<OperMode>(preferences.getInt("heatMode", static_cast<int>(AUTO)));
  fanMode        = static_cast<OperMode>(preferences.getInt("fanMode",  static_cast<int>(AUTO)));
  currentSensorType = preferences.getInt("sensorType", 0);              //  센서 타입 로드 (기본값 0: SHT41)
  screenRotation = preferences.getInt("rotation", 0);                   //  화면회전 설정 로드 (기본값 0: 세로)
  preferences.end();


  tft.init();
  // 스프라이트 초기화 (메모리 할당)
  graphSprite.setColorDepth(16); // 16비트 컬러
  graphSprite.createSprite(layout_graph_w, layout_graph_h);     // 그래프 크기만큼 생성

  //tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(BG_COLOR);

  // 레이아웃 적용 및 화면 회전
  updateLayout(); 
  tft.setRotation(screenRotation); // 0 or 1

  // 스프라이트 크기 재설정 (가로/세로 크기에 맞춰 다시 생성)
  graphSprite.deleteSprite();                                   // 기존꺼 삭제
  graphSprite.setColorDepth(16);
  graphSprite.createSprite(layout_graph_w, layout_graph_h);     // 변수 사용




  // Apply loaded humidifier state after reboot
//////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (humidifierMode == ON)       humidifierOn();
  else if (humidifierMode == OFF) humidifierOff();

  if (heaterMode == ON)           heaterOn();
  else if (heaterMode == OFF)     heaterOff();

  if (fanMode == ON)              fanOn();
  else if (fanMode == OFF)        fanOff();
//////////////////////////////////////////////////////////////////////////////////////////////////////////




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
  Wire.setTimeout(1000);            // Set 1ms timeout for I2C communication
  
  // --- 선택된 센서만 초기화 ---
  if (currentSensorType == 0) {
    sht4.begin(Wire, SHT41_I2C_ADDR);           // SHT41 초기화
    delay(10); 
    sht4.softReset();
  } else {                                      // AHT20 + BMP280 초기화
    if (!aht20.begin()) {}                      // 에러 처리 생략
    if (!bmp280.begin(0x76)) {}                 // 에러 처리 생략
  }
  


  drawTitle();
  loadDataForDisplay();
  updateGraphTimeScale();
  drawGraphFrame();
  drawGraph();
  drawConditionFace();
  drawStatusIcons();



  startAP();
  server.on("/", HTTP_GET, [](){ handleRoot(false); });
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/dashboard", HTTP_GET, handleDashboard);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/ntpconfig", HTTP_GET, handleNTPConfig);
  server.on("/remote", HTTP_GET, handleRemote);
  server.on("/setterminal", HTTP_POST, handleSetTerminal);        // (가습기,히터,팬 통합)
  server.on("/sensorconfig", HTTP_GET, handleSensorConfig);
  server.on("/sensorsave", HTTP_POST, handleSensorSave);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/ntpsave", HTTP_POST, handleNTPSave);
  server.on("/downloadlog", HTTP_GET, handleDownloadLog);
  server.on("/sensordata", HTTP_GET, handleSensorData);
  server.begin();
  delay(50);
  if (savedSsid.length() > 0) {
    WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) delay(500);
  }
  delay(50);
  externalAPConnected = (WiFi.status() == WL_CONNECTED);
  delay(50);
}




// =============================================================================================================================
void loop() {
  unsigned long nowMs = millis();
  
  server.handleClient();
  handleEncoderButton();


  // Check for manual humidifier auto-off
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (humidifierMode == ON && manualHumidifierStartTime != 0) {
    if ((millis() - manualHumidifierStartTime) >= (HUMIDIFIER_AUTO_OFF_MINUTES * 60 * 1000UL)) {
      humidifierOff();
      humidifierMode = OFF;                                                 // 상태변경
      manualHumidifierStartTime = 0;
      preferences.begin("Storage", false);                                  // 상태저장
      preferences.putInt("humiMode", static_cast<int>(humidifierMode));
      preferences.end();
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (heaterMode == ON &&  manualHeaterStartTime != 0) {
    if ((millis() - manualHeaterStartTime) >= (HEATER_AUTO_OFF_MINUTES * 60 * 1000UL)) {
      heaterOff();
      heaterMode = OFF;                       // 수동 OFF 상태로 변경
      manualHeaterStartTime = 0;
      preferences.begin("Storage", false);
      preferences.putInt("heatMode", static_cast<int>(heaterMode));
      preferences.end();
   }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (fanMode == ON && manualFanStartTime != 0) {
    if ((millis() - manualFanStartTime) >= (FAN_AUTO_OFF_MINUTES * 60 * 1000UL)) {
      fanOff();
      fanMode = OFF;                          // 수동 OFF 상태로 변경
      manualFanStartTime = 0;
      preferences.begin("Storage", false);
      preferences.putInt("fanMode", static_cast<int>(fanMode));
      preferences.end();
   }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////



  // Handle system info display overlay
  if (sysInfoDisplayUntil > 0) {
    if (nowMs >= sysInfoDisplayUntil) {
      sysInfoDisplayUntil = 0;
      tft.fillScreen(BG_COLOR);                     // 화면 초기화
      drawTitle();
      drawTimeTempHumi(lastTemp, lastHumi, true);
      drawConditionFace();
      drawStatusIcons();
      drawGraphFrame();
      drawGraph();
    }
    //return; // Do nothing else while info is displayed
    // [수정] 여기에 있던 return; 을 삭제함
    // 이유: 이걸 지워야 정보창이 떠 있는 3초 동안에도 히터 제어와 웹서버가 멈추지 않음.  => 1초 주기로 디스플레이 작동으로 화면깨짐발생 (추가수정요망) 
  }
  

  handleEncoderRotation();
  ntpUpdate();

  // --- [수정] WiFi 자동 재연결 로직 (30초마다 체크) ---
  static unsigned long lastWifiCheck = 0;
  const unsigned long WIFI_CHECK_INTERVAL = 30000;            // 30초마다 한 번씩 연결 상태를 확인
  if (nowMs - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = nowMs;
    // 와이파이가 끊겨 있고, 설정된 SSID가 있다면 재연결 시도
    if (WiFi.status() != WL_CONNECTED) {
       // LCD 상단에 연결 시도 중임을 작게 표시 (선택사항)
       // tft.fillRect(0, 0, 10, 5, TFT_RED);
       WiFi.reconnect(); 
    }
  }



  if (nowMs - lastSample >= SAMPLE_INTERVAL) {
      lastSample = nowMs;
      float temperature, humidity;
      if (readSensor(temperature, humidity)) {
          lastTemp = temperature; lastHumi = humidity;
          accTemp += temperature; accHumi += humidity; accCount++;
      } else { lastTemp = NAN; lastHumi = NAN; }


    // [수정] 정보창이 떠 있지 않을 때만 메인 화면 갱신
    if (sysInfoDisplayUntil == 0) {
        drawTimeTempHumi(lastTemp, lastHumi, true);
        drawConditionFace();
        drawStatusIcons();
    }

  checkHumidity();
  checkTemperature();
  // 온도,습도가 높으면 FAN 가동할 것, checkHumidity() + checkTemperature() => checkEnvironment() 
  }




  if (nowMs - lastGraphSampleMs >= GRAPH_SAMPLE_INTERVAL_SEC * 1000UL) {
      lastGraphSampleMs = nowMs;
      float avgTemp = (accCount > 0) ? (accTemp / accCount) : NAN;
      float avgHumi = (accCount > 0) ? (accHumi / accCount) : NAN;
      
      pushToDisplayBuffer(avgTemp, avgHumi);

      accTemp = 0; accHumi = 0; accCount = 0;

      // [수정] 정보창이 떠 있지 않을 때만 그래프 그리기
      if (sysInfoDisplayUntil == 0) {
          drawGraph();
      }
  }



  // --- 바람개비 부드러운 애니메이션 (0.1초 간격) ---
  static unsigned long lastFanAnimMs = 0;
  
  if (nowMs - lastFanAnimMs >= 100) { // 100ms 마다 갱신
    lastFanAnimMs = nowMs;




    // 팬이 켜져 있을 때만 그립니다. (Portrait 모드 기준 좌표)
    /*
    if (fanMode != OFF && digitalRead(FAN_PIN) == HIGH) {
      // 좌표 계산: icon_x_start(30) + icon_gap(60)*3 = 210
      int fanX = 210; // 기본 세로 모드 위치 (30 + 60*3)
      if (screenRotation == 1) {
          fanX += 75; // 가로 모드일 때 drawStatusIcons에서 이동한 만큼 똑같이 이동
      }
      int fanY = layout_icons_y + 6 + 2; // 140 + 6 + 2 = 148
      // 초록색으로 회전 애니메이션 그리기
      drawSpinningFan(fanX, fanY, 12, TFT_GREEN); 
    }
  }
  */

    // [수정] 정보창이 없고, 팬이 켜져 있을 때만 그리기
    if (sysInfoDisplayUntil == 0 && fanMode != OFF && digitalRead(FAN_PIN) == HIGH) {
      
      int fanX = 210; 
      if (screenRotation == 1) {
          fanX += 75; // 가로 모드 보정
      }
      int fanY = layout_icons_y + 6 + 2; 

      drawSpinningFan(fanX, fanY, 12, TFT_GREEN); 
    }
  }

}
