#define DISABLE_ALL_LIBRARY_WARNINGS

#define ST7789_DRIVER
//#define TFT_RGB_ORDER TFT_RGB
//#define TFT_RGB_ORDER TFT_BGR

// 해상도 설정

#define TFT_WIDTH_OFFSET 20                             // main.cpp, 텍스트 자동 줄바뀜 방지를 위한 가상공간 확보


#define TFT_WIDTH  240 + TFT_WIDTH_OFFSET               // 텍스트 자동 줄바뀜 방지
#define TFT_HEIGHT 320 + TFT_WIDTH_OFFSET

// 핀맵 설정
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC   16
#define TFT_RST  17

// 폰트 로드
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

#define SMOOTH_FONT

// SPI 클럭 주파수 설정
#define SPI_FREQUENCY  27000000
//#define SPI_FREQUENCY 20000000
