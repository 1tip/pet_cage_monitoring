# pet_cage_monitoring
  
  
#define TFT_MOSI 23  
#define TFT_SCLK 18  
#define TFT_CS    5  
#define TFT_DC   16  
#define TFT_RST  17  
  
26 DHT11(온습도센서)  
32 PWM LED#1  
21 PWM LED#2  
12 PWM FAN (Not Available)  
27 Rotary Encoder (DT)  
25 Rotary Encoder (CLK)  
22 Rotary Encoder (SWITCH)  
  
  
  
  
# platformio.ini  
[env:esp32dev]  
platform = espressif32  
board = esp32dev  
framework = arduino  
board_build.f_cpu = 240000000L  
board_build.f_flash = 80000000L  
board_build.flash_mode = qio  
board_build.partitions = default.csv  
lib_deps =   
        bodmer/TFT_eSPI @ ^2.5.43  
        adafruit/DHT sensor library@^1.4.6  
        adafruit/Adafruit Unified Sensor@^1.1.15  
build_flags =   
        -D USER_SETUP_LOADED=1  
        -D TFT_RGB_ORDER=TFT_BGR  
        -I include/  
        -include include/User_Setup.h  
monitor_speed = 115200  
upload_speed = 921600  
