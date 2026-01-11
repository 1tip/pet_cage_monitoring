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
  
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
board_build.f_cpu = 240000000L
board_build.f_flash = 80000000L
board_build.flash_mode = qio
board_build.partitions = default.csv

lib_deps =
    ;lorol/LittleFS_esp32@^1.0.6
    bodmer/TFT_eSPI @ ^2.5.43
    adafruit/Adafruit Unified Sensor@^1.1.15
    arduino-libraries/NTPClient@^3.2.1
    sensirion/Sensirion I2C SHT4x@^1.1.2
    adafruit/Adafruit AHTX0@^2.0.5
    adafruit/Adafruit BMP280 Library@^2.6.8

build_flags =
    -D USER_SETUP_LOADED=1
    -D TFT_RGB_ORDER=TFT_BGR
    -I include/
    -include include/User_Setup.h

monitor_speed = 115200
upload_speed = 921600
```


<img width="1009" height="715" alt="image" src="https://github.com/user-attachments/assets/87005772-8779-44cd-8027-194986f76060" />
<img width="1002" height="1003" alt="image" src="https://github.com/user-attachments/assets/ef55b780-c381-4327-91b2-6bab423cd9c1" />




