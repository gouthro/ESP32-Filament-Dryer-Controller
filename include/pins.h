#pragma once
// TFT (LovyanGFX config uses these)
#define PIN_TFT_MOSI 13
#define PIN_TFT_MISO 12
#define PIN_TFT_SCLK 14
#define PIN_TFT_CS   15
#define PIN_TFT_DC    2
#define PIN_TFT_RST  -1
#define PIN_TFT_BL   27

// Touch / I2C
#define PIN_TOUCH_SDA 33
#define PIN_TOUCH_SCL 32
#define PIN_TOUCH_INT 21
#define PIN_TOUCH_RST 25

// Heater / Fan PWM
#define PIN_HEATER_PWM 22
#define PIN_FAN_PWM    26

// MAX31865 on VSPI
#define PIN_RTD_CS   5
#define PIN_RTD_MOSI 23
#define PIN_RTD_MISO 19
#define PIN_RTD_SCK  18
#define PIN_RTD_RDY  4  // optional