#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <lvgl.h>
#include <Adafruit_MAX31865.h>
#include "pins.h"
#include "LGFX_Config.hpp"

LGFX_ST7796 tft;

/* =================== LVGL core =================== */
static lv_display_t *disp = nullptr;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;

static const uint32_t LVGL_TICK_MS = 5;
static const int HOR_RES = 480;
static const int VER_RES = 320;

/* ======= Touch transform (tune if needed) ======= */
static bool TOUCH_SWAP_XY = true;
static bool TOUCH_INVERT_X = false;
static bool TOUCH_INVERT_Y = false;

/* ================= GT911 (polling) ================ */
class GT911 {
public:
  bool begin(TwoWire &bus, int sda, int scl, int pinRST = -1, int pinINT = -1) {
    _wire = &bus; _rst = pinRST; _int = pinINT;
    _wire->begin(sda, scl, 400000);

    // Optional reset sequence
    if (_rst >= 0) {
      pinMode(_rst, OUTPUT);
      if (_int >= 0) { pinMode(_int, OUTPUT); digitalWrite(_int, LOW); }
      digitalWrite(_rst, LOW);  delay(10);
      digitalWrite(_rst, HIGH); delay(50);
      if (_int >= 0) { pinMode(_int, INPUT_PULLUP); }
    }

    // Try addresses 0x5D -> 0x14
    _addr = 0x5D;
    if (!probe()) { _addr = 0x14; if (!probe()) return false; }

    // Read native X/Y max to scale properly (little-endian)
    uint8_t xy[4] = {0};
    if (read(0x8048, xy, 4)) {
      _xMax = (uint16_t)xy[0] | ((uint16_t)xy[1] << 8);
      _yMax = (uint16_t)xy[2] | ((uint16_t)xy[3] << 8);
      if (_xMax == 0 || _yMax == 0) { _xMax = 480; _yMax = 320; }
    }
    Serial.printf("[GT911] addr=0x%02X  native=%ux%u\n", _addr, _xMax, _yMax);
    return true;
  }

  bool getPoint(uint16_t &x, uint16_t &y, bool &pressed) {
    uint8_t status = 0;
    if (!read8(0x814E, status)) { pressed = false; return false; }
    uint8_t n = status & 0x0F;
    pressed = ((status & 0x80) && (n > 0));
    if (!pressed) { write8(0x814E, 0x00); return true; }

    // P1
    uint8_t buf[6];
    if (!read(0x8150, buf, sizeof(buf))) { pressed = false; return false; }
    uint16_t rx = ((uint16_t)buf[1] << 8) | buf[0];
    uint16_t ry = ((uint16_t)buf[3] << 8) | buf[2];

    // Transform
    uint16_t tx = rx, ty = ry;
    if (TOUCH_SWAP_XY) { uint16_t t = tx; tx = ty; ty = t; }
    if (TOUCH_INVERT_X) tx = (_xMax > 0 ? (_xMax - 1 - tx) : tx);
    if (TOUCH_INVERT_Y) ty = (_yMax > 0 ? (_yMax - 1 - ty) : ty);

    // Scale to LVGL resolution
    uint32_t sx = (_xMax > 1) ? (uint32_t)tx * (HOR_RES - 1) / (_xMax - 1) : tx;
    uint32_t sy = (_yMax > 1) ? (uint32_t)ty * (VER_RES - 1) / (_yMax - 1) : ty;
    x = (uint16_t)min<uint32_t>(sx, HOR_RES - 1);
    y = (uint16_t)min<uint32_t>(sy, VER_RES - 1);

    write8(0x814E, 0x00); // clear ready flag
    return true;
  }

  uint16_t xMax() const { return _xMax; }
  uint16_t yMax() const { return _yMax; }

private:
  TwoWire *_wire = nullptr;
  uint8_t _addr = 0x5D;
  int _rst = -1, _int = -1;
  uint16_t _xMax = 480, _yMax = 320;

  bool probe() { uint8_t id[4] = {0}; return read(0x8140, id, 4); }

  bool write8(uint16_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg >> 8); _wire->write(reg & 0xFF); _wire->write(val);
    return (_wire->endTransmission() == 0);
  }
  bool read8(uint16_t reg, uint8_t &val) { return read(reg, &val, 1); }
  bool read(uint16_t reg, uint8_t *buf, size_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg >> 8); _wire->write(reg & 0xFF);
    if (_wire->endTransmission(false) != 0) return false;
    size_t got = _wire->requestFrom((int)_addr, (int)len);
    if (got != len) return false;
    for (size_t i = 0; i < len; ++i) buf[i] = _wire->read();
    return true;
  }
};

/* ================= AHT30 (I2C @ 0x38) ================ */
class AHT30 {
public:
  bool begin(TwoWire& w, uint8_t addr = 0x38) {
    _w = &w; _addr = addr;
    writeCmd(0xBA); delay(20);                  // soft reset
    uint8_t initCmd[3] = {0xE1, 0x08, 0x00};    // init/cal
    if (!write(initCmd, 3)) return false;
    delay(10);
    _ok = true; return true;
  }
  bool read(float& tempC, float& rh) {
    if (!_ok) return false;
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};       // trigger measure
    if (!write(cmd, 3)) return false;
    delay(80);

    uint8_t buf[6] = {0};
    if (!read(buf, 6)) return false;

    uint32_t raw = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    uint32_t raw_rh = raw >> 4;
    uint32_t raw_t  = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    rh    = (raw_rh * 100.0f) / 1048576.0f;
    tempC = (raw_t  * 200.0f  / 1048576.0f) - 50.0f;

    if (rh < 0) rh = 0; if (rh > 100) rh = 100;
    return true;
  }
private:
  TwoWire* _w=nullptr; uint8_t _addr=0x38; bool _ok=false;
  bool writeCmd(uint8_t c){ _w->beginTransmission(_addr); _w->write(c); return (_w->endTransmission()==0); }
  bool write(uint8_t* p,size_t n){ _w->beginTransmission(_addr); for(size_t i=0;i<n;i++) _w->write(p[i]); return (_w->endTransmission()==0); }
  bool read(uint8_t* p,size_t n){ size_t got=_w->requestFrom((int)_addr,(int)n); if(got!=n) return false; for(size_t i=0;i<n;i++) p[i]=_w->read(); return true; }
};

/* =============== Instances & UI labels =============== */
static GT911 touch;
static AHT30 aht;
static Adafruit_MAX31865 rtd(PIN_RTD_CS, PIN_RTD_MISO, PIN_RTD_MOSI, PIN_RTD_SCK);

static lv_obj_t* lbl_readout = nullptr;  // AHT30 chamber
static lv_obj_t* lbl_rtd = nullptr;      // PT100 (MAX31865)

/* ================= LVGL callbacks ================= */
static void lv_tick_task(void *arg) {
  (void)arg;
  while (true) { lv_tick_inc(LVGL_TICK_MS); vTaskDelay(pdMS_TO_TICKS(LVGL_TICK_MS)); }
}

static void my_flush_cb(lv_display_t *d, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)px_map, w * h);
  tft.endWrite();
  lv_display_flush_ready(d);
}

static void indev_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  (void)indev;
  uint16_t x, y; bool pressed;
  if (!touch.getPoint(x, y, pressed)) { data->state = LV_INDEV_STATE_RELEASED; return; }
  data->point.x = (lv_coord_t)x;
  data->point.y = (lv_coord_t)y;
  data->state   = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/* =============== UI helpers =============== */
static void toast_close_cb(lv_timer_t *t) {
  lv_obj_t *mbox = (lv_obj_t*)lv_timer_get_user_data(t);
  if (mbox) lv_obj_del(mbox);
  lv_timer_del(t);
}
static void btn_clicked(lv_event_t *e) {
  lv_obj_t *btn = lv_event_get_target(e);
  lv_obj_t *lbl = lv_obj_get_child(btn, 0);
  const char *txt = lbl ? lv_label_get_text(lbl) : "Button";
  Serial.printf("[UI] Clicked: %s\n", txt);

  lv_obj_t *m = lv_msgbox_create(NULL);
  lv_msgbox_add_title(m, "Tapped");
  lv_msgbox_add_text(m, txt);
  lv_msgbox_add_close_button(m);
  lv_obj_center(m);
  lv_timer_create(toast_close_cb, 900, m);
}

/* =============== Build home screen =============== */
static void build_home_screen() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "Filament Dryer");
  lv_obj_set_style_text_color(title, lv_color_hex(0xEAEAEA), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lbl_readout = lv_label_create(scr);
  lv_label_set_text(lbl_readout, "Chamber: --.-°C   RH: --%");
  lv_obj_set_style_text_color(lbl_readout, lv_color_hex(0xCFE8FF), 0);
  lv_obj_align(lbl_readout, LV_ALIGN_TOP_MID, 0, 40);

  lbl_rtd = lv_label_create(scr);
  lv_label_set_text(lbl_rtd, "PT100: --.-°C");
  lv_obj_set_style_text_color(lbl_rtd, lv_color_hex(0xFFD27A), 0);
  lv_obj_align(lbl_rtd, LV_ALIGN_TOP_MID, 0, 60);

  auto mkbtn = [&](const char* txt, int xoff){
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 140, 64);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, xoff, -20);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, txt);
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn, btn_clicked, LV_EVENT_CLICKED, nullptr);
  };
  mkbtn("Presets", -170);
  mkbtn("Manual",   0);
  mkbtn("Settings", 170);
}

/* =============== PWM & Buffers =============== */
static void setup_pwm() {
  ledcSetup(0, 1000, 10);  ledcAttachPin(PIN_HEATER_PWM, 0); ledcWrite(0, 0);
  ledcSetup(1, 25000, 10); ledcAttachPin(PIN_FAN_PWM,    1); ledcWrite(1, 0);
}

static void alloc_lvgl_buffers() {
  auto try_alloc = [](int lines, bool dual) -> bool {
    size_t px = (size_t)HOR_RES * (size_t)lines;
    lv_color_t* a = (lv_color_t*)heap_caps_malloc(px*sizeof(lv_color_t),
                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_color_t* b = nullptr;
    if (!a) return false;
    if (dual) {
      b = (lv_color_t*)heap_caps_malloc(px*sizeof(lv_color_t),
                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
      if (!b) { free(a); return false; }
    }
    buf1 = a; buf2 = b;
    lv_display_set_buffers(disp, buf1, buf2, px*sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    Serial.printf("[LVGL] Buffers ok: %s, %d lines\n", dual ? "dual" : "single", lines);
    return true;
  };
  for (int l : {40,30,20,10}) if (try_alloc(l, true)) return;
  for (int l : {20,10})       if (try_alloc(l, false)) return;
  // Last resort tiny single buffer
  int l = 8; size_t px = (size_t)HOR_RES*l;
  buf1 = (lv_color_t*)malloc(px*sizeof(lv_color_t)); buf2 = nullptr;
  lv_display_set_buffers(disp, buf1, buf2, px*sizeof(lv_color_t),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  Serial.println("[LVGL] WARNING: tiny single buffer (malloc).");
}

/* =============== Sensor timer (AHT30 + MAX31865) =============== */
static void sensor_timer_cb(lv_timer_t* t) {
  (void)t;

  // AHT30 chamber
  float tc=0.0f, rh=0.0f;
  if (aht.read(tc, rh)) {
    char buf[64]; snprintf(buf, sizeof(buf), "Chamber: %.1f°C   RH: %.0f%%", tc, rh);
    if (lbl_readout) lv_label_set_text(lbl_readout, buf);
  } else {
    if (lbl_readout) lv_label_set_text(lbl_readout, "Chamber: --.-°C   RH: --%");
  }

  // PT100 (MAX31865) – assumes 430Ω reference (adjust if your board differs)
  float rtdTemp = rtd.temperature(100.0, 430.0);
  if (isnan(rtdTemp) || rtdTemp < -200 || rtdTemp > 850) {
    if (lbl_rtd) lv_label_set_text(lbl_rtd, "PT100: ERR");
  } else {
    char b2[32]; snprintf(b2, sizeof(b2), "PT100: %.1f°C", rtdTemp);
    if (lbl_rtd) lv_label_set_text(lbl_rtd, b2);
  }
}

/* ========================= setup/loop ========================= */
void setup() {
  Serial.begin(115200);
  delay(200);

  // Display
  tft.init();
  tft.setRotation(1);                  // landscape
  if (PIN_TFT_BL >= 0) { pinMode(PIN_TFT_BL, OUTPUT); digitalWrite(PIN_TFT_BL, HIGH); }

  // LVGL
  lv_init();
  disp = lv_display_create(HOR_RES, VER_RES);
  lv_display_set_flush_cb(disp, my_flush_cb);
  alloc_lvgl_buffers();
  xTaskCreatePinnedToCore(lv_tick_task, "lv_tick", 2048, NULL, 3, NULL, 1);

  // I2C (GT911 + AHT30 share the bus)
  Wire.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, 400000);

  // Touch
  if (!touch.begin(Wire, PIN_TOUCH_SDA, PIN_TOUCH_SCL, PIN_TOUCH_RST, PIN_TOUCH_INT)) {
    Serial.println("[GT911] init failed – touch disabled");
  } else {
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, indev_read_cb);
  }

  // UI + PWM
  build_home_screen();
  setup_pwm();

  // Sensors
  if (!aht.begin(Wire)) Serial.println("[AHT30] init failed"); else Serial.println("[AHT30] init OK @ 0x38");
  if (!rtd.begin(MAX31865_3WIRE)) Serial.println("[MAX31865] init failed"); else Serial.println("[MAX31865] init OK (3-wire)");

  // Periodic updates
  lv_timer_create(sensor_timer_cb, 1000, nullptr);
}

void loop() {
  lv_timer_handler();
  delay(5);
}
