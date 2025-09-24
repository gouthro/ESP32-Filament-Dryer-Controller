#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <lvgl.h>
#include "pins.h"
#include "LGFX_Config.hpp"

LGFX_ST7796 tft;

/* ---------- LVGL v9 display context ---------- */
static lv_display_t *disp = nullptr;

/* Draw buffers */
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;

static const uint32_t LVGL_TICK_MS = 5;
static const int HOR_RES = 480;
static const int VER_RES = 320;

/* ======== Touch transform config (tweak if needed) ======== */
static bool TOUCH_SWAP_XY = true;
static bool TOUCH_INVERT_X = false;
static bool TOUCH_INVERT_Y = false;

/* ==================== Minimal GT911 driver (polling) ==================== */
class GT911 {
public:
  bool begin(TwoWire &bus, int sda, int scl, int pinRST = -1, int pinINT = -1) {
    _wire = &bus;
    _rst = pinRST;
    _int = pinINT;

    _wire->begin(sda, scl, 400000);

    // Optional reset
    if (_rst >= 0) {
      pinMode(_rst, OUTPUT);
      if (_int >= 0) { pinMode(_int, OUTPUT); digitalWrite(_int, LOW); }
      digitalWrite(_rst, LOW);  delay(10);
      digitalWrite(_rst, HIGH); delay(50);
      if (_int >= 0) { pinMode(_int, INPUT_PULLUP); }
    }

    // Probe typical addresses
    _addr = 0x5D;
    if (!probe()) { _addr = 0x14; if (!probe()) return false; }

    // Read native resolution (little endian)
    // X max @ 0x8048..49, Y max @ 0x804A..4B
    uint8_t xy[4] = {0};
    if (read(0x8048, xy, 4)) {
      _xMax = (uint16_t)xy[0] | ((uint16_t)xy[1] << 8);
      _yMax = (uint16_t)xy[2] | ((uint16_t)xy[3] << 8);
      if (_xMax == 0 || _yMax == 0) { _xMax = 480; _yMax = 320; }
    } else {
      _xMax = 480; _yMax = 320;
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

    // First point (P1)
    uint8_t buf[6];
    if (!read(0x8150, buf, sizeof(buf))) { pressed = false; return false; }
    uint16_t rx = ((uint16_t)buf[1] << 8) | buf[0];
    uint16_t ry = ((uint16_t)buf[3] << 8) | buf[2];

    // Transform
    uint16_t tx = rx, ty = ry;
    if (TOUCH_SWAP_XY) { uint16_t t = tx; tx = ty; ty = t; }
    if (TOUCH_INVERT_X) tx = (_xMax > 0 ? (_xMax - 1 - tx) : tx);
    if (TOUCH_INVERT_Y) ty = (_yMax > 0 ? (_yMax - 1 - ty) : ty);

    uint32_t sx = (_xMax > 1) ? (uint32_t)tx * (HOR_RES - 1) / (_xMax - 1) : tx;
    uint32_t sy = (_yMax > 1) ? (uint32_t)ty * (VER_RES - 1) / (_yMax - 1) : ty;

    x = (uint16_t)min<uint32_t>(sx, HOR_RES - 1);
    y = (uint16_t)min<uint32_t>(sy, VER_RES - 1);

    write8(0x814E, 0x00);  // clear ready flag

    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint > 300) {
      lastPrint = now;
      Serial.printf("[GT911] raw(%u,%u) -> lv(%u,%u)  n=%u\n", rx, ry, x, y, n);
    }
    return true;
  }

  uint16_t xMax() const { return _xMax; }
  uint16_t yMax() const { return _yMax; }

private:
  TwoWire *_wire = nullptr;
  uint8_t _addr = 0x5D;
  int _rst = -1, _int = -1;
  uint16_t _xMax = 480, _yMax = 320;

  bool probe() {
    uint8_t id[4] = {0};
    return read(0x8140, id, 4); // ACK is enough
  }

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

static GT911 touch;

/* ---------- LVGL indev read (pointer) ---------- */
static void indev_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  (void)indev;
  uint16_t x, y; bool pressed;
  if (!touch.getPoint(x, y, pressed)) { data->state = LV_INDEV_STATE_RELEASED; return; }
  data->point.x = (lv_coord_t)x;
  data->point.y = (lv_coord_t)y;
  data->state   = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/* ==================== LVGL plumbing & UI ==================== */
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

/* ---- Toast auto-close timer (C callback, not a lambda) ---- */
static void toast_close_cb(lv_timer_t *t) {
  lv_obj_t *mbox = (lv_obj_t*)lv_timer_get_user_data(t);
  if (mbox) lv_obj_del(mbox);
  lv_timer_del(t);
}

/* ---- Button click handler ---- */
static void btn_clicked(lv_event_t *e) {
  lv_obj_t *btn = lv_event_get_target(e);
  lv_obj_t *lbl = lv_obj_get_child(btn, 0);
  const char *txt = lbl ? lv_label_get_text(lbl) : "Button";

  Serial.printf("[UI] Clicked: %s\n", txt);

  // LVGL 9: create msgbox, then add title/text/close button
  lv_obj_t *m = lv_msgbox_create(NULL);
  lv_msgbox_add_title(m, "Tapped");
  lv_msgbox_add_text(m, txt);
  lv_msgbox_add_close_button(m);
  lv_obj_center(m);

  // Auto-close in ~900 ms
  lv_timer_t *tmr = lv_timer_create(toast_close_cb, 900, m);
  (void)tmr; // tmr is owned by LVGL
}

/* ---- Home screen ---- */
static void build_home_screen() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "Filament Dryer");
  lv_obj_set_style_text_color(title, lv_color_hex(0xEAEAEA), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t *readout = lv_label_create(scr);
  lv_label_set_text(readout, "Chamber: 24.3°C   RH: 18%");
  lv_obj_set_style_text_color(readout, lv_color_hex(0xCFE8FF), 0);
  lv_obj_align(readout, LV_ALIGN_TOP_MID, 0, 40);

  auto mkbtn = [&](const char* txt, int xoff) {
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

/* PWM setup (heater + fan) */
static void setup_pwm() {
  ledcSetup(0, 1000, 10);  ledcAttachPin(PIN_HEATER_PWM, 0); ledcWrite(0, 0);
  ledcSetup(1, 25000, 10); ledcAttachPin(PIN_FAN_PWM,    1); ledcWrite(1, 0);
}

/* Adaptive LVGL buffers (DMA internal RAM preferred) */
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
  // tiny fallback
  int l = 8; size_t px = (size_t)HOR_RES*l;
  buf1 = (lv_color_t*)malloc(px*sizeof(lv_color_t)); buf2 = nullptr;
  lv_display_set_buffers(disp, buf1, buf2, px*sizeof(lv_color_t),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  Serial.println("[LVGL] WARNING: tiny single buffer (malloc).");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Display HW
  tft.init();
  tft.setRotation(1); // landscape
  if (PIN_TFT_BL >= 0) { pinMode(PIN_TFT_BL, OUTPUT); digitalWrite(PIN_TFT_BL, HIGH); }

  // LVGL
  lv_init();
  disp = lv_display_create(HOR_RES, VER_RES);
  lv_display_set_flush_cb(disp, my_flush_cb);
  alloc_lvgl_buffers();

  // LVGL tick task
  xTaskCreatePinnedToCore(lv_tick_task, "lv_tick", 2048, NULL, 3, NULL, 1);

  // Touch init (polling)
  if (!touch.begin(Wire, PIN_TOUCH_SDA, PIN_TOUCH_SCL, PIN_TOUCH_RST, PIN_TOUCH_INT)) {
    Serial.println("[GT911] init failed – touch disabled");
  } else {
    Serial.printf("[GT911] OK. Native %ux%u. swap=%d invX=%d invY=%d\n",
                  touch.xMax(), touch.yMax(), TOUCH_SWAP_XY, TOUCH_INVERT_X, TOUCH_INVERT_Y);
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, indev_read_cb);
  }

  // UI + IO
  build_home_screen();
  setup_pwm();
}

void loop() {
  lv_timer_handler();
  delay(5);
}
