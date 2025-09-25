#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "pins.h"

class LGFX_ST7796 : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel;
  lgfx::Bus_SPI _bus;
public:
  LGFX_ST7796() {
    auto b = _bus.config();
    b.spi_host = VSPI_HOST;
    b.spi_mode = 0;
    b.freq_write = 40000000;
    b.freq_read  = 16000000;
    b.spi_3wire = false;
    b.use_lock = true;
    b.dma_channel = 1;
    b.pin_sclk = PIN_TFT_SCLK;
    b.pin_mosi = PIN_TFT_MOSI;
    b.pin_miso = PIN_TFT_MISO;
    b.pin_dc   = PIN_TFT_DC;
    _bus.config(b); _panel.setBus(&_bus);

    auto p = _panel.config();
    p.pin_cs   = PIN_TFT_CS;
    p.pin_rst  = PIN_TFT_RST;
    p.pin_busy = -1;
    p.panel_width  = 320;
    p.panel_height = 480;
    p.readable = true;
    p.invert   = false;
    p.rgb_order= false;
    p.dlen_16bit = false;
    p.bus_shared = true;
    _panel.config(p);
    setPanel(&_panel);
  }
};