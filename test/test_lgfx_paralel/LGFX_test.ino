#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX_ESP32_Shield : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9486 _panel_instance;  // troque aqui se necessário, veja abaixo
  //lgfx::Panel_ILI9488 _panel_instance;
  //lgfx::Panel_ST7796  _panel_instance;
  //lgfx::Panel_HX8357D _panel_instance;
  
  lgfx::Bus_Parallel8 _bus_instance;

public:
  LGFX_ESP32_Shield() {
    auto cfg = _bus_instance.config();
    cfg.freq_write = 8000000;  // reduzido para teste
    cfg.pin_d0 = 04;  
    cfg.pin_d1 = 05;  
    cfg.pin_d2 = 13; 
    cfg.pin_d3 = 14;
    cfg.pin_d4 = 16; 
    cfg.pin_d5 = 17; 
    cfg.pin_d6 = 18; 
    cfg.pin_d7 = 19;
    cfg.pin_rd = 21; 
    cfg.pin_wr = 22; 
    cfg.pin_rs = 23;

    _bus_instance.config(cfg);
    _panel_instance.setBus(&_bus_instance);

    auto pcfg = _panel_instance.config();
    pcfg.panel_width  = 320;
    pcfg.panel_height = 480;
    pcfg.pin_cs  = 25;
    pcfg.pin_rst = 26;
    _panel_instance.config(pcfg);

    setPanel(&_panel_instance);
  }
};

LGFX_ESP32_Shield tft;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_RED);
  delay(1000);
  tft.fillScreen(TFT_GREEN);
  delay(1000);
  tft.fillScreen(TFT_BLUE);
  Serial.println("Fim do teste");
}

void loop() {}