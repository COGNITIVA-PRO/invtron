// test_invtron_pins.cpp
// Hardware test: TFT display + encoder (KY-040) + 2 touch buttons + INA3221 (canal 3).
//  - encoder turned   -> "direita" / "esquerda"
//  - button pressed   -> "bt_encoder" / "bt_1" (touch BACK, pin 33) / "bt_2" (touch OK, pin 32)
//  - every INA_PERIOD_MS -> "ch3 x.xxV yyy.ymA"
// Every message is shown on the display (scrolling) and sent to Serial with "\n".

#include <Arduino.h>
#include <Wire.h>

#define LGFX_USE_V1            // must come before LovyanGFX
#include <LovyanGFX.hpp>

// Begin of pin definitions for the ESP32-38 board
// -----------------------------------------------
// ESP32-38 LEFT SIDE:
//                   V5 // +5V from SX1308
//                  CMD // not connected
//                  SD3 // not connected
//                  SD2 // not connected
//                   13  // free to use, not connected
//                  GND // GND from SX1308
//                   12  // strapping pin, free to use, use with care!
//                   14  // free to use, not connected
// Next 3 pins are used by the KY-040 Encoder
#define ENCODER_SW  27  // SW  - blue
#define ENCODER_DT  26  // DT  - yellow
#define ENCODER_CLK 25  // CLK - green
// Next 2 pins are used by the touch buttons
#define BTN_BAK     33  // BACK button blue
#define BTN_1       BTN_BAK  // touch 1 = BACK (prints "bt_1")
#define BTN_OK      32  // OK button blue
#define BTN_2       BTN_OK   // touch 2 = OK   (prints "bt_2")
//                  35  // free to use, not connected
#define CHG_PIN     34  // would read High if charging, Low if not charging
//                  36  // input-only, free to use, not connected
//                  39  // input-only, free to use, not connected
//                  38  // enable, used by the reset button (to GND)
//                 3V3  // 3.3V output to power the INA3221 battery level sensor

// ESP32-38 RIGHT SIDE:
//                 CLK  // not connected
//                 SD0  // not connected
//                 SD1  // not connected
//                  15  // strapping pin, free to use, use with care!
//                  02  // strapping pin, free to use, use with care!
//                  00  // strapping pin, free to use, use with care!
// Pins used by the SPI TFT Display 1.8" 128x160 (ST7735):
#define TFT_LED     04  // BL/LED    blue
#define TFT_SCK     16  // SCL/SCK   green
#define TFT_SDA     17  // SDA       yellow
//                  05   // strapping pin, free to use, use with care!
#define TFT_A0      18  // DC/A0     blue
#define TFT_RST     19  // RESET     green
//                  GND // to INA3221 GND
#define TFT_CS      21  // CS        blue
//                  RSD // not connected
//                  TXD // not connected
// Pins used by the INA3221 battery level sensor (I2C, both pins are bidirectional on the ESP32):
#define INA_SCL     22  // INA_SCL    green  (I2C clock)
#define INA_SDA     23  // INA_SDA    blue   (I2C data)
//                  GND // used by the reset button (to GND EN pin)
// End of pin definitions for the ESP32-38 board
// ---------------------------------------------

#define TFT_MISO     -1

// ---- TFT tuning (change these if the panel shows shifted / wrong-colored / inverted image) ----
#define TFT_SPI_FREQ   8000000    // 8 MHz = the speed used by test_tft_raw.cpp (known to work); raise later
#define TFT_PWM_BL     0          // 0 = backlight is a plain GPIO (as in test_tft_raw.cpp); 1 = LovyanGFX PWM
#define TFT_OFFSET_X   0          // some 1.8" modules need 2 (green tab) or 0 (red tab)
#define TFT_OFFSET_Y   0          // some 1.8" modules need 1 (green tab) or 0 (red tab)
#define TFT_RGB_ORDER  false      // true if red and blue are swapped
#define TFT_INVERT     false      // true if colors are inverted (black <-> white)

// ---- INA3221 ----
#define INA_CHANNEL     3         // 1..3
#define INA_SHUNT_MOHM  100       // shunt resistor in milliohms (most breakouts: 0.1 ohm = 100 mOhm)
#define INA_PERIOD_MS   2000      // time between readings

// Resistive Tension Divider
#define R1 10270.0f // 10k27 Ohms
#define R2 32750.0f // 32k75 Ohms
#define MIN_READS 10   // number of readings to get the average voltage

// Touch modules (TTP223-like) output HIGH when touched. Use LOW/INPUT_PULLUP if yours are active-low.
#define TOUCH_ACTIVE_LEVEL HIGH
#define TOUCH_PIN_MODE     INPUT_PULLDOWN

#define DEBOUNCE_MS   30

// 1 = also print "<botao> solto" when a button is released. If you touch ONCE and never see "solto",
// the touch module is latching (TTP223 in TOGGLE mode: remove jumper A / solder bridge) or it is
// active-LOW (jumper B): then set TOUCH_ACTIVE_LEVEL to LOW and TOUCH_PIN_MODE to INPUT_PULLUP.
#define LOG_RELEASE   1

// ---- Display ----
class LGFX_ESP32_Shield : public lgfx::LGFX_Device {
  lgfx::Panel_ST7735S _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;
public:
  LGFX_ESP32_Shield() {
    auto b = _bus.config();
    b.spi_host = HSPI_HOST;  b.spi_mode = 0;
    b.freq_write = TFT_SPI_FREQ; b.freq_read = 8000000;
    b.spi_3wire = false;     b.use_lock = true;
    b.dma_channel = 0;       // DMA disabled: avoids version-dependent DMA init problems
    b.pin_sclk = TFT_SCK;    b.pin_mosi = TFT_SDA;
    b.pin_miso = TFT_MISO;   b.pin_dc = TFT_A0;
    _bus.config(b);
    _panel.setBus(&_bus);

    auto p = _panel.config();
    p.pin_cs = TFT_CS;  p.pin_rst = TFT_RST;  p.pin_busy = -1;
    p.panel_width = 128; p.panel_height = 160;
    p.offset_x = TFT_OFFSET_X; p.offset_y = TFT_OFFSET_Y;
    p.offset_rotation = 0;
    p.readable = false;  p.bus_shared = false;
    p.invert = TFT_INVERT;
    p.rgb_order = TFT_RGB_ORDER;
    _panel.config(p);

#if TFT_PWM_BL
    auto l = _light.config();
    l.pin_bl = TFT_LED; l.invert = false; l.freq = 12000; l.pwm_channel = 7;
    _light.config(l);
    _panel.setLight(&_light);
#endif

    setPanel(&_panel);
  }
};

LGFX_ESP32_Shield tft;

// ---- Scrolling log ----
#define LINE_H     10
#define MAX_LINES  12            // 128 px / 10 px (landscape: 160 x 128)
#define LINE_CHARS 26

String lines[MAX_LINES];
int    lineCount = 0;
uint32_t msgNo = 0;

// Shows the message on the display (scrolling up when full) and on Serial (with "\n")
void Log(const String& msg) {
  String m = String(++msgNo) + " " + msg;
  Serial.print(m);
  Serial.print("\n");

  if (lineCount == MAX_LINES) {
    for (int i = 1; i < MAX_LINES; i++) lines[i - 1] = lines[i];
    lineCount--;
  }
  lines[lineCount++] = m.substring(0, LINE_CHARS);

  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(lgfx::top_left);
  for (int i = 0; i < lineCount; i++) {
    uint16_t fg = (i == lineCount - 1) ? TFT_YELLOW : TFT_WHITE;
    tft.setTextColor(fg, TFT_BLACK);               // foreground + background
    tft.drawString(lines[i].c_str(), 2, 2 + i * LINE_H);
  }
  tft.endWrite();
}

// ---- Encoder (ISR only records the direction; printing is done in loop) ----
int  encSteps = 0;  
int  lastClk;

void IRAM_ATTR ReadEncoder() {
  int clk = digitalRead(ENCODER_CLK);
  if (clk != lastClk && clk == LOW) {
    if (digitalRead(ENCODER_DT) != clk) encSteps++;
    else                                encSteps--;
  }
  lastClk = clk;
}

// ---- Buttons ----
struct Button {
  uint8_t     pin;
  uint8_t     activeLevel;
  const char* name;
  bool        stable;        // debounced state (true = pressed)
  bool        lastRaw;
  uint32_t    changedAt;
};

Button buttons[] = {
  { ENCODER_SW, LOW,                "bt_encoder", false, false, 0 },
  { BTN_1,      TOUCH_ACTIVE_LEVEL, "bt_1",       false, false, 0 },
  { BTN_2,      TOUCH_ACTIVE_LEVEL, "bt_2",       false, false, 0 },
};

void PollButtons() {
  for (Button& b : buttons) {
    bool raw = (digitalRead(b.pin) == b.activeLevel);
    if (raw != b.lastRaw) { b.lastRaw = raw; b.changedAt = millis(); }
    if (raw != b.stable && millis() - b.changedAt >= DEBOUNCE_MS) {
      b.stable = raw;
      if (b.stable)           Log(b.name);                    // press
#if LOG_RELEASE
      else                    Log(String(b.name) + " solto"); // release (diagnostic)
#endif
    }
  }
}

// ---- INA3221 (plain I2C, no library) ----
#define INA_REG_CONFIG   0x00
#define INA_REG_MANUF_ID 0xFE            // should read 0x5449
#define INA_CONFIG_VALUE 0x7127          // default: all channels on, continuous mode

uint8_t inaAddr = 0;                     // 0 = not found

bool InaRead16(uint8_t reg, uint16_t& val) {
  Wire.beginTransmission(inaAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)inaAddr, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  val = ((uint16_t)hi << 8) | lo;
  return true;
}

bool InaWrite16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(inaAddr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

bool InaBegin() {
  Wire.begin(INA_SDA, INA_SCL);          // order: SDA, SCL
  Wire.setClock(100000);
  for (uint8_t addr = 0x40; addr <= 0x43; addr++) {   // A0 pin: GND/VS/SDA/SCL
    inaAddr = addr;
    uint16_t id;
    if (InaRead16(INA_REG_MANUF_ID, id) && id == 0x5449) {
      InaWrite16(INA_REG_CONFIG, INA_CONFIG_VALUE);
      return true;
    }
  }
  inaAddr = 0;
  return false;
}

// Reads bus voltage (V) and current (mA) of INA_CHANNEL
bool InaReadChannel(int CH, float& volts, float& mA) {
  if (!inaAddr) return false;
  // uint8_t regShunt = 0x01 + 2 * (INA_CHANNEL - 1);
  // uint8_t regBus   = 0x02 + 2 * (INA_CHANNEL - 1);
  uint8_t regShunt = 0x01 + 2 * (CH - 1);
  uint8_t regBus   = 0x02 + 2 * (CH - 1);
  uint16_t rawShunt, rawBus;
  if (!InaRead16(regShunt, rawShunt) || !InaRead16(regBus, rawBus)) return false;

  int16_t shunt = ((int16_t)rawShunt) >> 3;   // 12 bits, signed, LSB = 40 uV
  int16_t bus   = ((int16_t)rawBus)   >> 3;   // 12 bits, LSB = 8 mV
  volts = bus * 0.008f;
  mA    = (shunt * 40.0f) / INA_SHUNT_MOHM;   // uV / mOhm = mA
  return true;
}

// Reads the voltage on the CHG_PIN (ADC) and calculates the real voltage of the circuit using a resistive divider.
float readVoltage() {
  uint32_t voltsSum = 0;

  // 1. Averages the voltage readings on the CHG_PIN (ADC)
  for (int i = 0; i < MIN_READS; i++) {
    voltsSum += analogReadMilliVolts(CHG_PIN);
    delay(2);
  }

  // 2. Averages the readings and converts to volts (from millivolts)
  float readVoltage = (float)voltsSum / MIN_READS / 1000.0;

  // 3. Adjusts the real voltage of the circuit using the resistive divider formula: Vreal = Vpino * ((R1 + R2) / R2)
  float adjVolts = readVoltage * ((R1 + R2) / R2);

  // 4. Returns the adjusted voltage value (V) to the caller
  return (adjVolts);

}

void setup() {
  Serial.begin(115200);
  delay(1000);                       // let the serial monitor attach before the first message
  Serial.print("[boot] Serial OK\n");

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);
  pinMode(BTN_1, TOUCH_PIN_MODE);
  pinMode(BTN_2, TOUCH_PIN_MODE);
  pinMode(CHG_PIN, INPUT);  
#if !TFT_PWM_BL
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);       // backlight always on
#endif
  Serial.print("[boot] pins OK\n");

  Serial.print("[boot] tft.init()...\n");
  tft.init();
  tft.setBrightness(255);
  tft.setColorDepth(16);
  tft.setRotation(1);
  Serial.print("[boot] tft.init() OK\n");

  // Display self-test: if the colors appear but text does not, the problem is only in the text code;
  // if nothing changes, the SPI/panel setup is wrong (check wiring, offsets, TFT_RGB_ORDER/INVERT).
  tft.fillScreen(TFT_RED);   delay(300);
  tft.fillScreen(TFT_GREEN); delay(300);
  tft.fillScreen(TFT_BLUE);  delay(300);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("TFT OK", 2, 2);
  Serial.print("[boot] display OK\n");
  delay(500);

  lastClk = digitalRead(ENCODER_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), ReadEncoder, CHANGE);
  Serial.print("[boot] encoder ISR OK\n");

  Log("teste de pinos pronto");

  if (InaBegin()) {
    char buf[32];
    snprintf(buf, sizeof(buf), "INA3221 ok (0x%02X)", inaAddr);
    Log(buf);
  } else {
    Log("INA3221 not present?");
  }
  analogSetPinAttenuation(CHG_PIN, ADC_11db); // 0..3.3V range (ADC1, safe with Wi-Fi)

  Serial.printf("\n- Tension sensor setup on pin %d", CHG_PIN);
}

void loop() {
  int steps = encSteps;
  float v, i;
  char buf[32];
  static uint32_t hb = 0;

  // Heartbeat on Serial only (not on the display) to prove loop() is running
  if (millis() - hb >= 5000) { hb = millis(); Serial.print("[alive]\n"); }

  // Encoder: print one message per detected click
  noInterrupts();
  encSteps = 0;
  interrupts();
  while (steps > 0) { Log("direita");  steps--; }
  while (steps < 0) { Log("esquerda"); steps++; }

  PollButtons();

  // INA3221: periodic reading of channel INA_CHANNEL
  static uint32_t inaT = 0;
  if (inaAddr && millis() - inaT >= INA_PERIOD_MS) {
    inaT = millis();
    for (int ch = 1; ch <= 3; ch++) {
      if (InaReadChannel(ch, v, i)) snprintf(buf, sizeof(buf), "ch%d %.2fV %.1fmA", ch, v, i);
      else                      snprintf(buf, sizeof(buf), "ch%d erro I2C", ch);
      Log(buf);                 // display + Serial (with "\n")
    }
    // if (InaReadChannel(v, i)) snprintf(buf, sizeof(buf), "ch%d %.2fV %.1fmA", INA_CHANNEL, v, i);
    // else                      snprintf(buf, sizeof(buf), "ch%d erro I2C", INA_CHANNEL);
    // Log(buf);                 // display + Serial (with "\n")
    if  (readVoltage() > 2.0) {
       Log("charging");
    }
    else {
      Log("not charging");
    }
  }
}
