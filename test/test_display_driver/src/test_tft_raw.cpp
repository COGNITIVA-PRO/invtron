// test_tft_raw.cpp
// Display test WITHOUT LovyanGFX: talks to the ST7735 through the Arduino SPI driver.
// Purpose: tell wiring problems apart from LovyanGFX-configuration problems.
//  - Screen cycles RED / GREEN / BLUE / WHITE every second  -> wiring is OK, fix the LovyanGFX config.
//  - Screen stays blank                                      -> check wiring / power / module (CS, DC, RST, SCK, SDA, BL).
// Serial (115200) prints every step with "\n".

#include <Arduino.h>
#include <SPI.h>

#define TFT_LED   4
#define TFT_SCK  16
#define TFT_SDA  17     // MOSI
#define TFT_A0   18     // DC
#define TFT_RST  19
#define TFT_CS   21

// Visible window inside the controller RAM. Typical values:
//  red/black tab : col 0,  row 0
//  green tab     : col 2,  row 1
#define COL_START 0
#define ROW_START 0
#define W 128
#define H 160

SPIClass spi(HSPI);                      // any SPI host works: pins go through the GPIO matrix
SPISettings cfg(8000000, MSBFIRST, SPI_MODE0);

void cmd(uint8_t c) {
  digitalWrite(TFT_A0, LOW);
  digitalWrite(TFT_CS, LOW);
  spi.beginTransaction(cfg); spi.transfer(c); spi.endTransaction();
  digitalWrite(TFT_CS, HIGH);
}

void data(const uint8_t* d, size_t n) {
  digitalWrite(TFT_A0, HIGH);
  digitalWrite(TFT_CS, LOW);
  spi.beginTransaction(cfg);
  for (size_t i = 0; i < n; i++) spi.transfer(d[i]);
  spi.endTransaction();
  digitalWrite(TFT_CS, HIGH);
}

void cmdData(uint8_t c, std::initializer_list<uint8_t> d) {
  cmd(c);
  if (d.size()) data(d.begin(), d.size());
}

void fill(uint16_t rgb565) {
  uint16_t x0 = COL_START, x1 = COL_START + W - 1;
  uint16_t y0 = ROW_START, y1 = ROW_START + H - 1;
  cmdData(0x2A, { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 });   // CASET
  cmdData(0x2B, { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 });   // RASET
  cmd(0x2C);                                                                              // RAMWR
  digitalWrite(TFT_A0, HIGH);
  digitalWrite(TFT_CS, LOW);
  spi.beginTransaction(cfg);
  uint8_t hi = rgb565 >> 8, lo = rgb565 & 0xFF;
  for (uint32_t i = 0; i < (uint32_t)W * H; i++) { spi.transfer(hi); spi.transfer(lo); }
  spi.endTransaction();
  digitalWrite(TFT_CS, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.print("[raw] start\n");

  pinMode(TFT_LED, OUTPUT); digitalWrite(TFT_LED, HIGH);
  pinMode(TFT_A0,  OUTPUT);
  pinMode(TFT_CS,  OUTPUT); digitalWrite(TFT_CS, HIGH);
  pinMode(TFT_RST, OUTPUT);

  spi.begin(TFT_SCK, -1, TFT_SDA, -1);       // SCK, MISO (unused), MOSI, SS (manual)
  Serial.print("[raw] SPI OK\n");

  // Hardware reset
  digitalWrite(TFT_RST, HIGH); delay(20);
  digitalWrite(TFT_RST, LOW);  delay(20);
  digitalWrite(TFT_RST, HIGH); delay(150);
  Serial.print("[raw] reset done\n");

  cmd(0x01); delay(150);                      // SWRESET
  cmd(0x11); delay(255);                      // SLPOUT
  cmdData(0x3A, { 0x05 });                    // COLMOD: 16 bit/pixel
  cmdData(0x36, { 0x00 });                    // MADCTL: portrait, RGB order (try 0x08 if red<->blue)
  // cmd(0x21);                               // INVON: uncomment if the colors look inverted
  cmd(0x29); delay(100);                      // DISPON
  Serial.print("[raw] controller initialised\n");
}

void loop() {
  static const uint16_t colors[] = { 0xF800, 0x07E0, 0x001F, 0xFFFF };   // R, G, B, white
  static const char*    names[]  = { "RED", "GREEN", "BLUE", "WHITE" };
  static int i = 0;
  fill(colors[i]);
  Serial.printf("[raw] %s\n", names[i]);
  i = (i + 1) % 4;
  delay(1000);
}
