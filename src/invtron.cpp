#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <algorithm>
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

// CRITICAL FIREBASE MACROS MUST COME BEFORE THE INCLUDE LINE
#define ENABLE_USER_AUTH  // enables FirebaseClient user authentication
#define ENABLE_DATABASE   // enables RealtimeDatabase feature

#include <FirebaseClient.h>
#include <ArduinoJson.h>
#include <vector>

#define LGFX_USE_V1       // enables version 1 (config-based) features (must come before LovyanGFX)
#include <LovyanGFX.hpp>
#include <SPI.h>
#include "secret/credentials.h"

// Network/Wifi credentials (from secret/credentials.h)
#define WIFI_SSID     WIFI_SSID1
#define WIFI_PASSWORD WIFI_PASSWORD1

// Firebase credentials (from secret/credentials.h)
#define API_KEY       MY_API_KEY
#define DATABASE_URL  MY_DATABASE_URL
#define USER_EMAIL    MY_USER_EMAIL
#define USER_PASS     MY_USER_PASS

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
#define CHG_PIN     34  // input-only, free to use, not connected
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

// This TFT display does not have the MISO pin
#define TFT_MISO    -1

// 0: normal portrait; 1: 90 deg clockwise (landscape); 2: inverted portrait; 3: inverted landscape
#define TFT_Orientation 3

// ---- TFT tuning (change these if the panel shows shifted / wrong-colored / inverted image) ----
#define TFT_SPI_FREQ   8000000    // 8 MHz = the speed used by test_tft_raw.cpp (known to work); raise later
#define TFT_PWM_BL     0          // 0 = backlight is a plain GPIO (as in test_tft_raw.cpp); 1 = LovyanGFX PWM
#define TFT_OFFSET_X   0          // some 1.8" modules need 2 (green tab) or 0 (red tab)
#define TFT_OFFSET_Y   0          // some 1.8" modules need 1 (green tab) or 0 (red tab)
#define TFT_RGB_ORDER  false      // true if red and blue are swapped
#define TFT_INVERT     false      // true if colors are inverted (black <-> white)

// ---- INA3221 ----
#define INA_CHANNEL      3        // 1..3
#define INA_SHUNT_MOHM   100      // shunt resistor in milliohms (most breakouts: 0.1 ohm = 100 mOhm)
#define INA_PERIOD_MS    1000     // time between readings
#define INA_REG_CONFIG   0x00     // default register value
#define INA_REG_MANUF_ID 0xFE     // should read 0x5449
#define INA_CONFIG_VALUE 0x7127   // default: all channels on, continuous mode

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

//--------------------------------------------------------------------------------------------------------------------------------------
// Application-specific constants
//--------------------------------------------------------------------------------------------------------------------------------------
#define DELAY_TIME        500    // generic time-out/delay unit (in ms)
#define AUTH_TIMEOUT_MS 20000    // max time waiting for the Firebase sign-in
#define BUFFER_LIMIT    40000    // max size of an item picture (bytes)

// ---- Lazy-load parameters ----
// PAGE_SIZE (N): how many records are loaded from the cloud at a time. Turning the encoder past the
// last displayed record loads the next N; turning it before the first one loads the previous N.
#define PAGE_SIZE          8
// Safety cap: the biggest number of records a single request may ask for (protects the ESP32 heap).
// Filtered lists (Subcategory / Item) are read with an offset window, so the cost of page P grows
// with P: P * PAGE_SIZE must stay <= MAX_FETCH_ITEMS.
#define MAX_FETCH_ITEMS  120

// ---- Transport security ----
// 1 = validate the server certificate with the CA bundle that ships with the ESP32 core (recommended)
// 0 = insecure, certificate NOT validated (debug only, never ship it!)
#define TLS_VERIFY         1
// Item pictures are public data; plain http:// picture URLs are accepted when this is 1.
#define IMG_ALLOW_HTTP     1

#if TLS_VERIFY
// CA bundle embedded by the ESP32 Arduino core
extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
extern const uint8_t rootca_crt_bundle_end[]   asm("_binary_x509_crt_bundle_end");
#endif

//--------------------------------------------------------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t inaAddr = 0; // 0 = INA3221 not present
static uint32_t inaT = 0;

// Encoder state variables
bool EncoderTurned = false;
int  TurnDirection = 0;
int  LastCLKStatus = 0;

// Touch Buttons state variables
bool ButtonPressed = false;
bool OkPressed = false;
bool BackPressed = false;

// Battery icon picture cache (so changing the quantity does not download the picture again)
uint8_t* IconBuf = nullptr;
size_t   IconLen = 0;
String   IconUrl = "";

//--------------------------------------------------------------------------------------------------------------------------------------
// Menu model
//--------------------------------------------------------------------------------------------------------------------------------------
// Database layout (see rules.json):  /Class/{id}, /Category/{id}, /Subcategory/{id} (Category_ID),
// /Item/{id} (Class_ID, Subcategory_ID, ...), /Inventory/{id} (Item_ID, Quantity)
enum MenuStatus { MENU_CLASS, MENU_CATEGORY, MENU_SUBCATEGORY, MENU_ITEM, MENU_ADJ_QTY };
MenuStatus CurrentMenuStatus = MENU_CLASS;

struct MenuEntry {
  String key;    // record id
  String label;  // text shown on the display
  String cls;    // Class_ID (items only)
};

struct LevelState { bool fromTail; int page; int sel; };
LevelState Saved[4] = {};   // list position of each level, restored by the Back button

struct ListSpec {
  const char* path = "";
  const char* nameField = "";
  const char* filterField = nullptr;  // child used for the server-side filter (needs .indexOn)
  String      filterValue;
  bool        isItem = false;
};

// Current selection
String idClass = "", idCat = "", idSub = "", idItem = "";

// Item being edited
String CurrentItemName = "", CurrentItemPartNo = "", CurrentItemPic = "", CurrentItemUnit = "";
String InventoryId = "";          // key of the /Inventory record ("" = none yet)
int    InventoryQty = 0;
int    InventoryQtyOriginal = 0;

// Picture cache (so changing the quantity does not download the picture again)
uint8_t* PicBuf = nullptr;
size_t   PicLen = 0;

// Entries currently displayed (one page) and the selected one
std::vector<MenuEntry> Menu;
int SelectedIndex = 0;

// Position of the page on screen: PageFromTail=false -> PageNo-th page counted from the first record;
// PageFromTail=true -> PageNo-th page counted from the last record (0 = last page).
bool PageFromTail = false;
int  PageNo = 0;

String LastError = "";


//--------------------------------------------------------------------------------------------------------------------------------------
// Class and interrupt definitions
//--------------------------------------------------------------------------------------------------------------------------------------
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

// Write-only Stream that collects an HTTP body in RAM (HTTPClient::writeToStream also decodes
// "Transfer-Encoding: chunked"). Refuses data beyond its capacity, which aborts the download.
class MemSink : public Stream {
public:
  uint8_t* buf;
  size_t   cap;
  size_t   len = 0;
  MemSink(uint8_t* b, size_t c) : buf(b), cap(c) {}
  size_t write(uint8_t b) override { if (len >= cap) return 0; buf[len++] = b; return 1; }
  size_t write(const uint8_t* p, size_t n) override {
    if (len + n > cap) return 0;
    memcpy(buf + len, p, n);
    len += n;
    return n;
  }
  int  available() override { return 0; }
  int  read() override { return -1; }
  int  peek() override { return -1; }
  void flush() override {}
};


// Interrupt service routine to read the encoder state
void IRAM_ATTR ReadEncoder() {
  int CLKStatus = digitalRead(ENCODER_CLK);
  if (CLKStatus != LastCLKStatus && CLKStatus == LOW) {
    if (digitalRead(ENCODER_DT) != CLKStatus) TurnDirection = 1;
    else TurnDirection = -1;
    EncoderTurned = true;
  }
  LastCLKStatus = CLKStatus;
}

//--------------------------------------------------------------------------------------------------------------------------------------
// Object instances
//--------------------------------------------------------------------------------------------------------------------------------------

// Instantiate the display class (tft) using the LGFX_ESP32_Shield class
LGFX_ESP32_Shield tft;

// Instantiate Firebase user authentication (e-mail/password; the ID token is refreshed by database_app.loop())
UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASS);

// Initialize Firebase components
FirebaseApp database_app;

// ssl_client is dedicated to Firebase. Pictures use their own client (img_client), otherwise every
// picture download would tear down the Firebase TLS session.
WiFiClientSecure ssl_client;
WiFiClientSecure img_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);

// Instance of RealtimeDatabase
RealtimeDatabase rtdb;


//--------------------------------------------------------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------------------------------------------------------
void ConnectToWiFi(const char* ssid, const char* password);
bool EnsureWiFi();
void ConfigureTLS(WiFiClientSecure& c);
bool SyncClock();
void Halt(const String& msg);
void DrawBusy(const char* msg = "Loading...");
void ShowError(const String& msg);
bool FetchPage(const ListSpec& sp, bool fromTail, int page, std::vector<MenuEntry>& out, int& rawCount);
bool GoPage(int dir);
bool LoadAt(bool tail, int page);
bool LoadHead();
void Enter(MenuStatus next);
void GoBack();
void RestoreLevel();
void LoadItemDetails(const String& id);
bool FetchPicture(const String& url, bool isIcon = false);
void FreePicture(uint8_t* aBuffer);
void SaveQty();
void OnSelect();
void HandleEncoder(int dir);
void UpdateDisplay();
void DrawList();
void DrawQtyScreen();
void DrawQty();
bool InaRead16(uint8_t reg, uint16_t& val);
bool InaWrite16(uint8_t reg, uint16_t val);
bool InaBegin();
bool InaReadChannel(float& volts, float& mA);
bool charging();
void showBatteryStatus(float volts);
float getInaVolts();

//--------------------------------------------------------------------------------------------------------------------------------------
// SETUP
//--------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  char buf[32];
  unsigned long Auth_Start = millis();
  unsigned long LastDot = 0;

  Serial.begin(115200);

  // Initialize the encoder pins and attach the interrupt
  Serial.println("Initializing Encoder");
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);
  pinMode(BTN_OK,      TOUCH_PIN_MODE);
  pinMode(BTN_BAK,     TOUCH_PIN_MODE);
  pinMode(TFT_LED,     OUTPUT);
  pinMode(CHG_PIN,     INPUT); 
  digitalWrite(TFT_LED, HIGH);       // backlight always on

  LastCLKStatus = digitalRead(ENCODER_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), ReadEncoder, CHANGE);

  // Initialize the TFT display
  Serial.println("Initializing TFT Display");
  tft.init();

  tft.setRotation(TFT_Orientation);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextWrap(false, false);

  // Wi-Fi: two sets of credentials (e.g. home and office) are tried in order
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  // Check if there's a INA3221 sensor (I2C)
  if (InaBegin()) {
    tft.printf("\nINA3221 found at 0x%02X", inaAddr);
    tft.printf("\n%s",buf);
  } else {
    tft.printf("\nINA3221 module not present!");
  }

  ConnectToWiFi(WIFI_SSID1, WIFI_PASSWORD1);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\n > Warning! Wi-Fi %s is not responding, trying %s instead.\n", WIFI_SSID1, WIFI_SSID2);
    tft.printf("\n > Wi-Fi %s failed, trying %s\n", WIFI_SSID1, WIFI_SSID2);
    ConnectToWiFi(WIFI_SSID2, WIFI_PASSWORD2);
    if (WiFi.status() != WL_CONNECTED) Halt("ERROR 01: Couldn't connect to Wi-Fi!");
  }
  tft.printf("\nConnected to Wi-Fi %s\n", WiFi.SSID().c_str());
  Serial.printf("Connected to Wi-Fi %s\n", WiFi.SSID().c_str());

  // Certificate validation needs a valid clock (notBefore/notAfter)
#if TLS_VERIFY
  tft.print("Syncing clock (NTP)...\n");
  Serial.print("Syncing clock (NTP)...\n");
  if (!SyncClock()) Halt("ERROR 04: Clock not synchronized (NTP). TLS cannot be validated.");
#endif

  // TLS for Firebase and for pictures
  ConfigureTLS(ssl_client);
  ConfigureTLS(img_client);

  // Initialize FirebaseApp with the secure client and user authentication
  initializeApp(aClient, database_app, getAuth(user_auth));

  // Configure the RealtimeDatabase instance with the database URL
  database_app.getApp<RealtimeDatabase>(rtdb);
  rtdb.url(DATABASE_URL);

  // tft.fillScreen(TFT_BLACK);
  // tft.setCursor(2, 2);
  Serial.print("Firebase auth in progress...\n");
  tft.print("Firebase auth in progress...\n");

  while (!database_app.ready()) {
    database_app.loop();
    if (millis() - Auth_Start > AUTH_TIMEOUT_MS) {
      // The old code did "break" here and went on unauthenticated. Now the device stops.
      Halt("ERROR 02: Firebase sign-in failed/timed out. Check e-mail/password, verified e-mail and the 'approved' flag.");
    }
    if (millis() - LastDot > DELAY_TIME) {
      LastDot = millis();
      Serial.print(".");
      tft.print(".");
    }
    delay(10);
  }
  Serial.println("\nFirebase signed in.");
  tft.println("\nFirebase signed in.");

  // Load the first page of the first menu
  Serial.print("\nLoading class data...\n");
  tft.print("\nLoading class data...\n");
  CurrentMenuStatus = MENU_CLASS;
  LoadHead();
  UpdateDisplay();
}



//--------------------------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  bool backNow; // back btn pressed now
  bool okNow;   // OK btn pressed now
  int  dir;     // encoder turn direction (> 0 = CW, < 0 = CCW)
  
  // Keeps the connection and the ID token alive (token refresh)
  database_app.loop();

  if (EncoderTurned) {
    dir = TurnDirection;
    EncoderTurned = false;
    HandleEncoder(dir);
  }

  // Encoder push button and OK touch button: select / confirm
  okNow = (digitalRead(ENCODER_SW) == LOW) || (digitalRead(BTN_OK) == TOUCH_ACTIVE_LEVEL);
  if (okNow && !ButtonPressed) {
    ButtonPressed = true;
    delay(DELAY_TIME / 10); // debounce delay
    if ((digitalRead(ENCODER_SW) == LOW) || (digitalRead(BTN_OK) == TOUCH_ACTIVE_LEVEL)) OnSelect();
  }
  if (!okNow) ButtonPressed = false;

  // Back touch button: go up one level
  backNow = (digitalRead(BTN_BAK) == TOUCH_ACTIVE_LEVEL);
  if (backNow && !BackPressed) {
    BackPressed = true;
    delay(DELAY_TIME / 10);
    if (digitalRead(BTN_BAK) == TOUCH_ACTIVE_LEVEL) GoBack();
  }
  if (!backNow) BackPressed = false;

  // Periodic reading of channel INA3221 INA_CHANNEL
  if (inaAddr && millis() - inaT >= INA_PERIOD_MS) {
    showBatteryStatus(getInaVolts());
    inaT = millis();
  }
}



//--------------------------------------------------------------------------------------------------------------------------------------
// INA3221 (plain I2C, no library)
//--------------------------------------------------------------------------------------------------------------------------------------
float getInaVolts() {
  float v, i;
  char buf[32];
  if (InaReadChannel(v, i)) {
    snprintf(buf, sizeof(buf), "ch%d %.2fV %.1fmA", INA_CHANNEL, v, i);
  }
  else {
    snprintf(buf, sizeof(buf), "ch%d I2C error!", INA_CHANNEL);
    v = -1.0f;
  }
  //return(String(buf));
  return (v);
}

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
bool InaReadChannel(float& volts, float& mA) {
  if (!inaAddr) return false;
  uint8_t regShunt = 0x01 + 2 * (INA_CHANNEL - 1);
  uint8_t regBus   = 0x02 + 2 * (INA_CHANNEL - 1);
  uint16_t rawShunt, rawBus;
  if (!InaRead16(regShunt, rawShunt) || !InaRead16(regBus, rawBus)) return false;

  int16_t shunt = ((int16_t)rawShunt) >> 3;   // 12 bits, signed, LSB = 40 uV
  int16_t bus   = ((int16_t)rawBus)   >> 3;   // 12 bits, LSB = 8 mV
  volts = bus * 0.008f;
  mA    = (shunt * 40.0f) / INA_SHUNT_MOHM;   // uV / mOhm = mA
  return true;
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Check for the presence of a charger on the tension divider sensor (CHG_PIN).
//--------------------------------------------------------------------------------------------------------------------------------------
bool charging() {
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

  // 4. Returns the result to the caller
  return ((adjVolts > 2.0f));
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Network / security helpers
//--------------------------------------------------------------------------------------------------------------------------------------
void ConnectToWiFi(const char* _WIFI_SSID, const char* _WIFI_PASSWORD) {
  Serial.printf("Connecting to Wi-Fi %s\n", _WIFI_SSID);
  tft.printf("Connecting to Wi-Fi %s\n", _WIFI_SSID);

  WiFi.begin(_WIFI_SSID, _WIFI_PASSWORD);
  unsigned long Auth_Start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - Auth_Start > (DELAY_TIME * 40)) break;
    Serial.print(".");
    tft.print(".");
    delay(DELAY_TIME);
  }
}

// Re-connects when the Wi-Fi link was lost
bool EnsureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.reconnect();
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
  return WiFi.status() == WL_CONNECTED;
}

// Validated TLS (CA bundle). The previous setInsecure() accepted ANY certificate (man-in-the-middle!).
void ConfigureTLS(WiFiClientSecure& c) {
#if TLS_VERIFY
  #if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0))
    c.setCACertBundle(rootca_crt_bundle_start, rootca_crt_bundle_end - rootca_crt_bundle_start);
  #else
    c.setCACertBundle(rootca_crt_bundle_start);
  #endif
#else
  c.setInsecure();
#endif
}

bool SyncClock() {
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  unsigned long t0 = millis();
  while (time(nullptr) < 1700000000 && millis() - t0 < 20000) {
    delay(250);
    Serial.print(".");
  }
  return time(nullptr) >= 1700000000;
}

void Halt(const String& msg) {
  Serial.println("\n" + msg);
  Serial.println("Device halted!");
  Serial.flush();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_RED);
  tft.setTextWrap(true, true);
  tft.print(msg + "\n\nDevice halted!");
  delay(DELAY_TIME * 10);
  esp_deep_sleep_start();
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Small display helpers
//--------------------------------------------------------------------------------------------------------------------------------------
// Truncates a string to a maximum length, adding "..." if it exceeds that length
String truncar(const String& s, int maxLen) {
  if ((int)s.length() <= maxLen) return s;
  return s.substring(0, maxLen - 3) + "...";
}

void DrawBusy(const char* msg) {
  tft.fillRect(0, 116, 160, 12, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(2, 118);
  tft.print(msg);
}

void ShowError(const String& msg) {
  Serial.println("ERROR: " + msg);
  tft.fillRect(0, 104, 160, 24, TFT_BLACK);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(1);
  tft.setCursor(2, 106);
  tft.print(truncar(msg, 26));
  delay(DELAY_TIME * 4);
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Lazy-loaded pages
//--------------------------------------------------------------------------------------------------------------------------------------
// Which list belongs to each menu level
ListSpec SpecFor(MenuStatus s) {
  ListSpec sp;
  switch (s) {
    case MENU_CLASS:
      sp.path = "/Class";       sp.nameField = "Class_name";
      break;
    case MENU_CATEGORY:
      sp.path = "/Category";    sp.nameField = "Category_name";
      break;
    case MENU_SUBCATEGORY:
      sp.path = "/Subcategory"; sp.nameField = "Subcategory_name";
      sp.filterField = "Category_ID"; sp.filterValue = idCat;
      break;
    default: // MENU_ITEM
      sp.path = "/Item";        sp.nameField = "Item_name";
      sp.filterField = "Subcategory_ID"; sp.filterValue = idSub;
      sp.isItem = true;
      break;
  }
  return sp;
}

// Firebase key order: keys that look like 32-bit integers first (numerically), then the other keys
// in lexicographic order. The server JSON object is not guaranteed to be ordered, so we sort.
static bool IntKey(const String& s, long& v) {
  size_t n = s.length();
  if (n == 0 || n > 11) return false;
  size_t i = 0;
  if (s[0] == '-') { if (n == 1) return false; i = 1; }
  if (s[i] == '0' && n > i + 1) return false;
  for (size_t k = i; k < n; k++) if (!isdigit((unsigned char)s[k])) return false;
  long long x = atoll(s.c_str());
  if (x < -2147483648LL || x > 2147483647LL) return false;
  v = (long)x;
  return true;
}

static bool KeyLess(const MenuEntry& a, const MenuEntry& b) {
  long x = 0, y = 0;
  bool ia = IntKey(a.key, x), ib = IntKey(b.key, y);
  if (ia && ib) return x < y;
  if (ia != ib) return ia;
  return strcmp(a.key.c_str(), b.key.c_str()) < 0;
}

// Reads ONE page from the cloud (never the whole collection).
//  fromTail=false: page 'page' counted from the first record; fromTail=true: counted from the last record.
//  The query asks the server for (page+1)*PAGE_SIZE records and keeps only the wanted slice, so a request
//  never returns more than MAX_FETCH_ITEMS records. Only the needed fields are kept while parsing.
//  rawCount = records found in the slice (before the Class filter); out = records to show.
bool FetchPage(const ListSpec& sp, bool fromTail, int page, std::vector<MenuEntry>& out, int& rawCount) {
  out.clear();
  rawCount = 0;

  int need = (page + 1) * PAGE_SIZE;
  if (need > MAX_FETCH_ITEMS) return true;   // beyond the safety window: behaves like "no more records"

  if (!EnsureWiFi()) { LastError = "Wi-Fi lost"; return false; }

  DatabaseOptions opt;
  opt.filter.orderBy(sp.filterField ? sp.filterField : "$key");
  if (sp.filterField) opt.filter.equalTo(sp.filterValue);
  if (fromTail) opt.filter.limitToLast(need);
  else          opt.filter.limitToFirst(need);

  // Synchronous read: returns when the answer has arrived (the async variant returned before the
  // data was there).
  String payload = rtdb.get<String>(aClient, sp.path, opt);
  if (aClient.lastError().code() != 0) {
    LastError = aClient.lastError().message();
    return false;
  }
  if (payload.length() < 2 || payload == "null") return true;   // no records

  JsonDocument filter;
  filter["*"][sp.nameField] = true;
  if (sp.isItem) {
    filter["*"]["Part No"]  = true;
    filter["*"]["Class_ID"] = true;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
  payload = "";   // release the big String as soon as possible
  if (err) { LastError = String("JSON: ") + err.c_str(); return false; }

  JsonObject obj = doc.as<JsonObject>();
  std::vector<MenuEntry> all;
  all.reserve(obj.size());
  for (JsonPair kv : obj) {
    MenuEntry e;
    e.key = kv.key().c_str();
    JsonObject rec = kv.value().as<JsonObject>();
    String name = String((const char*)(rec[sp.nameField] | e.key.c_str()));
    if (sp.isItem) {
      const char* pn = rec["Part No"] | "";
      e.cls = String((const char*)(rec["Class_ID"] | ""));
      e.label = (pn[0] ? String(pn) + " " : String("")) + name;
    } else {
      e.label = name;
    }
    all.push_back(e);
  }
  std::sort(all.begin(), all.end(), KeyLess);

  int c = (int)all.size();
  int start, end;
  if (!fromTail) { start = page * PAGE_SIZE;  end = std::min(c, need); }
  else           { end = c - page * PAGE_SIZE; start = std::max(0, c - need); }
  if (end <= start) return true;

  rawCount = end - start;
  for (int i = start; i < end; i++) {
    // Class selected in the first menu filters the item list
    if (sp.isItem && idClass.length() && all[i].cls != idClass) continue;
    out.push_back(all[i]);
  }
  return true;
}

// Next/previous target page. Past the last page it wraps to the first one, and vice versa.
static void NextTarget(int dir, bool& tail, int& page) {
  if (dir > 0) {
    if (!tail) page++;
    else if (page > 0) page--;
    else { tail = false; page = 0; }      // wrap: first page
  } else {
    if (tail) page++;
    else if (page > 0) page--;
    else { tail = true; page = 0; }       // wrap: last page
  }
}

// dir > 0: lazy-load the page after the last displayed record; dir < 0: the page before the first one.
// The display is only changed when the new page was loaded successfully.
bool GoPage(int dir) {
  ListSpec sp = SpecFor(CurrentMenuStatus);
  bool tail = PageFromTail;
  int  page = PageNo;
  bool wrappedOnce = false;

  for (int attempt = 0; attempt < 16; attempt++) {
    NextTarget(dir, tail, page);

    std::vector<MenuEntry> out;
    int raw = 0;
    DrawBusy();
    if (!FetchPage(sp, tail, page, out, raw)) { ShowError(LastError); return false; }

    if (raw == 0) {                       // ran past the end of the list
      if (wrappedOnce) break;             // the list is empty
      wrappedOnce = true;
      tail = (dir < 0);                   // wrap around and try again
      page = -1;
      continue;
    }
    if (out.empty()) continue;            // everything filtered out by the Class filter: keep walking

    Menu = out;
    PageFromTail = tail;
    PageNo = page;
    SelectedIndex = (dir > 0) ? 0 : (int)Menu.size() - 1;
    return true;
  }
  return false;
}

// Loads exactly the given page (used to restore a position)
bool LoadAt(bool tail, int page) {
  ListSpec sp = SpecFor(CurrentMenuStatus);
  std::vector<MenuEntry> out;
  int raw = 0;
  DrawBusy();
  if (!FetchPage(sp, tail, page, out, raw)) { ShowError(LastError); return false; }
  if (out.empty()) return false;
  Menu = out;
  PageFromTail = tail;
  PageNo = page;
  SelectedIndex = 0;
  return true;
}

// Loads the first page of the current menu level
bool LoadHead() {
  Menu.clear();
  SelectedIndex = 0;
  PageFromTail = false;
  PageNo = -1;          // GoPage(+1) moves to page 0
  return GoPage(+1);
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Navigation
//--------------------------------------------------------------------------------------------------------------------------------------
void Enter(MenuStatus next) {
  Saved[CurrentMenuStatus] = { PageFromTail, PageNo, SelectedIndex };
  CurrentMenuStatus = next;
  LoadHead();
  UpdateDisplay();
}

void RestoreLevel() {
  LevelState s = Saved[CurrentMenuStatus];
  if (!LoadAt(s.fromTail, s.page)) LoadHead();
  if (!Menu.empty()) SelectedIndex = constrain(s.sel, 0, (int)Menu.size() - 1);
  UpdateDisplay();
}

void GoBack() {
  if (CurrentMenuStatus == MENU_CLASS) return;
  if (CurrentMenuStatus == MENU_ADJ_QTY) {
    FreePicture(PicBuf);
    CurrentMenuStatus = MENU_ITEM;
  } else {
    CurrentMenuStatus = (MenuStatus)((int)CurrentMenuStatus - 1);
  }
  RestoreLevel();
}

void HandleEncoder(int dir) {
  if (CurrentMenuStatus == MENU_ADJ_QTY) {
    if (dir > 0) InventoryQty++;
    else if (InventoryQty > 0) InventoryQty--;
    DrawQty();            // only the number is redrawn (no network access)
    return;
  }

  if (Menu.empty()) return;

  if (dir > 0) {
    if (SelectedIndex < (int)Menu.size() - 1) SelectedIndex++;
    else { GoPage(+1); EncoderTurned = false; }   // past the last displayed record -> next N
  } else {
    if (SelectedIndex > 0) SelectedIndex--;
    else { GoPage(-1); EncoderTurned = false; }   // before the first displayed record -> previous N
  }
  DrawList();
}

// Encoder push / OK button
void OnSelect() {
  switch (CurrentMenuStatus) {
    case MENU_CLASS:
      if (Menu.empty()) return;
      idClass = Menu[SelectedIndex].key;
      Enter(MENU_CATEGORY);
      break;

    case MENU_CATEGORY:
      if (Menu.empty()) return;
      idCat = Menu[SelectedIndex].key;
      Enter(MENU_SUBCATEGORY);
      break;

    case MENU_SUBCATEGORY:
      if (Menu.empty()) return;
      idSub = Menu[SelectedIndex].key;
      Enter(MENU_ITEM);
      break;

    case MENU_ITEM:
      if (Menu.empty()) return;
      Saved[MENU_ITEM] = { PageFromTail, PageNo, SelectedIndex };
      LoadItemDetails(Menu[SelectedIndex].key);
      break;

    case MENU_ADJ_QTY:
      SaveQty();
      break;
  }
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Item details, picture and inventory
//--------------------------------------------------------------------------------------------------------------------------------------
void FreePicture(uint8_t* aBuffer) {
  if (aBuffer) { free(aBuffer); aBuffer = nullptr; }
}

// Downloads the picture into RAM (JPEG only, up to BUFFER_LIMIT bytes). 'Picture' is a complete URL.
bool FetchPicture(const String& url, bool isIcon) {
  if (isIcon) {
    FreePicture(IconBuf);
  } else {
    FreePicture(PicBuf);
  }
  bool https = url.startsWith("https://");
  if (!https) {
#if IMG_ALLOW_HTTP
    if (!url.startsWith("http://")) return false;
#else
    return false;
#endif
  }
  if (!EnsureWiFi()) return false;

  WiFiClient plain;
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(6000);
  if (!(https ? http.begin(img_client, url) : http.begin(plain, url))) return false;

  bool ok = false;
  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    int size = http.getSize();   // -1 when unknown (chunked)
    if (size <= (int)BUFFER_LIMIT) {
      size_t cap = (size > 0) ? (size_t)size : (size_t)BUFFER_LIMIT;
      uint8_t* buf = (uint8_t*)malloc(cap);
      if (buf) {
        MemSink sink(buf, cap);
        int got = http.writeToStream(&sink);
        if (got > 2 && sink.len > 2 && ((buf[0] == 0xFF && buf[1] == 0xD8) || (buf[1] == 0x50 && buf[2] == 0x4E && buf[3] == 0x47))) {   // JPEG or PNG magic number
          if (isIcon) {
            IconBuf = buf;
            IconLen = sink.len;
          } else {
            PicBuf = buf;
            PicLen = sink.len;
          }
          ok = true;
        } else {
          free(buf);
        }
      }
    }
  }
  http.end();
  return ok;
}

void LoadItemDetails(const String& id) {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.print("Loading Item...");

  idItem = id;
  CurrentItemName = CurrentItemPartNo = CurrentItemPic = CurrentItemUnit = "";
  InventoryId = "";
  InventoryQty = InventoryQtyOriginal = 0;
  FreePicture(PicBuf);

  if (!EnsureWiFi()) { ShowError("Wi-Fi lost"); UpdateDisplay(); return; }

  // 1) Item record (one small read)
  String payload = rtdb.get<String>(aClient, "/Item/" + idItem);
  if (aClient.lastError().code() == 0) {
    JsonDocument doc;
    if (!deserializeJson(doc, payload)) {
      CurrentItemName   = doc["Item_name"].as<String>();
      CurrentItemPartNo = doc["Part No"].as<String>();
      CurrentItemPic    = doc["Picture"].as<String>();
      CurrentItemUnit   = doc["Unit"].as<String>();
    }
  } else {
    CurrentItemName = "(error loading item)";
    Serial.printf("error loading /Item/%s: %s\n", idItem.c_str(), aClient.lastError().message().c_str());
  }

  // 2) Inventory record of this item (indexed query on Inventory/Item_ID)
  DatabaseOptions opt;
  opt.filter.orderBy("Item_ID");
  opt.filter.equalTo(idItem);
  opt.filter.limitToFirst(1);
  payload = rtdb.get<String>(aClient, "/Inventory", opt);
  if (aClient.lastError().code() == 0) {
    JsonDocument doc;
    if (payload.length() > 2 && !deserializeJson(doc, payload)) {
      for (JsonPair kv : doc.as<JsonObject>()) {
        InventoryId  = kv.key().c_str();
        InventoryQty = kv.value()["Quantity"].as<int>();
        break;
      }
    }
  } else {
    Serial.printf("error loading /Inventory for %s: %s\n", idItem.c_str(), aClient.lastError().message().c_str());
  }
  InventoryQtyOriginal = InventoryQty;

  // 3) Picture (cached in RAM)
  DrawBusy("Loading picture...");
  FetchPicture(CurrentItemPic);

  CurrentMenuStatus = MENU_ADJ_QTY;
  UpdateDisplay();
}

// Persists the updated inventory quantity to Firebase
void SaveQty() {
  // Nothing changed on an existing record: just go back
  if (InventoryId.length() && InventoryQty == InventoryQtyOriginal) { GoBack(); return; }

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(2, 50);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.print("Updating changes to Firebase...");

  bool ok = false;
  if (!EnsureWiFi()) {
    LastError = "Wi-Fi lost";
  } else if (InventoryId.length()) {
    // Existing record: update only the number (Quantity must be a number, see rules.json)
    ok = rtdb.set<int>(aClient, "/Inventory/" + InventoryId + "/Quantity", InventoryQty);
    if (!ok || aClient.lastError().code() != 0) { ok = false; LastError = aClient.lastError().message(); }
  } else {
    // First stock record of this item: the whole record is written at once so the
    // .validate rules (required children) are satisfied. Inventory_ID must equal its key.
    JsonDocument d;
    d["Inventory_ID"] = idItem;
    d["Item_ID"]      = idItem;
    d["Quantity"]     = InventoryQty;
    String s;
    serializeJson(d, s);
    ok = rtdb.set<object_t>(aClient, "/Inventory/" + idItem, object_t(s));
    if (!ok || aClient.lastError().code() != 0) { ok = false; LastError = aClient.lastError().message(); }
    else InventoryId = idItem;
  }

  if (ok) {
    InventoryQtyOriginal = InventoryQty;
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(2, 55);
    tft.print("Inventory updated!");
    delay(DELAY_TIME * 3);
    GoBack();                 // back to the item list, same page and same position
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.setCursor(2, 30);
    tft.setTextWrap(true, true);
    tft.printf("Error updating inventory:\n%s", LastError.c_str());
    tft.setTextWrap(false, false);
    Serial.printf("Error updating inventory: %s\n", LastError.c_str());
    delay(DELAY_TIME * 6);
    UpdateDisplay();          // stay on the quantity screen so the user can retry
  }
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Display battery status icon
//--------------------------------------------------------------------------------------------------------------------------------------
void showBatteryStatus(float volts) {
  String img;
  int x = 160 - 21, y = 1, w = 20, h = 32;
  float percent = (volts / 4.2f) * 100.0f;

  // check wich icon to show (charging or not, and battery level)
  if (charging()) {
    if (percent >= 80.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_ch4.png";}
    else if (percent >= 60.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_ch3.png";}
    else if (percent >= 40.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_ch2.png";}
    else if (percent >= 20.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_ch1.png";}
    else {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_ch0.png";}
  } else {
    if (percent >= 80.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_nc4.png";}
    else if (percent >= 60.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_nc3.png";}
    else if (percent >= 40.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_nc2.png";}
    else if (percent >= 20.0f) {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_nc1.png";}
    else {img = "https://cognitiva-pro.github.io/invtron_webapp/icons/batt_nc0.png";}
  }

  // only fetch the picture if it changed
  if (img != IconUrl) { 
    IconUrl = img;
    FetchPicture(img, true);
    if (IconBuf) { tft.drawPng(IconBuf, IconLen, x, y, w, h); }
  }
}



// --------------------------------------------------------------------------------------------------------------------------------------
// Display Drawing
// --------------------------------------------------------------------------------------------------------------------------------------
void UpdateDisplay() {
  if (CurrentMenuStatus == MENU_ADJ_QTY) DrawQtyScreen();
  else DrawList();

  // Periodic reading of channel INA3221 INA_CHANNEL
  if (inaAddr && millis() - inaT >= INA_PERIOD_MS) {
    showBatteryStatus(getInaVolts());
    inaT = millis();
  }

}

void DrawList() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_GREEN);

  const char* title = "";
  if      (CurrentMenuStatus == MENU_CLASS)       title = "CLASS";
  else if (CurrentMenuStatus == MENU_CATEGORY)    title = "CATEGORY";
  else if (CurrentMenuStatus == MENU_SUBCATEGORY) title = "SUBCATEGORY";
  else if (CurrentMenuStatus == MENU_ITEM)        title = "ITEM";

  // page marker: p3 = third page from the start; e / e-2 = last page / 2 pages before the last
  String pg;
  if (PageNo >= 0) pg = PageFromTail ? (PageNo == 0 ? String("end") : "end-" + String(PageNo)) : "p" + String(PageNo + 1);
  tft.print(String(title) + ":");
  tft.setTextColor(TFT_DARKGREY);
  tft.setCursor(140 - 8 * (pg.length() + 1), 2);
  tft.print(pg);
  tft.drawFastHLine(0, 12, 160, TFT_DARKGREY);

  if (Menu.empty()) {
    tft.setTextColor(TFT_DARKGREY);
    tft.setCursor(2, 22);
    tft.print("(empty)");
    return;
  }

  const int LINE_HEIGHT = 10;
  const int VISIBLE_LINES = 10;   // lines that fit between y=18 and y=118

  int WindowStart = 0;
  if (SelectedIndex >= VISIBLE_LINES) WindowStart = SelectedIndex - VISIBLE_LINES + 1;
  int WindowEnd = std::min((int)Menu.size(), WindowStart + VISIBLE_LINES);

  int y = 18;
  for (int i = WindowStart; i < WindowEnd; i++) {
    tft.setCursor(2, y);
    if (i == SelectedIndex) {
      tft.setTextColor(TFT_RED);
      tft.print(">");
    } else {
      tft.setTextColor(TFT_WHITE);
      tft.print(" ");
    }
    tft.print(truncar(Menu[i].label, 25));
    y += LINE_HEIGHT;
  }
}

void DrawQtyScreen() {
  tft.fillScreen(TFT_BLACK);

  // Layout: item photo at top-left, item name at top right, inventory below, and instructions at the bottom
  const int ItemPictureX = 2, ItemPictureY = 2, ItemPictureW = 44, ItemPictureH = 44;
  const int textoX = ItemPictureX + ItemPictureW + 4;
  const int textoW = 160 - textoX - 2;

  if (PicBuf && PicLen) {
    tft.drawJpg(PicBuf, PicLen, ItemPictureX, ItemPictureY, ItemPictureW, ItemPictureH);
  } else {
    tft.drawRect(ItemPictureX, ItemPictureY, ItemPictureW, ItemPictureH, TFT_DARKGREY);
    tft.setTextColor(TFT_DARKGREY);
    tft.setTextSize(1);
    tft.setCursor(ItemPictureX + 4, ItemPictureY + 20);
    tft.print("No Picture");
  }

  // Constrain the text area to avoid overlapping with the item picture
  tft.setClipRect(textoX, ItemPictureY, textoW, ItemPictureH);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(textoX, ItemPictureY);
  tft.setTextWrap(true, true);
  tft.print(CurrentItemPartNo);
  tft.print("\n");
  tft.setTextColor(TFT_WHITE);
  tft.print(CurrentItemName);
  tft.setTextWrap(false, false);
  tft.clearClipRect();

  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.setCursor(2, ItemPictureY + ItemPictureH + 4);
  tft.print("Qty:");

  DrawQty();

  tft.setTextSize(1);
  tft.setTextColor(TFT_RED);
  tft.setCursor(2, 112);
  tft.print("Push: save");
  tft.setCursor(2, 120);
  tft.print("Back: cancel");
}

// Redraws only the number (called on every encoder tick)
void DrawQty() {
  tft.fillRect(0, 58, 160, 22, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(2, 60);
  tft.printf("< %d > ", InventoryQty);
  tft.setTextSize(1);
  tft.print(CurrentItemUnit);
}
