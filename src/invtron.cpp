#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// CRITICAL FIREBASE MACROS MUST COME BEFORE THE INCLUDE LINE
#define ENABLE_USER_AUTH  // enables FirebaseClient user authentication
#define ENABLE_DATABASE   // enables RealtimeDatabase feature

#include <FirebaseClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <LovyanGFX.hpp>
#include <SPI.h>
#include "secret/credentials.h"

// Network/Wifi credentials (from secret/credendials.h)
#define WIFI_SSID     WIFI_SSID1
#define WIFI_PASSWORD WIFI_PASSWORD1

// Firebase credentials (from secret/credendials.h)
#define API_KEY       MY_API_KEY
#define DATABASE_URL  MY_DATABASE_URL
#define USER_EMAIL    MY_USER_EMAIL
#define USER_PASS     MY_USER_PASS

// Base URL for item pictures (.jpg)
// Ex.: Firebase Hosting: "https://your-project.web.app/images/"
// This firmware concatenates IMAGES_BASE_URL + the item Part. No. as the filename of an item's photo.
#define IMAGES_BASE_URL "https://SEU-PROJETO.web.app/images/"

// Begin of pin definitions for the ESP32-38 board
// -----------------------------------------------
// ESP32-38 LEFT SIDE:
//                  V5  // +5V from SX1308
//                  CMD // not connected
//                  SD3 // not connected
//                  SD2 // not connected
#define NC_13       13  // free to use, not connected
//                  GND // GND from SX1308
#define NC_12       12  // strapping pin, free to use, use with care!
#define NC_14       14  // free to use, not connected
// Nest 3 pins are used by the KY-040 Encoder
#define ENCODER_SW  27  // SW  - blue
#define ENCODER_DT  26  // DT  - yellow
#define ENCODER_CLK 25  // CLK - green
// Next 2 pins are used by the touch buttons
#define BTN_OK      33  // OK button   blue
#define BTN_BAK     32  // back button blue
#define NC_35       35  // free to use, not connected
#define NC_34       34  // input-only, free to use, not connected
#define NC_36_SN    36  // input-only, free to use, not connected
#define NC_39_SP    39  // input-only, free to use, not connected
#define NC_38_EN    38  // enable, used by the reset button (to GND)
//                  3V3 // 3.3V output to power the INA3221 battery level sensor

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
#define NC_05       05  // strapping pin, free to use, use with care!
#define TFT_A0      18  // DC/A0     blue
#define TFT_RST     19  // RESET     green
//                  GND // to INA3221 GND
#define TFT_CS      21  // CS        blue
//                  RSD // not connected
//                  TXD // not connected
// Pins used by the INA3221 battery level sensor:
#define INA_SCL     22  // INA_SCL    green, output-only, reserved for battery analog level meter enable 
#define INA_SDA     23  // INA_SDA    blue,  input-only,  reserved for battery analog level meter data
//                  GND // used by the reset button (to GND EN pin)
// End of pin definitions for the ESP32-38 board
// ---------------------------------------------

// This TFT display does not need MISO pin
#define TFT_MISO    -1  // This TFT display does not need MISO pin

// Set the behavior of libraries used
#define LGFX_USE_V1       // enables version 1 (config-based) features

// Application-specific constants
#define MAX_RETRIES      3 // max times to try to connect to an SSID
#define DELAY_TIME     500 // time-out for connecting to peripherals (in ms)
#define BUFFER_LIMIT 40000 // Image buffer size


//--------------------------------------------------------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------------------------------------------------------
void ConnectToWiFi(const char* ssid, const char* password);
void LoadMenuData(String path);
void UpdateDisplay();
void ApplyUpdatedQty();
void LoadItemDetails(String SelectedItemID);

//--------------------------------------------------------------------------------------------------------------------------------------
// Class definitions
//--------------------------------------------------------------------------------------------------------------------------------------
class LGFX_ESP32_Shield : public lgfx::LGFX_Device {
  lgfx::Panel_ST7735S _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
  lgfx::Light_PWM     _light_instance;

public:
  LGFX_ESP32_Shield() {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 27000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;

      cfg.pin_sclk = TFT_SCK;
      cfg.pin_mosi = TFT_SDA;
      cfg.pin_miso = TFT_MISO;
      cfg.pin_dc   = TFT_A0;

      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs   = TFT_CS;
      cfg.pin_rst  = TFT_RST;
      cfg.pin_busy = -1;

      cfg.panel_width  = 128;
      cfg.panel_height = 160;
      cfg.offset_x = 0;
      cfg.offset_y = 0;

      cfg.readable   = false; // no MISO, let's disable read operations
      cfg.bus_shared = false; // SD is not in use, so the SPI bus is not shared with any other device

      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl      = TFT_LED;
      cfg.invert      = false;
      cfg.freq        = 12000;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

// Instantiate the display class (tft) using the LGFX_ESP32_Shield class
LGFX_ESP32_Shield tft; 

// Instantiate Firebase user authentication
UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASS);

// Initialize Firebase components
FirebaseApp database_app;

// WiFiClientSecure is used for HTTPS connections to Firebase
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client); 

// Instance of RealtimeDatabase and AsyncResult
RealtimeDatabase rtdb; 

AsyncResult fb_result;

// Define the menu composition, status enum and initialize variables for menu navigation
enum MenuStatus { MENU_CLASS, MENU_CATEGORY, MENU_SUBCATEGORY, MENU_ITEM, MENU_ADJ_QTY };

// Define the translation dictionary for multiple languages (English, Portuguese, Spanish):
const String  T_Class       = "Class",
              T_Category    = "Category",
              T_Subcategory = "Subcategory",
              T_Location    = "Location",
              T_Item        = "Item",
              T_Inventory   = "Inventory",
              T_History     = "History",
              T_Exit        = "Exit";


/* = {
en:{About:"About",abT:"About INVTRON",ab1b:"Learn more about this project on our GitHub page: {u}",ab5:"This application and its components use, in whole or in part, code and images generated by A.I.",perm:"Denied by the database rules. Check that you are signed in and that the rules allow this write. Validation rules (.validate) also report this error, e.g. a value longer than allowed or outside a list (Part No, Cap, Unit, Tol%).",imfmt:"File format",cgA:"Your partner in A.I.",cgB:"Visit {u} to learn more",ab1:"INVTRON is a companion webapp for the INVTRON hardware.",ab2:"This software is provided \"as is\", without warranty of any kind, express or implied, including but not limited to any warranties against defects, bugs, merchantability, or fitness for a particular purpose. It is made available for use under the terms and conditions of the GNU Lesser General Public License (LGPL-2.1).",ab3:"In no event shall the authors or copyright holders be liable for any claim, damages, loss of data, or other liability, whether in an action of contract, tort or otherwise, arising from, out of, or in connection with the software or the use or other dealings in the software.",ab4:"Use it at your own discretion.",imopt:"optional",Import:"Import",imj:"Import from JSON",imc:"Import from CSV",imcol:"Collection",imcols:"Columns (* required)",imres:"{n} record(s) to add, {m} rejected.",imgo:"Proceed with the import?",imdone:"{n} record(s) imported.",imbad:"The file is empty or is not valid for the chosen format.",rQuote:"stray or unclosed quote: put the value in double quotes and double any inner quote (e.g. \"LCD 7\"\" 50 pins\")",rLn:"Line",rReq:"{f} is required",rRef:"{f} not found: {v}",rEnum:"invalid {f}: {v}",rUrl:"invalid URL in {f}",rLen:"{f} is too long (max {m})",rDup:"duplicate",Export:"Export",xj:"Export as JSON",xc:"Export as CSV",pick:"Choose a picture",srch:"Search pictures for this Part No",imgpn:"Enter the Part No first.",imgcfg:"Image search is not configured. Set IMG.key in the code.",imgnone:"No pictures found.",imgerr:"Could not reach the image search service.",imgbad:"That picture could not be loaded. Choose another one.",Classes:"Classes",Categories:"Categories",Subcategories:"Subcategories",Locations:"Locations",Items:"Items",
Class_name:"Class name",Category_name:"Category name",Subcategory_name:"Subcategory name",Location_name:"Location name",Item_name:"Item name",Qty:"Qty",
save:"Save",del:"Delete",ok:"OK",cancel:"Cancel",signin:"Sign in",
inUse:"This line is in use!",delItem:"Inventory history will be deleted too. Do you want to proceed?",usePic:"Use this picture?",nf:"Item not found",dup:"This Part No already exists!",out:"Sign out",pw:"Password",bad:"Invalid e-mail or password",slow:"Not confirmed by the server yet. Keep this page open: the change syncs when the connection returns. Check your connection, the databaseURL and the database rules.",chk:"Could not reach the Realtime Database. Check the databaseURL in cfg, that the database was created and that its rules allow signed-in users."},
pt:{About:"Sobre",abT:"Sobre INVTRON",ab1b:"Saiba mais sobre este projeto na nossa página do GitHub: {u}",ab5:"Este aplicativo e seus componentes utilizam no todo ou em parte código e imagens gerados por I.A.",perm:"Negado pelas regras do banco. Verifique se você está conectado e se as regras permitem esta gravação. Regras de validação (.validate) também geram este erro, por exemplo um valor maior que o permitido ou fora de uma lista (Part No, Cap, Unit, Tol%).",imfmt:"Formato do arquivo",cgA:"O seu parceiro em I.A.",cgB:"Acesse {u} e saiba mais",ab1:"O INVTRON é um webapp complementar ao hardware INVTRON.",ab2:"Este software é fornecido \"no estado em que se encontra\" (\"as is\"), sem garantia de qualquer tipo, expressa ou implícita, incluindo, entre outras, garantias contra defeitos, erros, comercialização ou adequação a uma finalidade específica. Ele é disponibilizado para uso nos termos e condições da GNU Lesser General Public License (LGPL-2.1).",ab3:"Em nenhuma hipótese os autores ou detentores dos direitos autorais serão responsáveis por qualquer reclamação, danos, perda de dados ou outra responsabilidade, seja em ação contratual, extracontratual ou de outra natureza, decorrente de, ou relacionada ao software, ao seu uso ou a outras negociações com o software.",ab4:"Use a seu critério.",imopt:"opcional",Import:"Importar",imj:"Importar de JSON",imc:"Importar de CSV",imcol:"Coleção",imcols:"Colunas (* obrigatório)",imres:"{n} registro(s) a incluir, {m} rejeitado(s).",imgo:"Deseja prosseguir com a importação?",imdone:"{n} registro(s) importado(s).",imbad:"O arquivo está vazio ou não é válido para o formato escolhido.",rQuote:"aspas soltas ou sem fechar: coloque o valor entre aspas duplas e dobre as aspas internas (ex.: \"LCD 7\"\" 50 pinos\")",rLn:"Linha",rReq:"{f} é obrigatório",rRef:"{f} não encontrado: {v}",rEnum:"{f} inválido: {v}",rUrl:"URL inválida em {f}",rLen:"{f} é longo demais (máx. {m})",rDup:"duplicado",Export:"Exportar",xj:"Exportar como JSON",xc:"Exportar como CSV",pick:"Escolha uma imagem",srch:"Buscar imagens deste Part No",imgpn:"Informe o Part No primeiro.",imgcfg:"A busca de imagens não está configurada. Preencha IMG.key no código.",imgnone:"Nenhuma imagem encontrada.",imgerr:"Não foi possível acessar o serviço de busca de imagens.",imgbad:"Não foi possível carregar essa imagem. Escolha outra.",Class:"Classe",Category:"Categoria",Subcategory:"Subcategoria",Location:"Local",Item:"Item",Inventory:"Inventário",History:"Histórico",
Classes:"Classes",Categories:"Categorias",Subcategories:"Subcategorias",Locations:"Locais",Items:"Itens",
Class_name:"Nome da classe",Category_name:"Nome da categoria",Subcategory_name:"Nome da subcategoria",Location_name:"Nome do local",Item_name:"Nome do item",
Unit:"Unid",Gain:"Ganho",Details:"Detalhes",Picture:"Imagem",Action:"Ação",Quantity:"Quantidade",Qty:"Qtd",
save:"Gravar",del:"Excluir",ok:"OK",cancel:"Cancelar",signin:"Entrar",
inUse:"Esta linha está em uso!",delItem:"O histórico do inventário também será excluído. Deseja continuar?",usePic:"Usar esta imagem?",nf:"Item não encontrado",dup:"Este Part No já existe!",out:"Sair",pw:"Senha",bad:"E-mail ou senha inválidos",slow:"Ainda não confirmado pelo servidor. Mantenha esta página aberta: a alteração sincroniza quando a conexão voltar. Verifique a conexão, a databaseURL e as regras do banco.",chk:"Não foi possível acessar o Realtime Database. Verifique a databaseURL no cfg, se o banco foi criado e se as regras permitem usuários autenticados."},
es:{About:"Acerca de",abT:"Acerca de INVTRON",ab1b:"Conozca más sobre este proyecto en nuestra página de GitHub: {u}",ab5:"Esta aplicación y sus componentes utilizan, total o parcialmente, código e imágenes generados por I.A.",perm:"Denegado por las reglas de la base. Compruebe que ha iniciado sesión y que las reglas permiten esta escritura. Las reglas de validación (.validate) también generan este error, p. ej. un valor más largo de lo permitido o fuera de una lista (Part No, Cap, Unit, Tol%).",imfmt:"Formato del archivo",cgA:"Su socio en I.A.",cgB:"Visite {u} y conozca más",ab1:"INVTRON es una aplicación web complementaria del hardware INVTRON.",ab2:"Este software se proporciona \"tal cual\", sin garantía de ningún tipo, expresa o implícita, incluidas, entre otras, las garantías contra defectos, errores, comerciabilidad o idoneidad para un fin determinado. Se pone a disposición para su uso conforme a los términos y condiciones de la GNU Lesser General Public License (LGPL-2.1).",ab3:"En ningún caso los autores o titulares de los derechos de autor serán responsables de ninguna reclamación, daño, pérdida de datos u otra responsabilidad, ya sea en una acción contractual, extracontractual o de otro tipo, que surja de, o esté relacionada con, el software, su uso u otros tratos con el software.",ab4:"Úselo según su propio criterio.",imopt:"opcional",Import:"Importar",imj:"Importar desde JSON",imc:"Importar desde CSV",imcol:"Colección",imcols:"Columnas (* obligatorio)",imres:"{n} registro(s) por añadir, {m} rechazado(s).",imgo:"¿Continuar con la importación?",imdone:"{n} registro(s) importado(s).",imbad:"El archivo está vacío o no es válido para el formato elegido.",rQuote:"comillas sueltas o sin cerrar: ponga el valor entre comillas dobles y duplique las internas (p. ej.: \"LCD 7\"\" 50 pines\")",rLn:"Línea",rReq:"{f} es obligatorio",rRef:"{f} no encontrado: {v}",rEnum:"{f} no válido: {v}",rUrl:"URL no válida en {f}",rLen:"{f} es demasiado largo (máx. {m})",rDup:"duplicado",Export:"Exportar",xj:"Exportar como JSON",xc:"Exportar como CSV",pick:"Elija una imagen",srch:"Buscar imágenes de este Part No",imgpn:"Introduzca primero el Part No.",imgcfg:"La búsqueda de imágenes no está configurada. Complete IMG.key en el código.",imgnone:"No se encontraron imágenes.",imgerr:"No se pudo acceder al servicio de búsqueda de imágenes.",imgbad:"No se pudo cargar esa imagen. Elija otra.",Class:"Clase",Category:"Categoría",Subcategory:"Subcategoría",Location:"Ubicación",Item:"Artículo",Inventory:"Inventario",History:"Historial",
Classes:"Clases",Categories:"Categorías",Subcategories:"Subcategorías",Locations:"Ubicaciones",Items:"Artículos",
Class_name:"Nombre de la clase",Category_name:"Nombre de la categoría",Subcategory_name:"Nombre de la subcategoría",Location_name:"Nombre de la ubicación",Item_name:"Nombre del artículo",
Unit:"Unidad",Gain:"Ganancia",Details:"Detalles",Picture:"Imagen",Action:"Acción",Quantity:"Cantidad",Qty:"Cant.",
save:"Guardar",del:"Eliminar",ok:"OK",cancel:"Cancelar",signin:"Entrar",
inUse:"¡Esta línea está en uso!",delItem:"El historial del inventario también se eliminará. ¿Desea continuar?",usePic:"¿Usar esta imagen?",nf:"Artículo no encontrado",dup:"¡Este Part No ya existe!",out:"Salir",pw:"Contraseña",bad:"Correo o contraseña no válidos",slow:"Aún no confirmado por el servidor. Mantenga esta página abierta: el cambio se sincroniza al volver la conexión. Compruebe la conexión, la databaseURL y las reglas.",chk:"No se pudo acceder a Realtime Database. Compruebe la databaseURL en cfg, que la base exista y que sus reglas permitan usuarios autenticados."}};
*/

MenuStatus CurrentMenuStatus = MENU_CLASS;

// Variables to hold the current selection and inventory quantity
String idClass = "", idCat = "", idSub = "", idItem = "";
String CurrentItemName = "", CurrentItemPic = "";
int InventoryQty = 0;

// Vectors to hold menu keys and values, and the selected index
std::vector<String> MenuKeys; 
std::vector<String> MenuValues; 
int SelectedIndex = 0;

// Encoder state variables
volatile bool EncoderTurned = false;
volatile int TurnDirection = 0; 
int  LastCLKStatus;
bool ButtonPressed = false;

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
// SETUP
//--------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  int retries = 0;
  unsigned long Auth_Start;
  
  Serial.begin(115200);
  
  // Initialize the encoder pins and attach the interrupt
  Serial.println ("Inicializing Encoder");
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);
  LastCLKStatus = digitalRead(ENCODER_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), ReadEncoder, CHANGE);

  // Initialize the TFT display
  Serial.println ("Inicializing TFT Display");
  tft.init();

   // 0:   0 degrees (normal portrait mode); 
   // 1:  90 degrees clockwise (landscape mode); 
   // 2: 180 degrees (inverted portrait); 
   // 3: 270 degrees clockwise (inverted landscape)
  tft.setRotation(1); 
  
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);


  // Wifi connection
  // Assume you have put your Wi-Fi credentials in secret/credentials.h
  // You can have two sets of credentials (WIFI_SSID1/WIFI_PASSWORD1 and WIFI_SSID2/WIFI_PASSWORD2) to 
  // try connecting to two different networks (e.g., home and office). The code will attempt to connect 
  // to the first one, and if it fails, it will try the second one.
  
  // Attempt to connect to the first Wi-Fi network
  ConnectToWiFi(WIFI_SSID1, WIFI_PASSWORD1);
  if (WiFi.status() != WL_CONNECTED) { 
    Serial.printf("\n > Warning! Wi-Fi %s is not responding, trying %s instead.\n", WIFI_SSID1, WIFI_SSID2);
    tft.printf("\n > Warning! Wi-Fi %s is not responding, trying %s instead.\n", WIFI_SSID1, WIFI_SSID2);   

    // Attempt to connect to the second Wi-Fi network
    ConnectToWiFi(WIFI_SSID2, WIFI_PASSWORD2);
    if (WiFi.status() != WL_CONNECTED) { 
      Serial.print("\nERROR 01: Couldn't connect to Wi-Fi!\n");
      Serial.print("Device halted!\n");
      tft.print("\nERROR 01: Couldn't connect to Wi-Fi!\n");
      tft.print("Device halted!\n");
      esp_deep_sleep_start(); 
    }
  }

  tft.printf("Connected to Wi-Fi %s\n", WIFI_SSID);
  Serial.printf("Connected to Wi-Fi %s\n", WIFI_SSID);
 
  // Set the WiFiClientSecure to insecure mode to avoid SSL certificate verification issues
  ssl_client.setInsecure(); 
  
  // Inicialize FirebaseApp with the secure client and user authentication
  initializeApp(aClient, database_app, getAuth(user_auth));
  
  // Configure the RealtimeDatabase instance with the database URL
  database_app.getApp<RealtimeDatabase>(rtdb);
  rtdb.url(DATABASE_URL);

  // Display a message indicating that Firebase authentication is in progress
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(2, 2);
  tft.print("Firebase authentication in progress...\n");
  Serial.print("Firebase authentication in progress...\n");
  
  
  Auth_Start = millis();
  while (!database_app.ready()) {
    database_app.loop();
    if (millis() - Auth_Start > (DELAY_TIME * 40)) {
      tft.print("\nERROR 02: Firebase authentication timed-out!\n");
      tft.println("Device halted!\n");
      Serial.println("\nERROR 02: Firebase authentication timed-out");
      Serial.println("Device halted!\n");
      break;
      Serial.flush();
      esp_deep_sleep_start();
    }
    Serial.print(".");
    tft.print(".");
    delay(DELAY_TIME);
  }

  // Load first dataset
  tft.print("Loading class data...\n");
  Serial.print("Loading class data...\n");
  LoadMenuData("/class");
}



//--------------------------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  // Run the FirebaseApp loop to handle background tasks and maintain the connection
  database_app.loop();

  // Check if the encoder has been turned and update the menu selection or inventory quantity accordingly
  if (EncoderTurned) {
    EncoderTurned = false; 
    if (CurrentMenuStatus == MENU_ADJ_QTY) {
      if (TurnDirection == 1) InventoryQty++;
      else if (TurnDirection == -1 && InventoryQty > 0) InventoryQty--;
    } else {
      if (TurnDirection == 1) { 
        if (SelectedIndex < (int)MenuKeys.size() - 1) SelectedIndex++;
        else SelectedIndex = 0;
      } else if (TurnDirection == -1) { 
        if (SelectedIndex > 0) SelectedIndex--;
        else SelectedIndex = (int)MenuKeys.size() - 1;
      }
    }
    UpdateDisplay();
  }

  if (digitalRead(ENCODER_SW) == LOW && !ButtonPressed) {
    ButtonPressed = true;
    delay(DELAY_TIME / 10); // debounce delay
    if (digitalRead(ENCODER_SW) == LOW) ApplyUpdatedQty();
  }
  if (digitalRead(ENCODER_SW) == HIGH) ButtonPressed = false; 
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Application Functions
//--------------------------------------------------------------------------------------------------------------------------------------

// Wi-Fi connection
void ConnectToWiFi(const char* _WIFI_SSID, const char* _WIFI_PASSWORD) {
  Serial.printf("Conecting to Wi-Fi %s\n", _WIFI_SSID);
  tft.printf("Conecting to Wi-Fi %s\n", _WIFI_SSID);
  
  WiFi.begin(_WIFI_SSID, _WIFI_PASSWORD);
  unsigned long Auth_Start = millis();

  while (WiFi.status() != WL_CONNECTED) { 
    if (millis() - Auth_Start > (DELAY_TIME * 40)) {
      break;
    }
    Serial.print(".");
    tft.print(".");
    delay(DELAY_TIME);
  }
} 


// Load menu data from Firebase and populate the menu vectors
void LoadMenuData(String path) {
  String aSTR;
  tft.fillScreen(TFT_BLACK); 
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.print("Fetching data from the cloud...\n"); 
  Serial.print("Fetching data from the cloud...\n");

  // Initialize menu vectors and reset the selected index
  MenuKeys.clear();
  MenuValues.clear();
  SelectedIndex = 0;

  // Read data from Firebase
  rtdb.get(aClient, path, fb_result);

  if (!fb_result.isError()) {
    Serial.print("Decoding JSON...\n");

    String jsonStr = fb_result.c_str(); 
    JsonDocument doc;
    deserializeJson(doc, jsonStr);
    JsonObject obj = doc.as<JsonObject>();

    Serial.print(" -Retrieving menu keys...\n");
    for (JsonPair kv : obj) {
      MenuKeys.push_back(kv.key().c_str());
      aSTR = kv.value().as<String>();
      MenuValues.push_back(aSTR);     
      Serial.printf("  -[%s]]\n", aSTR);
      // MenuValues.push_back(kv.value().as<String>());
    }
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(2, 30);
    tft.printf("\nERROR 03: Failed to fetch data from the cloud:\n%s", fb_result.error().message().c_str());
    Serial.printf("\nERROR 03: Failed to fetch data from the cloud:\n%s", fb_result.error().message().c_str());
    delay(DELAY_TIME * 4);
  }
  UpdateDisplay();
}




//--------------------------------------------------------------------------------------------------------------------------------------
// Downloads and draws item picture from the cloud. If not available, draws a placeholder rectangle instead
//--------------------------------------------------------------------------------------------------------------------------------------
bool DownloadAndDrawItemPicture(const String& PictFileName, int x, int y, int w, int h) {
  if (PictFileName.length() == 0) return false;

  const size_t MAX_PIC_SIZE = BUFFER_LIMIT;
  String url = String(IMAGES_BASE_URL) + PictFileName;

  HTTPClient http;
  bool ok = false;

  if (url.startsWith("https://")) {
    if (!http.begin(ssl_client, url)) return false;
  } else {
    if (!http.begin(url)) return false;
  }

  int HTTP_Code = http.GET();
  if (HTTP_Code == HTTP_CODE_OK) {
    int HTTP_StreamSize = http.getSize();
    if (HTTP_StreamSize > 0 && (size_t)HTTP_StreamSize <= MAX_PIC_SIZE) {
      uint8_t* buf = (uint8_t*)malloc(HTTP_StreamSize);
      if (buf) {
        WiFiClient* stream = http.getStreamPtr();
        size_t HTTP_BytesRead = 0;
        unsigned long TimeBegin = millis();
        while (HTTP_BytesRead < (size_t)HTTP_StreamSize && http.connected() && (millis() - TimeBegin) < 5000) {
          size_t StreamSize = stream->available();
          if (StreamSize) {
            int BytesRead = stream->readBytes(buf + HTTP_BytesRead, min(StreamSize, (size_t)(HTTP_StreamSize - HTTP_BytesRead)));
            HTTP_BytesRead += BytesRead;
          } else {
            delay(1);
          }
        }
        if (HTTP_BytesRead == (size_t)HTTP_StreamSize) {
          tft.drawJpg(buf, HTTP_StreamSize, x, y, w, h);
          ok = true;
        }
        free(buf);
      }
    }
  }
  http.end();
  return ok;
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Truncates a string to a maximum length, adding "..." if it exceeds that length
//--------------------------------------------------------------------------------------------------------------------------------------
String truncar(const String& s, int maxLen) {
  if ((int)s.length() <= maxLen) return s;
  return s.substring(0, maxLen - 3) + "...";
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Updates the display based on the current menu status and selection
//--------------------------------------------------------------------------------------------------------------------------------------  
void UpdateDisplay() {
  tft.fillScreen(TFT_BLACK);

  if (CurrentMenuStatus == MENU_ADJ_QTY) {
    // Layout: item photo at top-left, item name at top right, inventory below, and instructions at the bottom
    const int ItemPictureX = 2, ItemPictureY = 2, ItemPictureW = 44, ItemPictureH = 44;
    const int textoX = ItemPictureX + ItemPictureW + 4;
    const int textoW = 160 - textoX - 2;

    if (!DownloadAndDrawItemPicture(CurrentItemPic, ItemPictureX, ItemPictureY, ItemPictureW, ItemPictureH)) {
      tft.drawRect(ItemPictureX, ItemPictureY, ItemPictureW, ItemPictureH, TFT_DARKGREY);
      tft.setTextColor(TFT_DARKGREY);
      tft.setTextSize(1);
      tft.setCursor(ItemPictureX + 4, ItemPictureY + 20);
      tft.print("No Picture");
    }

    // Constrain the text area to avoid overlapping with the item picture
    tft.setClipRect(textoX, ItemPictureY, textoW, ItemPictureH);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(textoX, ItemPictureY);
    tft.setTextWrap(true, true);
    tft.print(CurrentItemName);
    tft.setTextWrap(false, false);
    tft.clearClipRect();

    // Display the inventory quantity below the item picture
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(1);
    tft.setCursor(2, ItemPictureY + ItemPictureH + 4);
    tft.print("Qty:");

    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN);
    tft.setCursor(2, ItemPictureY + ItemPictureH + 5);
    tft.printf("< %d >", InventoryQty);

    // Display instructions at the bottom of the screen
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED);
    tft.setCursor(2, 118);
    tft.print("Push to confirm");
    return;
  }

  // Display classe, categories, subcategories, and items
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_GREEN);
  if (CurrentMenuStatus == MENU_CLASS) tft.println("CLASS:");
  else if (CurrentMenuStatus == MENU_CATEGORY) tft.println("CATEGORY:");
  else if (CurrentMenuStatus == MENU_SUBCATEGORY) tft.println("SUBCATEGORY:");
  else if (CurrentMenuStatus == MENU_ITEM) tft.println("ITEM:");

  tft.setTextColor(TFT_WHITE);
  tft.println("----------------------------");

  const int LINE_HEIGH = 10;
  const int VISIBLE_LINES = 10; // the number of lines that fit between y=22 and y=118 (~96px / 10px)

  int WindowStart = 0;
  if (SelectedIndex >= VISIBLE_LINES) WindowStart = SelectedIndex - VISIBLE_LINES + 1;
  int WindowEnd = min((int)MenuValues.size(), WindowStart + VISIBLE_LINES);

  int y = 22;
  for (int i = WindowStart; i < WindowEnd; i++) {
    tft.setCursor(2, y);
    if (i == SelectedIndex) {
      tft.setTextColor(TFT_RED);
      tft.print(">");
    } else {
      tft.setTextColor(TFT_WHITE);
      tft.print(" ");
    }
    tft.println(truncar(MenuValues[i], 25));
    y += LINE_HEIGH;
  }
}



//--------------------------------------------------------------------------------------------------------------------------------------
//Fetches the details of the selected item from Firebase and updates the display and enables the user to adjust the inventory quantity
//--------------------------------------------------------------------------------------------------------------------------------------
void LoadItemDetails(String SelectedItemID) {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(2, 2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.print("Loading Item...");

  idItem = SelectedItemID;
  CurrentItemName = "";
  CurrentItemPic = "";
  InventoryQty = 0;

  rtdb.get(aClient, "/items/" + idItem, fb_result);
  if (!fb_result.isError()) {
    JsonDocument doc;
    deserializeJson(doc, fb_result.c_str());
    CurrentItemName = doc["ItemName"].as<String>();
    CurrentItemPic = doc["ItemPicture"].as<String>();
  } else {
    CurrentItemName = "(error loading /items)";
    Serial.printf("error loading /items/%s: %s\n", idItem.c_str(), fb_result.error().message().c_str());
  }

  rtdb.get(aClient, "/inventory/" + idItem + "/qty", fb_result);
  if (!fb_result.isError()) {
    InventoryQty = String(fb_result.c_str()).toInt();
  } else {
    Serial.printf("Erro loading /inventory/%s/qty: %s\n", idItem.c_str(), fb_result.error().message().c_str());
  }

  CurrentMenuStatus = MENU_ADJ_QTY;
  UpdateDisplay();
}



//--------------------------------------------------------------------------------------------------------------------------------------
// Persists the updated inventory quantity to Firebase
//--------------------------------------------------------------------------------------------------------------------------------------
void ApplyUpdatedQty() {
  if (MenuKeys.size() == 0 && CurrentMenuStatus != MENU_ADJ_QTY) return;

  switch (CurrentMenuStatus) {
    case MENU_CLASS:
      idClass = MenuKeys[SelectedIndex];
      CurrentMenuStatus = MENU_CATEGORY;
      LoadMenuData("/categories/" + idClass);
      break;    
      
      case MENU_CATEGORY:
      idCat = MenuKeys[SelectedIndex];
      CurrentMenuStatus = MENU_SUBCATEGORY;
      LoadMenuData("/subcategories/" + idCat);
      break;

    case MENU_SUBCATEGORY:
      idSub = MenuKeys[SelectedIndex];
      CurrentMenuStatus = MENU_ITEM;
      LoadMenuData("/items_per_subcategory/" + idSub);
      break;

    case MENU_ITEM:
      LoadItemDetails(MenuKeys[SelectedIndex]);
      break;

    case MENU_ADJ_QTY: {
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(2, 50);
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.print("Updating changes to Firebase...");

      String PathToUpdate = "/inventory/" + idItem + "/qty";

      // Uses .set since the payload is a String
      rtdb.set(aClient, PathToUpdate, String(InventoryQty), fb_result);

      if (!fb_result.isError()) {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(2, 55);
        tft.print("Inventory updated!");
        delay(DELAY_TIME * 3);
      } else {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(2, 40);
        tft.printf("Error updating inventory:\n%s", fb_result.error().message().c_str());
        delay(DELAY_TIME * 5);
      }

      CurrentMenuStatus = MENU_CATEGORY;
      LoadMenuData("/categories");
      break;
    }
  }
}
