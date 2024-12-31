#include <Arduino.h>
#include <ETH.h>
#include <PubSubClient.h>
#include <ElegantOTA.h> // ip/update
#include <EspAdsLib.h>
#include <Arduino_JSON.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_NeoPixel.h>

#include "config.html"
#include "index.html"

// Inputs/Outputs
#define EXT_LED_PIN   2
#define TEMP_SENS_PIN 35
#define NEO_PIXEL_PIN 5

#define NO_OF_BUTTONS     6
#define NO_OF_LEDS        6

#define BTN1_PIN 4
#define BTN2_PIN 32
#define BTN3_PIN 15
#define BTN4_PIN 14
#define BTN5_PIN 33
#define BTN6_PIN 34

#define BUTTON1 digitalRead(BTN1_PIN)
#define BUTTON2 digitalRead(BTN2_PIN)
#define BUTTON3 digitalRead(BTN3_PIN)
#define BUTTON4 digitalRead(BTN4_PIN)
#define BUTTON5 digitalRead(BTN5_PIN)
#define BUTTON6 digitalRead(BTN6_PIN)

#define PIN_TOGGLE(p) digitalWrite(p, !digitalRead(p));

#define ETH_ADDR                  1
#define ETH_RESET                 12
#define ETH_MDC_PIN               23
#define ETH_MDIO_PIN              18
#define ETH_TYPE                  ETH_PHY_LAN8720

#define FORMAT_LITTLEFS_IF_FAILED true



// Webserver
// ---------------------------------------------------------
AsyncWebServer  server(80); // Declare the WebServer object
AsyncEventSource events("/events");

// MQTT
// ---------------------------------------------------------
WiFiClient    ethClient;
PubSubClient  mqtt(ethClient);

char MqttTopicBtn[128];
char MqttTopicLed[128];
char MqttTopicStatus[128];

// ADS
// ---------------------------------------------------------
ADS::AmsAddr SrcAmsAddr((char*)"1.1.1.1.1.1", 43609);
ADS::AmsAddr DestAmsAddr((char*)"1.1.1.1.1.1", AMSPORT_R0_PLC_TC3);
ADS::Ads Ads;

// Website
// ---------------------------------------------------------
String  sSettings;
JSONVar SettingsJson;
JSONVar MqttLeds;
const static char* settingsfile    = "/settings";
#define DEFAULT_SETTINGS "{\"DevName\": \"EthSwitch\", \"server\": \"\", \"port\": \"1883\", \"user\": \"\", \"pass\": \"\", \"topic\": \"trash\", \"PlcIp\" : \"\", \"PlcAmsAddr\" : \"5.16.3.178.1.1\", \"PlcLedVar\": \"Main.u16LED\", \"PlcButtonVar\": \"Main.u16Button\" }"
String load_from_file(const char* file_name, String defaultvalue) ;
File         this_file;
bool bNewConnection = false;

// Neo Pixel
// ---------------------------------------------------------
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NO_OF_LEDS, NEO_PIXEL_PIN, NEO_RGB + NEO_KHZ800);

// Prototypes
// ---------------------------------------------------------
void loop();
void setup();
void LedTest(void);
bool MqttReconnect();
String GetClientId();
void UpdateLeds(void);
void ReadButtons(void);
bool AdsIsEnabled(void);
bool MqttIsEnabled(void);
void UpdateLedsOnWebsite(void);
String MacToStr(const uint8_t* mac);
float MCP9700_GetTemperature(uint16_t u16AdcRawValue);
bool write_to_file(const char* file_name, String contents);
String load_from_file(const char* file_name, String defaultvalue);
void MqttLedCallback(char* topic, byte* payload, unsigned int length);

// Structs
// ---------------------------------------------------------
typedef struct 
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
}sLed_t;

// Global var.
// ---------------------------------------------------------
char MqttStatusOnline[]  = "{\"Status\":\"Online\"}";
char MqttStatusOffline[] = "{\"Status\":\"Offline\"}"; // Last Will

sLed_t LEDS[NO_OF_LEDS];

String MacToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i)
    result += String(mac[i], 16);

  return result;
}


String GetClientId()
{
  String ClientId = (const char*)SettingsJson["DevName"];
  unsigned char mac[6];
  esp_read_mac(mac, ESP_MAC_ETH);
  ClientId += "_";
  ClientId += MacToStr(mac);
  return ClientId;
}

bool MqttIsEnabled(void)
{
  String s;
  s = SettingsJson["server"];
  if( s.length() > 3 )
    return true;
  else
    return false;
}

bool AdsIsEnabled(void)
{
  String s;
  s = SettingsJson["PlcIp"];
  if( s.length() > 3 )
    return true;
  else
    return false;
}

void UpdateLedsOnWebsite(void)
{
    JSONVar JsonLeds;
    for( int i = 0; i<NO_OF_LEDS; i++ )
      JsonLeds[i]["RGB"] = LEDS[i].R * 0x10000 + LEDS[i].G * 0x100 + LEDS[i].B; 
    
    events.send(JSON.stringify(JsonLeds).c_str(), "Leds",millis());
    Serial.println("Update LEDs on Website");
    Serial.print("  - ");
    Serial.println(JSON.stringify(JsonLeds).c_str());
}

String load_from_file(const char* file_name, String defaultvalue) 
{
  String result = "";

  this_file = LittleFS.open(file_name, "r");
  if (!this_file) { // failed to open the file, return defaultvalue
      return defaultvalue;
  }

  while (this_file.available()) {
      result += (char)this_file.read();
  }

  this_file.close();
  return result;
}

bool write_to_file(const char* file_name, String contents)
{
  File this_file = LittleFS.open(file_name, "w");
  if (!this_file) // failed to open the file, return false
      return false;

  int bytesWritten = this_file.print(contents);

  if (bytesWritten == 0)// write failed
      return false;

  this_file.close();
  return true;
}

#define ESP32_MAX_ADC_VALUE 0x0FFF
#define ESP32_REF_VOLTAGE 3.3f
// Offset: 500 mV @ 0 °C
// Gain: 10 mV / °C
float MCP9700_GetTemperature(uint16_t u16AdcRawValue)
{
  float tmp;
  tmp = (float)u16AdcRawValue * (ESP32_REF_VOLTAGE/(float)ESP32_MAX_ADC_VALUE);

  tmp -= 0.5f;
  tmp /= 0.01f;

  return tmp;
}

void setup()
{
  uint16_t u16TempRaw;
  float fTemp;

  String PlcIp;
  String ScrNetId;
  String DestNetId;
  String IP;

  // Inputs and Outputs
  // ---------------------------------------------

  // Buttons
  pinMode(BTN1_PIN, INPUT_PULLDOWN);
  pinMode(BTN2_PIN, INPUT_PULLDOWN);
  pinMode(BTN3_PIN, INPUT_PULLDOWN);
  pinMode(BTN4_PIN, INPUT_PULLDOWN);
  pinMode(BTN5_PIN, INPUT_PULLDOWN);
  pinMode(BTN6_PIN, INPUT_PULLDOWN);

  // User LED
  pinMode(EXT_LED_PIN, OUTPUT);

  // Serial
  Serial.begin(115200);

  // LED will blink 5 times
  for(int i=0; i<10; i++)
  {
    PIN_TOGGLE(EXT_LED_PIN);
    delay(100);
  }

  Serial.println("Starting up ...");

/*
  pinMode(TEMP_SENS_PIN, ANALOG);
  analogReadResolution(12);
  analogSetWidth(12);


  while(1)
  {
    delay(500);
    u16TempRaw = analogRead(TEMP_SENS_PIN);
    fTemp = MCP9700_GetTemperature(u16TempRaw);
    Serial.println(u16TempRaw);
    Serial.println(fTemp);
    Serial.println("---------------");
  }
  */
  
  // Little FS
  LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);

  // force defaults
  //#warning entfernen
  //write_to_file(settingsfile, DEFAULT_SETTINGS);

  // Load settings from filesystem
  sSettings = load_from_file(settingsfile,  DEFAULT_SETTINGS);
  SettingsJson = JSON.parse(sSettings);

  Serial.println("Current settings:");
  Serial.println(sSettings);

  // Init Neo Pixel
  strip.begin();
  strip.setBrightness(50);             // set the maximum LED intensity down to 50
  LedTest();

  // Turn all LEDs off
  memset(LEDS, 0, sizeof(LEDS));
  UpdateLeds();

  // Ethernet
  ETH.begin(ETH_ADDR, ETH_RESET, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLOCK_GPIO17_OUT);
  ETH.setHostname("EthSwitch");
  //ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP

  Serial.println("ETH.begin()");
  Serial.print("  - Link Speed: ");
  Serial.print(ETH.linkSpeed());
  Serial.print(" Mbit");

  while(!((uint32_t)ETH.localIP()))
  {
    delay(100);
    PIN_TOGGLE(EXT_LED_PIN);
  }; // Waiting for IP from DHCP

  IP = ETH.localIP().toString();
  Serial.println("  - IP: " + IP);

  // Default website requested
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", sIndexPage);
  });

  // Config wesite requested
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    String sJsonTxData =  "<!DOCTYPE html>\r\n<html>\r\n<script> var CurrentValues = '" + String(sSettings) + "'; </script>";
    Serial.print("Current settings: ");
    Serial.println(sSettings);
    sJsonTxData = sJsonTxData + String(sConfigPage);
    request->send_P(200, "text/html", sJsonTxData.c_str());
  });

  // Save button on config website has been pressed --> Save new config to file
  server.on("/save_new_config_data", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    int paramsNr = request->params();
    JSONVar doc;

    for(int i=0;i<paramsNr;i++)
    {
      AsyncWebParameter* p = request->getParam(i);
      doc[p->name()] = p->value();
    }

    String jsonString = JSON.stringify(doc);
    Serial.println(jsonString);

    write_to_file(settingsfile, jsonString);

    request->send_P(200, "text/html", "Data saved. Restarting device .... <br><button onclick=\"window.location.href='/';\">Back</button>");
    delay(1000);
    ESP.restart();
  });

  // Add event handler for live update of website
  server.addHandler(&events);
  events.onConnect([](AsyncEventSourceClient *client){
    Serial.println("Website is connected --> Send LED state");
    bNewConnection = true;
  });

  // Over The Ait Update (OTA)
  ElegantOTA.begin(&server); // Start ElegantOTA
  Serial.println("  - OTA started");

  server.begin(); // Start webserver
  Serial.println("  - Webserver started");

  // Config ADS
  if( AdsIsEnabled() )
  {
      Serial.println("  - ADS Enabled");
      PlcIp = SettingsJson["PlcIp"];
      DestNetId = SettingsJson["PlcAmsAddr"];
      DestAmsAddr.Change((char*)DestNetId.c_str(), AMSPORT_R0_PLC_TC3);

      IP = IP + ".1.1";
      SrcAmsAddr.Change((char*)IP.c_str(), 43609);
      
      Ads.SetAddr(&SrcAmsAddr, &DestAmsAddr, (char*)PlcIp.c_str());
      Ads.Connect();
      Serial.println("    - ADS connected");
  }
  else
  {
    Serial.println("  - ADS Disabled");
  }
  
  if( MqttIsEnabled() )
  {
    Serial.println("  - MQTT Enabled");
      // Create variables for MQTT toppics (LEDs, Buttons and Status)
    sprintf(MqttTopicBtn,      "%s/%s/Buttons", (const char*)SettingsJson["topic"], (const char*)SettingsJson["DevName"]);
    sprintf(MqttTopicLed,      "%s/%s/LEDS",    (const char*)SettingsJson["topic"], (const char*)SettingsJson["DevName"]);
    sprintf(MqttTopicStatus,   "%s/%s/Status",  (const char*)SettingsJson["topic"], (const char*)SettingsJson["DevName"]);
  }
  else
  {
    Serial.println("  - MQTT Disabled");
  }

  // Send the current LED state to website once after start up
  UpdateLedsOnWebsite();
}


// Check the Mqtt connection state and reconnect if necessary
// ----------------------------------------------------------
bool MqttReconnect()
{
    static long PreviousConnectTryMillis = 0;
    
    if (SettingsJson["server"].length() == 0)
    {
        //No server configured
        return false;
    }

    if (mqtt.connected())
        return true;

    if (millis() - PreviousConnectTryMillis >= (5000))
    {
        Serial.println("Attempting MQTT connection...");

        mqtt.setServer(SettingsJson["server"], atoi(SettingsJson["port"]));

        // Attempt to connect
        if( mqtt.connect(GetClientId().c_str(), (const char*)SettingsJson["user"], (const char*)SettingsJson["pass"], MqttTopicStatus, MQTTQOS1, true, MqttStatusOffline) )
        {
            Serial.println("  - Mqtt connected with ClientId " +  GetClientId());
            mqtt.subscribe(MqttTopicLed);
            mqtt.setKeepAlive(15); // 15 s
            mqtt.publish(MqttTopicStatus, MqttStatusOnline, true);
            return true;
        }
        else
        {
            Serial.print("  - Mqtt connect failed, rc=");
            Serial.println(mqtt.state());
            Serial.println(" - try again in 5 seconds");
        }

        PreviousConnectTryMillis = millis(); //Run only once every 5 seconds
    }
    return false;
}

// Will be called if the MQTT LED topic has new data --> Update LEDs and website
void MqttLedCallback(char* topic, byte* payload, unsigned int length) 
{
  int u32RGB;
  uint8_t u8MaxLedsObjs;

  String Buffer(payload, length);
  MqttLeds = JSON.parse(Buffer);

  Serial.println("Led update message received");
  Serial.print("  - ");
  Serial.println(JSON.stringify(MqttLeds).c_str());

  u8MaxLedsObjs = min(NO_OF_LEDS, MqttLeds.length());

  for(int i=0; i<u8MaxLedsObjs; i++ )
  {
    //Serial.println(JSON.stringify(MqttLeds[i]).c_str());
    u32RGB = MqttLeds[i]["RGB"];
    LEDS[i].R = (u32RGB&0x00FF0000)>>16;
    LEDS[i].G = (u32RGB&0x0000FF00)>>8;
    LEDS[i].B = (u32RGB&0x000000FF)>>0;
  }

  UpdateLedsOnWebsite();
  UpdateLeds();

  //Serial.println(LEDS[0].R);
  //Serial.println(LEDS[0].G);
  //Serial.println(LEDS[0].B);
}

std::string VarName;

void loop()
{
  static long LastAdsReadTime  = 0;
  static long LastAliveLedTime = 0;
  long now = millis();
  uint16_t u16Led;

  if( MqttIsEnabled() )
  {
    MqttReconnect();
    mqtt.setCallback(MqttLedCallback);
    mqtt.loop();
  }

  ReadButtons();
  UpdateLeds();

  ElegantOTA.loop();

  // to do: replace polling by notifications
  //        Variable will be read from PLC but not used yet
  if( AdsIsEnabled() )
  {
    if (now - LastAdsReadTime > 1000) 
    {
      LastAdsReadTime = now;
      VarName = std::string(SettingsJson["PlcLedVar"]);
      if( VarName.length() > 3 ) // no variable name set in config
      {
        Ads.ReadPlcVarByName(VarName, &u16Led, sizeof(uint16_t));
        Serial.println("Read Led from PLC: " + String(u16Led));
      }
    }
  }

  if (now - LastAliveLedTime > 500) 
  {
    LastAliveLedTime = now;
    PIN_TOGGLE(EXT_LED_PIN);
  }
}


// Button order          LED order
//  ╔═══════╦═══════╗    ╔═══════╦═══════╗
//  ║   1   ║   4   ║    ║   1   ║   6   ║  
//  ╠═══════║═══════╣    ╠═══════║═══════╣
//  ║   2   ║   5   ║    ║   2   ║   5   ║  
//  ╠═══════║═══════╣    ╠═══════║═══════╣
//  ║   3   ║   6   ║    ║   3   ║   4   ║
//  ╚═══════╩═══════╝    ╚═══════╩═══════╝  
           
uint8_t au8LedOrder[] = {0, 1, 2, 5, 4, 3};

void LedTest(void)
{
  for(long firstPixelHue = 0; firstPixelHue < 2*65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show(); // Update strip
    delay(10);    // Pause for a moment
  }

  strip.clear();
  strip.show(); // Update strip
}

void UpdateLeds(void)
{
  for( int i = 0; i<NO_OF_LEDS; i++)
    strip.setPixelColor(au8LedOrder[i], LEDS[i].G, LEDS[i].R, LEDS[i].B); // it seems that the colour channels are swappd for my LEDs(?)

  strip.show();
}

void AdsLedCallback(void* pData, size_t len)
{
  Serial.println("AdsLedCallback - Fuck Jeah");
}

String CreatButtonJson(uint8_t u8Btn)
{
   JSONVar BtnJson;
   for(int i=0; i<NO_OF_BUTTONS; i++ )
   {
       BtnJson[i]["state"] = (u8Btn & (1<<i)) ? 1 : 0;
   }

   return JSON.stringify(BtnJson);
}



void ReadButtons(void)
{
  static uint8_t u8BtnOld = 0x0;
  uint8_t  u8Btn = 0;
  uint16_t u16Btn; // muss in der PLC noch auf USINT umgestellt werden
  uint8_t  u8ChangedButtons;
  uint8_t  u8Mask = 0x01;
  bool     bState;

  if( BUTTON1 ) u8Btn += 0x01;
  if( BUTTON2 ) u8Btn += 0x02;
  if( BUTTON3 ) u8Btn += 0x04;
  if( BUTTON4 ) u8Btn += 0x08;
  if( BUTTON5 ) u8Btn += 0x10;
  if( BUTTON6 ) u8Btn += 0x20;

  u8ChangedButtons = u8BtnOld ^ u8Btn;
  
  if( u8ChangedButtons )
  {
    for(int i=1; i<=NO_OF_BUTTONS; i++)
    {
      bState = u8Btn&u8Mask;

      if( u8ChangedButtons & u8Mask ) // Has button changed?
      {
        Serial.print("Button ");
        Serial.print(i);

        if( bState )
        {
          Serial.println(" pressed");
        }
        else
        {
          Serial.println(" released");
        }
      }
      u8Mask = u8Mask << 1;
    }

    if( MqttIsEnabled() )
    {
      Serial.println("  - Send by MQTT");
      mqtt.publish(MqttTopicBtn, CreatButtonJson(u8Btn).c_str());
    }

    if( AdsIsEnabled() )
    {
      Serial.println("  - Send via ADS");
      VarName = std::string(SettingsJson["PlcButtonVar"]);
      u16Btn = u8Btn;
      Ads.WritePlcVarByName(VarName, &u16Btn, sizeof(uint16_t));
    }

    Serial.println("  - Send to website");
    events.send(CreatButtonJson(u8Btn).c_str(), "Buttons", millis());

    delay(10); // suppress bouncing button
  }
  

  u8BtnOld = u8Btn;

}
