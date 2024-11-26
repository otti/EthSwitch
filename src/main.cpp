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
#include "ParInSerOutShiftReg.h"


// Inputs/Outputs
#define EXT_LED_PIN   2
#define TEMP_SENS_PIN 35
#define NEO_PIXEL_PIN 5

#define NO_OF_BUTTONS     6
#define NO_OF_LEDS        1

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

void ReadButtons(void);
void UpdateLeds(void);

// Webserver
AsyncWebServer  server(80); // Declare the WebServer object
AsyncEventSource events("/events");

// MQTT
WiFiClient    ethClient;
PubSubClient  mqtt(ethClient);

// ADS
ADS::AmsAddr SrcAmsAddr((char*)"1.1.1.1.1.1", 43609);
ADS::AmsAddr DestAmsAddr((char*)"1.1.1.1.1.1", AMSPORT_R0_PLC_TC3);
ADS::Ads Ads;

// Website
String  sSettings;
JSONVar SettingsJson;
JSONVar MqttLeds;
const static char* settingsfile    = "/settings";
#define DEFAULT_SETTINGS "{\"DevName\": \"EthSwitch\", \"server\": \"mosquitto.lan\", \"port\": \"1883\", \"user\": \"\", \"pass\": \"\", \"topic\": \"trash\", \"PlcIp\" : \"plc.lan\", \"PlcAmsAddr\" : \"5.16.3.178.1.1\", \"PlcLedVar\": \"Main.u16LED\", \"PlcButtonVar\": \"Main.u16Button\" }"
String load_from_file(const char* file_name, String defaultvalue) ;
File         this_file;
bool bNewConnection = false;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NO_OF_LEDS, NEO_PIXEL_PIN, NEO_RGB + NEO_KHZ800);

char MqttBtnTopic[128];
char MqttLedTopic[128];

typedef struct 
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
}sLed_t;


sLed_t LEDS[NO_OF_LEDS];


String MacToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i)
    result += String(mac[i], 16);

  return result;
}

// Ethernet MAC: a0:b7:65:dc:c3:53
String GetClientId()
{
  String ClientId = "EthSwitch_";
  unsigned char mac[6];
  esp_read_mac(mac, ESP_MAC_ETH);
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
    {
      JsonLeds[i]["RGB"] = LEDS[i].R * 0x10000 + LEDS[i].G * 0x100 + LEDS[i].B; 
    }
    
    events.send(JSON.stringify(JsonLeds).c_str(), "Leds",millis());
    Serial.println("Update LEDs on Website");
    Serial.print("  - ");
    Serial.println(JSON.stringify(JsonLeds).c_str());
}

// u8BtnNo from 1 to NO_OF_BUTTONS
void mqtt_send_btn_state(uint8_t u8BtnNo, bool state)
{
  char topic[128];
  char msg[5];
  
  sprintf(topic, "%s/%s/BTN_CH%i", (const char*)SettingsJson["topic"], (const char*)SettingsJson["DevName"], u8BtnNo);
  Serial.println(topic);

  if( state )
    strcpy(msg, "ON");
  else
    strcpy(msg, "OFF");

  if( MqttIsEnabled() )
  {
    mqtt.publish(topic, msg);
  }

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
  if (!this_file) { // failed to open the file, return false
      return false;
  }

  int bytesWritten = this_file.print(contents);

  if (bytesWritten == 0) { // write failed
      return false;
  }

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
  delay(2500); // The serial monitor needs some time to start up after programming
  Serial.println("hello");

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

  // Load settings from filesystem
  sSettings = load_from_file(settingsfile,  DEFAULT_SETTINGS);
  SettingsJson = JSON.parse(sSettings);

  Serial.println("Current settings:");
  Serial.println(sSettings);

  strip.begin();
  strip.setBrightness(50);             // set the maximum LED intensity down to 50

  // Turn all LEDs off
  memset(LEDS, 0, sizeof(LEDS));
  UpdateLeds();

  for(int i=0; i<10; i++)
  {
    PIN_TOGGLE(EXT_LED_PIN);
    delay(150);
  }

  // Ethernet
  ETH.begin(ETH_ADDR, ETH_RESET, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLOCK_GPIO17_OUT); // Enable ETH
  ETH.setHostname("EthSwitch");
  //ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP

  Serial.println("ETH.begin()");
  Serial.println("Link Speed: ");
  Serial.print(ETH.linkSpeed());

  while(!((uint32_t)ETH.localIP())){}; // Waiting for IP (leave this line group to get IP via DHCP)

  IP = ETH.localIP().toString();
  Serial.println("IP: " + IP);

  // Config Webserver
 server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", sIndexPage);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    String sJsonTxData =  "<!DOCTYPE html>\r\n<html>\r\n<script> var CurrentValues = '" + String(sSettings) + "'; </script>";
    Serial.print("Current settings: ");
    Serial.println(sSettings);
    //server.send(200, "text/html", sJsonTxData+ String(sConfigPage));
    sJsonTxData = sJsonTxData + String(sConfigPage);
    request->send_P(200, "text/html", sJsonTxData.c_str());
  });

  // Save new config to file
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


  server.addHandler(&events);
  events.onConnect([](AsyncEventSourceClient *client){
    Serial.println("Website is connected --> Send LED state");
    bNewConnection = true;
  });

  // OTA
  ElegantOTA.begin(&server);    // Start ElegantOTA

  server.begin(); // Start server
  Serial.println("Web server started");

  // Config ADS
  if( AdsIsEnabled() )
  {
      Serial.println("ADS Enabled");
      PlcIp = SettingsJson["PlcIp"];
      DestNetId = SettingsJson["PlcAmsAddr"];
      DestAmsAddr.Change((char*)DestNetId.c_str(), AMSPORT_R0_PLC_TC3);

      IP = IP + ".1.1";
      SrcAmsAddr.Change((char*)IP.c_str(), 43609);
      
      Ads.SetAddr(&SrcAmsAddr, &DestAmsAddr, (char*)PlcIp.c_str());
      Ads.Connect();
  }
  else
  {
    Serial.println("ADS Disabled");
  }

  if( MqttIsEnabled() )
    Serial.println("MQTT Enabled");
  else
    Serial.println("MQTT Disabled");

  
  sprintf(MqttBtnTopic, "%s/%s/Buttons", (const char*)SettingsJson["topic"], (const char*)SettingsJson["DevName"]);
  sprintf(MqttLedTopic, "%s/%s/LEDS",    (const char*)SettingsJson["topic"], (const char*)SettingsJson["DevName"]);

  UpdateLedsOnWebsite(); // Once after startup

}



// -------------------------------------------------------
// Check the Mqtt status and reconnect if necessary
// -------------------------------------------------------
long PreviousConnectTryMillis = 0;
bool MqttReconnect()
{
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
        if( mqtt.connect(GetClientId().c_str(), (const char*)SettingsJson["user"], (const char*)SettingsJson["pass"]) )
        {
            Serial.println("  Mqtt connected with ClientId " +  GetClientId());
            mqtt.subscribe(MqttLedTopic);
            return true;
        }
        else
        {
          
            Serial.print("  Mqtt connect failed, rc=");
            Serial.print(mqtt.state());
            Serial.println(" try again in 5 seconds");
        }

        PreviousConnectTryMillis = millis(); //Run only once every 5 seconds
    }
    return false;
}

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



long lastMsg = 0;
char text[128];

uint16_t u16Led;
uint16_t u16LedOld = 0xFFFF;
std::string VarName;

void loop()
{
  long now = millis();

  if( MqttIsEnabled() )
  {
    MqttReconnect();
    mqtt.setCallback(MqttLedCallback);
    mqtt.loop();
  }

  ReadButtons();
  UpdateLeds();

  ElegantOTA.loop();

  if (now - lastMsg > 1000) 
  {
    if( AdsIsEnabled() )
    {
      VarName = std::string(SettingsJson["PlcLedVar"]);
      if( VarName.length() > 3 ) // no variable name set in config
      {
        Ads.ReadPlcVarByName(VarName, &u16Led, sizeof(uint16_t));
        Serial.println("Read Led from PLC: " + String(u16Led));
      }
    }
    lastMsg = now;
  }

}

bool bSwitchOld = true;


void UpdateLeds(void)
{

  for( int i = 0; i<NO_OF_LEDS; i++)
    strip.setPixelColor(i, LEDS[i].G, LEDS[i].R, LEDS[i].B); // it seems that the colour channels are swappd for my LEDs(?)

  strip.show();
}

void AdsLedCallback(void* pData, size_t len)
{
  Serial.println("AdsLedCallback - Fuck Jeah");
}

#warning send last will for buttons

long lastBtnRead = 0;
uint8_t u8BtnOld = 0x0;
void ReadButtons(void)
{

  uint8_t u8Btn = 0;
  uint16_t u16Btn; // muss in der PLC noch auf USINT umgestellt werden
  uint8_t u8ChangedButtons;
  uint8_t u8Mask = 0x01;
  bool bState;
  JSONVar BtnJson;

  if( BUTTON1 ) u8Btn += 0x01;
  if( BUTTON2 ) u8Btn += 0x02;
  if( BUTTON3 ) u8Btn += 0x04;
  if( BUTTON4 ) u8Btn += 0x08;
  if( BUTTON5 ) u8Btn += 0x10;
  if( BUTTON6 ) u8Btn += 0x20;

  u8ChangedButtons = u8BtnOld ^ u8Btn;
  
  if( u8ChangedButtons )
  {
    PIN_TOGGLE(EXT_LED_PIN);

    for(int i=1; i<=NO_OF_BUTTONS; i++)
    {
      bState = u8Btn&u8Mask;
      BtnJson[i-1]["state"] = bState ? 1 : 0;

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

        //if(MqttIsEnabled() )
        //{
        //  Serial.println("  - Send to MQTT Broker");
        //  mqtt_send_btn_state(i, bState);
        //}
      }
      u8Mask = u8Mask << 1;
    }

    if( MqttIsEnabled() )
    {
      mqtt.publish(MqttBtnTopic, JSON.stringify(BtnJson).c_str());
      Serial.println("  - Send by MQTT");
    }

    if( AdsIsEnabled() )
    {
      Serial.println("  - Send via ADS");
      VarName = std::string(SettingsJson["PlcButtonVar"]);
      u16Btn = u8Btn;
      Ads.WritePlcVarByName(VarName, &u16Btn, sizeof(uint16_t));
    }

    Serial.println("  - Send to website");
    events.send(JSON.stringify(BtnJson).c_str(), "Buttons", millis());

    delay(10); // suppress bouncing button
  }
  

  u8BtnOld = u8Btn;

}


/*

  [
    {
      "RGB": 255
    },
    {
      "RGB": 65280
    },
    {
      "RGB": 16711680
    },
    {
      "RGB": 255
    },
    {
      "RGB": 65280
    },
    {
      "RGB": 16711680
    }
  ]


*/
