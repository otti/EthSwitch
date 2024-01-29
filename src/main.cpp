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

#define ETH_ADDR        1
#define ETH_POWER_PIN   16//-1 //16 // Do not use it, it can cause conflict during the software reset.
#define ETH_POWER_PIN_ALTERNATIVE 16 //17
#define ETH_MDC_PIN    23
#define ETH_MDIO_PIN   18
#define ETH_TYPE       ETH_PHY_LAN8720

#define FORMAT_LITTLEFS_IF_FAILED      true

void ReadButtons(void);
void UpdateLeds(void);

AsyncWebServer  server(80); // Declare the WebServer object
AsyncEventSource events("/events");

// MQTT
WiFiClient    ethClient;
PubSubClient  mqtt(ethClient);

// ADS
ADS::AmsAddr SrcAmsAddr((char*)"1.1.1.1.1.1", 43609);
ADS::AmsAddr DestAmsAddr((char*)"1.1.1.1.1.1", AMSPORT_R0_PLC_TC3);
ADS::Ads Ads;

// Inputs/Outputs
#define EXT_LED_PIN 2
#define EXT_SWITCH_PIN 4
#define PIN_TOGGLE(p) digitalWrite(p, !digitalRead(p));

// Website
String sSettings;
JSONVar SettingsJson;
const static char* settingsfile    = "/settings";
#define DEFAULT_SETTINGS "{\"server\": \"192.168.0.82\", \"port\": \"1883\", \"user\": \"\", \"pass\": \"\", \"topic\": \"trash\", \"PlcIp\" : \"192.168.0.3\", \"PlcAmsAddr\" : \"5.16.3.178.1.1\", \"PlcLedVar\": \"Main.u16LED\", \"PlcButtonVar\": \"Main.u16Button\" }"
String load_from_file(const char* file_name, String defaultvalue) ;
File         this_file;

#define NEO_PIXEL_PIN 15
#define NEO_PIXEL_COUNT 24
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_RGB + NEO_KHZ800);

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

bool write_to_file(const char* file_name, String contents) {
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

void setup()
{

  String PlcIp;
  String ScrNetId;
  String DestNetId;
  String IP;


  // Inputs and Outputs
  pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
  digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
  pinMode(EXT_LED_PIN, OUTPUT);
  pinMode(EXT_SWITCH_PIN, INPUT_PULLUP);

  // Serial
  Serial.begin(115200);
  delay(2500); // The serial monitor needs some time to start up after programming
  Serial.println("hello");
  
  // Little FS
  LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);

  // Load settings from filesystem
  sSettings = load_from_file(settingsfile,  DEFAULT_SETTINGS);
  SettingsJson = JSON.parse(sSettings);

  Serial.println((const char*)SettingsJson["PlcIp"]);

  Serial.println("Current settings:");
  Serial.println(sSettings);

  // Ethernet
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // Enable ETH
  //ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP

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

  strip.begin();
  strip.setBrightness(50);             // set the maximum LED intensity down to 50

  
  
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
        if( mqtt.connect("ESP32 ETH01", (const char*)SettingsJson["user"], (const char*)SettingsJson["pass"]) )
        {
            Serial.println("  Mqtt connected");            
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

long lastMsg = 0;
char text[128];

uint16_t u16Led;
uint16_t u16LedOld = 0xFFFF;
std::string VarName;

void loop()
{
  long now = millis();
  ElegantOTA.loop();

  ReadButtons();
  UpdateLeds();
  if( MqttIsEnabled() )
    MqttReconnect();

  if (now - lastMsg > 1000) 
  {
    if( AdsIsEnabled() )
    {
      VarName = std::string(SettingsJson["PlcLedVar"]);
      Ads.ReadPlcVarByName(VarName, &u16Led, sizeof(uint16_t));
      Serial.println("Read Led from PLC: " + String(u16Led));
    }

    lastMsg = now;
  }
}

bool bSwitchOld = true;


void UpdateLeds(void)
{
  JSONVar LedJson;
  String topic;

  if( u16Led != u16LedOld ) // Leds have changed
  {
    if( u16Led&0x01 )
      digitalWrite(EXT_LED_PIN, 1);
    else
      digitalWrite(EXT_LED_PIN, 0);

    for(int i=0; i<6; i++ )
    {
      if( u16Led&(0x01<<i) )
      {
        LedJson["LED"][i] = 1; 
        strip.setPixelColor(i, 0, 50, 0);
      }
      else
      {
        LedJson["LED"][i] = 0; 
        strip.setPixelColor(i, 0, 0, 0);
      }
    }

    strip.show();
    Serial.println("LED has changed:");
   
    if(MqttIsEnabled() )
    {
      topic = SettingsJson["topic"];
      topic.replace("\"", "");
      topic = topic + "/GetLed";
      Serial.println("  - Send to mqtt (Topic: " + topic + ")");
      mqtt.publish(topic.c_str(), JSON.stringify(LedJson).c_str());
    }

    Serial.println("  - Send to website");
    events.send(JSON.stringify(LedJson).c_str(), "Leds",millis());

  }
  u16LedOld = u16Led;
}

long lastBtnRead = 0;
void ReadButtons(void)
{
  bool bSwitch;
  uint16_t u16OutVal;
  JSONVar BtnJson;
  long now = millis();
  String topic;
  std::string VarName;

  bSwitch = !digitalRead(EXT_SWITCH_PIN); // switch pressed = low level = false

  if (now - lastBtnRead > 10) // every 10 ms
  {
    lastBtnRead = now;
    if( bSwitch != bSwitchOld ) // button has changed?
    {
      //PIN_TOGGLE(EXT_LED_PIN);
      if( bSwitch )
        u16OutVal = 1;
      else
        u16OutVal = 0;

      BtnJson["BTN"][0] = u16OutVal; 
      BtnJson["BTN"][1] = 0;
      BtnJson["BTN"][2] = 0;
      BtnJson["BTN"][3] = 0;
      BtnJson["BTN"][4] = 0;
      BtnJson["BTN"][5] = 0; 

      Serial.println("Button pressed:");
      Serial.println("  - Send to website");
      events.send(JSON.stringify(BtnJson).c_str(), "Buttons", millis());

      if(MqttIsEnabled() )
      {
        topic = SettingsJson["topic"];
        topic.replace("\"", "");
        topic = topic + "/GetButton";
        Serial.println("  - Send to mqtt (Topic: " + topic + ")");
        mqtt.publish(topic.c_str(), JSON.stringify(BtnJson).c_str());
      }

      if( AdsIsEnabled() )
      {
        Serial.println("  - Send to PLC");
        VarName = std::string(SettingsJson["PlcButtonVar"]);
        Ads.WritePlcVarByName(VarName, &u16OutVal, sizeof(uint16_t));
      }
    }
    bSwitchOld = bSwitch;
  }

}

