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

#define NO_OF_BUTTONS 6

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

#define NEO_PIXEL_COUNT           1

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
const static char* settingsfile    = "/settings";
#define DEFAULT_SETTINGS "{\"server\": \"192.168.0.82\", \"port\": \"1883\", \"user\": \"\", \"pass\": \"\", \"topic\": \"trash\", \"PlcIp\" : \"192.168.0.3\", \"PlcAmsAddr\" : \"5.16.3.178.1.1\", \"PlcLedVar\": \"Main.u16LED\", \"PlcButtonVar\": \"Main.u16Button\" }"
String load_from_file(const char* file_name, String defaultvalue) ;
File         this_file;
bool bNewConnection = false;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_RGB + NEO_KHZ800);

#define MQTT_AUTODISCOVERY_TOPIC "homeassistant"

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

void send_mqtt_auto_discovery(void)
{
  char topic[128];
  char msg[128];

  if( MqttIsEnabled() )
  {
    for(int i=1; i<=NO_OF_BUTTONS; i++ )
    {
      sprintf(msg, "{\"name\": \"Eingang_CH%i\", \"state_topic\": \"%s/binary_sensor/Eingang_CH%i/state\"}", i, MQTT_AUTODISCOVERY_TOPIC, i );
      sprintf(topic, "%s/binary_sensor/Eingang_CH%i/config", MQTT_AUTODISCOVERY_TOPIC, i);
      mqtt.publish(topic, msg, true);
    }
  }
}

// u8BtnNo from 1 to NO_OF_BUTTONS
void mqtt_send_btn_state(uint8_t u8BtnNo, bool state)
{
  char topic[128];
  char msg[5];
  
  sprintf(topic, "%s/binary_sensor/Eingang_CH%i/state", MQTT_AUTODISCOVERY_TOPIC, u8BtnNo);

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

void fade(uint16_t u16Delay)
{
    int i;
    for(i=0; i<255; i++ )
    {
      strip.setPixelColor(0, 0, i, 0);
      strip.show();
      delay(u16Delay);
    }

    for(i=255; i>=0; --i )
    {
      strip.setPixelColor(0, 0, i, 0);
      strip.show();
      delay(u16Delay);
    }

    for(i=0; i<255; i++ )
    {
      strip.setPixelColor(0, 0, 0, i);
      strip.show();
      delay(u16Delay);
    }

    for(i=255; i>=0; --i )
    {
      strip.setPixelColor(0, 0, 0, i);
      strip.show();
      delay(u16Delay);
    }

    for(i=0; i<255; i++ )
    {
      strip.setPixelColor(0, i, 0, 0);
      strip.show();
      delay(u16Delay);
    }

    for(i=255; i>=0; --i )
    {
      strip.setPixelColor(0, i, 0, 0);
      strip.show();
      delay(u16Delay);
    }
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

  for(int i=0; i<10; i++)
  {
    PIN_TOGGLE(EXT_LED_PIN);
    delay(150);
  }

  fade(2);


  // Ethernet
  ETH.begin(ETH_ADDR, ETH_RESET, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLOCK_GPIO17_OUT); // Enable ETH
  ETH.setHostname("EthSwitch");
  //ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP
  Serial.println("Link Speed: ");
  Serial.print(ETH.linkSpeed());
  Serial.println("ETH.begin()");

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
            
            send_mqtt_auto_discovery();            
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

  ReadButtons();
  UpdateLeds();

  ElegantOTA.loop();

  if( MqttIsEnabled() )
  {
    mqtt.loop();
    MqttReconnect();
  }


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
  JSONVar LedJson;
  String topic;
  long now = millis();

  if( u16Led != u16LedOld || bNewConnection ) // Leds have changed
  {
    bNewConnection = false;
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
uint8_t u8BtnOld = 0xFF;
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
      BtnJson["BTN"][i-1] = bState ? 1 : 0;

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

        if(MqttIsEnabled() )
        {
          Serial.println("  - Send to MQTT Broker");
          mqtt_send_btn_state(i, bState);
        }

      }
      u8Mask = u8Mask << 1;
    }

    Serial.println(JSON.stringify(BtnJson).c_str());

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


/*
  
  uint16_t u16OutVal;
  JSONVar BtnJson;
  long now = millis();
  String topic;
  std::string VarName;



  if (now - lastBtnRead > 10) // every 10 ms
  {
    lastBtnRead = now;
    if( bSwitch != bSwitchOld ) // button has changed?
    {
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
      #if ENABLE_ETHERNET == 1
      Serial.println("  - Send to website");
      events.send(JSON.stringify(BtnJson).c_str(), "Buttons", millis());

//      if(MqttIsEnabled() )
//      {
//        topic = SettingsJson["topic"];
//        topic.replace("\"", "");
//        topic = topic + "/GetButton";
//        Serial.println("  - Send to mqtt (Topic: " + topic + ")");
//        mqtt.publish(topic.c_str(), JSON.stringify(BtnJson).c_str());
//      }

      if( AdsIsEnabled() )
      {
        Serial.println("  - Send to PLC");
        VarName = std::string(SettingsJson["PlcButtonVar"]);
        Ads.WritePlcVarByName(VarName, &u16OutVal, sizeof(uint16_t));
      }
      #endif // ENABLE_ETHERNET
    }
    bSwitchOld = bSwitch;
  }
*/

}

