#include <Arduino.h>
#include <ETH.h>
//#include <WebServer.h> 
#include <PubSubClient.h>
#include <ElegantOTA.h> // ip/update
#include <EspAdsLib.h>
#include <Arduino_JSON.h>
#include <LittleFS.h>

#include <ESPAsyncWebServer.h>


#include "config.html"
#include "index.html"


#define ETH_ADDR        1
#define ETH_POWER_PIN   16//-1 //16 // Do not use it, it can cause conflict during the software reset.
#define ETH_POWER_PIN_ALTERNATIVE 16 //17
#define ETH_MDC_PIN    23
#define ETH_MDIO_PIN   18
#define ETH_TYPE       ETH_PHY_LAN8720

void ReadButtons(void);
void UpdateLeds(void);

AsyncWebServer  server(80); // Declare the WebServer object
AsyncEventSource events("/events");

uint16_t cnt = 23;

// MQTT
const char* mqtt_server = "mosquitto.fritz.box";
WiFiClient    ethClient;
PubSubClient  mqtt(ethClient);

// ADS
ADS::AmsAddr SrcAmsAddr((char*)"192.168.0.203.1.1", 43609);
ADS::AmsAddr DestAmsAddr((char*)"5.16.3.178.1.1", AMSPORT_R0_PLC_TC3);
char DestIpAddr[] = "192.168.0.3"; // IP of the TwinCAT target machine
ADS::Ads Ads(&SrcAmsAddr, &DestAmsAddr, DestIpAddr);

// Inputs/Outputs
#define EXT_LED_PIN 2
#define EXT_SWITCH_PIN 4
#define PIN_TOGGLE(p) digitalWrite(p, !digitalRead(p));

// Website
String sSettings;
JSONVar SettingsJson;
const static char* settingsfile    = "/settings";
#define DEFAULT_SETTINGS "{\"server\": \"192.168.0.82\", \"port\": \"1883\", \"user\": \"\", \"pass\": \"\", \"topic\": \"LS111/Metering\", \"key\": \"ElectricalPower\", \"power\" : \"2000\", \"time\": \"300\"}"
String load_from_file(const char* file_name, String defaultvalue) ;
File         this_file;



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




/*
void ConfigPage(void)
{
    String sJsonTxData =  "<script> var CurrentValues = '" + String(sSettings) + "'; </script>";
    Serial.print("Current settings: ");
    Serial.println(sSettings);
    server.send(200, "text/html", sJsonTxData+ String(sConfigPage));
}
*/

String processor(const String& var)
{
  //Serial.println(var);
  if(var == "SW1")
    return String(cnt);
  else
    return String("foo");
}

void setup()
{
  // Inputs and Outputs
  pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
  digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
  pinMode(EXT_LED_PIN, OUTPUT);
  pinMode(EXT_SWITCH_PIN, INPUT_PULLUP);

  // Serial
  Serial.begin(115200);
  delay(2500);
  Serial.println("hello");

  // Load settings from filesystem
  sSettings = load_from_file(settingsfile,  DEFAULT_SETTINGS);
  SettingsJson = JSON.parse(sSettings);

  Serial.println("Current settings:");
  Serial.println(sSettings);

  // Ethernet
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // Enable ETH
  //ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP

  while(!((uint32_t)ETH.localIP())){}; // Waiting for IP (leave this line group to get IP via DHCP)

  // Config Webserver

 server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", sIndexPage, processor);
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(&events);

  // OTA
  ElegantOTA.begin(&server);    // Start ElegantOTA

  server.begin(); // Start server
  Serial.println("Web server started");

  // Config ADS
  Ads.Connect();

  // Config MQTT
  mqtt.setServer(mqtt_server, 1883);
  mqtt.connect("ESP32 ETH01");
}

long lastMsg = 0;
char text[128];

uint16_t u16Led;
uint16_t u16LedOld;

void loop()
{
  long now = millis();
  ElegantOTA.loop();

  ReadButtons();
  UpdateLeds();

  if (now - lastMsg > 1000) 
  {
    cnt++;
    sprintf(text, "%i", cnt);
    //mqtt.publish("trash/ETH01", text);
    lastMsg = now;
    Serial.println(text);
    Ads.ReadPlcVarByName((char*)"Main.u16LED", &u16Led, sizeof(uint16_t));
   
    //PIN_TOGGLE(EXT_LED_PIN);
  }
}

bool bSwitchOld = true;


void UpdateLeds(void)
{
  JSONVar LedJson;

  if( u16Led != u16LedOld ) // Leds have changed
  {
    if( u16Led )
      digitalWrite(EXT_LED_PIN, 1);
    else
      digitalWrite(EXT_LED_PIN, 0);

    LedJson["LED"][0] = u16Led != 0 ? 1 : 0; 
    LedJson["LED"][1] = 0;
    LedJson["LED"][2] = 0;
    LedJson["LED"][3] = 0;
    LedJson["LED"][4] = 0;
    LedJson["LED"][5] = 0; 

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
 
      events.send(JSON.stringify(BtnJson).c_str(), "Buttons",millis());

      Ads.WritePlcVarByName("Main.u16Button", &u16OutVal, sizeof(uint16_t));
    }
    bSwitchOld = bSwitch;
  }

}

