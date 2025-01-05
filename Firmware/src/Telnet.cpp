#include "Telnet.h"

ESPTelnet telnet;
EscapeCodes ansi;
#define TELNET_PORT 23

void setupTelnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onTelnetInput);

  Serial.print("  - Telnet: ");
  if (telnet.begin(TELNET_PORT, false)) {
    Serial.println("running");
  } else {
    Serial.println("error.");
  }
}

// (optional) callback functions for telnet events
void onTelnetConnect(String ip)
{
  Serial.print("  - Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  
  telnet.print(ansi.cls());
  telnet.print(ansi.home());
  telnet.print(ansi.setFG(ANSI_BRIGHT_WHITE));
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("enter \"bye\" to disconnect.)");
  telnet.print(ansi.reset());
}

void onTelnetDisconnect(String ip)
{
  Serial.println("  - Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
}

void onTelnetReconnect(String ip)
{
  Serial.print("  - Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip)
{
  Serial.print("  - Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connected");
}

void onTelnetInput(String str) {
  // checks for a certain command
 if (str == "bye")
 {
    telnet.print(ansi.setFG(ANSI_BRIGHT_WHITE));
    telnet.println("> disconnecting you...");
    telnet.print(ansi.reset());
    telnet.disconnectClient();
    }
}