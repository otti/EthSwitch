#ifndef __TELNET_H__
#define __TELNET_H__

#include <ESPTelnet.h>
#include <EscapeCodes.h>

void onTelnetConnect(String ip);
void onTelnetDisconnect(String ip);
void onTelnetReconnect(String ip);
void onTelnetConnectionAttempt(String ip);
void onTelnetInput(String str);
void setupTelnet();

#endif // __TELNET_H__