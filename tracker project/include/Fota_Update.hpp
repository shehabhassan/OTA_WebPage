#ifndef FotaUpdate_HPP
#define FotaUpdate_HPP
#include <Arduino.h>

bool fetchJSONOverGSM(const char* url, String* response);
bool parseJSONResponse(const String& response, char* outSSID, char* outPassword, bool* updateFlag);
// Connect to Wi-Fi
void connectToWiFi(const char* ssid, const char* password);
bool disableUpdateFlagOverGSM(const char* url);
bool downloadHexFile(const char* url, String* hexFile);
// Parse and flash HEX file
bool parseHexAndFlash(const String& hexData);
#endif