#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_partition.h"

#include "../include/Fota_Update.hpp"
// GSM Serial
#define SerialAT Serial1  // Use Serial1 for GSM communication

// Wi-Fi Credentials (default, will be replaced by server response)
char ssid[32] = "default-SSID";
char password[32] = "default-PASSWORD";

// Server URLs
const char* gsmServerURL = "https://example.com/get-fota-details"; // URL for JSON response via GSM
const char* firmwareURL = "https://example.com/firmware.hex";      // URL for HEX file via Wi-Fi

// Flags and buffers
String jsonResponse = "";  // To store JSON response
String hexData = "";       // To store HEX file content

// Connect to GSM and fetch JSON data
bool fetchJSONOverGSM(const char* url, String* response) 
{
  SerialAT.println("AT+HTTPINIT");
  delay(1000);
  SerialAT.println(String("AT+HTTPPARA=\"URL\",\"") + url + "\"");
  delay(1000);
  SerialAT.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(1000);
  SerialAT.println("AT+HTTPACTION=0");  // HTTP GET
  delay(10000);

  if (SerialAT.find("+HTTPACTION: 0,200")) 
  {
    SerialAT.println("AT+HTTPREAD");
    delay(1000);
    *response = SerialAT.readString();
    Serial.println("GSM JSON Response: " + *response);
    return true;
  } 
  else 
  {
    Serial.println("Failed to fetch JSON over GSM.");
    return false;
  }
}

// Parse JSON response
bool parseJSONResponse(const String& response, char* outSSID, char* outPassword, bool* updateFlag) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    Serial.println("Failed to parse JSON response.");
    return false;
  }

  const char* serial = doc["serial"];
  const char* wifi = doc["wifi"];
  const char* pass = doc["password"];
  *updateFlag = doc["update_flag"];

  strncpy(outSSID, wifi, 32);
  strncpy(outPassword, pass, 32);

  Serial.printf("Parsed JSON:\nSerial: %s\nWi-Fi: %s\nPassword: %s\nUpdate Flag: %d\n", serial, wifi, pass, *updateFlag);
  return true;
}

// Connect to Wi-Fi
void connectToWiFi(const char* ssid, const char* password) {
  Serial.printf("Connecting to Wi-Fi: %s\n", ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
}

// Disable Update Flag (optional step to communicate back to the server)
bool disableUpdateFlagOverGSM(const char* url) {
  SerialAT.println("AT+HTTPINIT");
  delay(1000);
  SerialAT.println(String("AT+HTTPPARA=\"URL\",\"") + url + "\"");
  delay(1000);
  SerialAT.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(1000);
  SerialAT.println("AT+HTTPDATA=20,10000");
  delay(1000);
  SerialAT.println("{\"update_flag\":false}");
  delay(1000);
  SerialAT.println("AT+HTTPACTION=1");  // HTTP POST
  delay(10000);

  if (SerialAT.find("+HTTPACTION: 1,200")) {
    Serial.println("Update flag disabled successfully.");
    return true;
  } else {
    Serial.println("Failed to disable update flag.");
    return false;
  }
}

// Download HEX file
bool downloadHexFile(const char* url, String* hexFile) {
  HTTPClient http;
  Serial.printf("Connecting to %s\n", url);
  http.begin(url);

  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {
    *hexFile = http.getString();
    Serial.println("HEX file downloaded successfully.");
    http.end();
    return true;
  } else {
    Serial.printf("HTTP GET failed. Response code: %d\n", httpResponseCode);
    http.end();
    return false;
  }
}

// Parse and flash HEX file
bool parseHexAndFlash(const String& hexData) {
  Serial.println("Parsing HEX file...");
  
  const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  if (!partition) {
    Serial.println("Failed to find OTA partition!");
    return false;
  }

  esp_partition_erase_range(partition, 0, partition->size);

  int lineNumber = 0;
  int baseAddress = 0;
  for (int i = 0; i < hexData.length();) {
    lineNumber++;
    
    int lineStart = i;
    while (i < hexData.length() && hexData[i] != '\n') i++;
    String line = hexData.substring(lineStart, i);
    line.trim();  // Correctly trim the line
    i++;

    if (line[0] != ':') {
      Serial.printf("Invalid HEX line: %s\n", line.c_str());
      return false;
    }

    int byteCount = strtol(line.substring(1, 3).c_str(), NULL, 16);
    int address = strtol(line.substring(3, 7).c_str(), NULL, 16) + baseAddress;
    int recordType = strtol(line.substring(7, 9).c_str(), NULL, 16);

    if (recordType == 0x00) {
      String data = line.substring(9, 9 + byteCount * 2);
      uint8_t dataBytes[byteCount];
      for (int j = 0; j < byteCount; j++) {
        dataBytes[j] = strtol(data.substring(j * 2, j * 2 + 2).c_str(), NULL, 16);
      }

      esp_partition_write(partition, address, dataBytes, byteCount);

    } else if (recordType == 0x01) {
      Serial.println("End of HEX file.");
      break;

    } else if (recordType == 0x04) {
      baseAddress = strtol(line.substring(9, 13).c_str(), NULL, 16) << 16;

    } else {
      Serial.printf("Unsupported record type: %02X\n", recordType);
      return false;
    }
  }

  Serial.println("HEX file applied successfully!");
  return true;
}

