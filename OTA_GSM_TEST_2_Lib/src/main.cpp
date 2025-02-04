// Device not found, scanning again
#define UPDATE_OTA_AT_COMMAND
// #define TINY_GSM_MODEM_SIM800
// #define AT_DEBUG_GSM_LIB

#ifdef UPDATE_OTA_GSM_LIB
#include <Arduino.h>
#include <Update.h>
#include "HttpsOTAUpdate.h"
static const char *server_certificate = 
"-----BEGIN CERTIFICATE-----\n"
"MIIEyDCCA7CgAwIBAgIQDPW9BitWAvR6uFAsI8zwZjANBgkqhkiG9w0BAQsFADBh\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
"MjAeFw0yMTAzMzAwMDAwMDBaFw0zMTAzMjkyMzU5NTlaMFkxCzAJBgNVBAYTAlVT\n"
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxMzAxBgNVBAMTKkRpZ2lDZXJ0IEdsb2Jh\n"
"bCBHMiBUTFMgUlNBIFNIQTI1NiAyMDIwIENBMTCCASIwDQYJKoZIhvcNAQEBBQAD\n"
"ggEPADCCAQoCggEBAMz3EGJPprtjb+2QUlbFbSd7ehJWivH0+dbn4Y+9lavyYEEV\n"
"cNsSAPonCrVXOFt9slGTcZUOakGUWzUb+nv6u8W+JDD+Vu/E832X4xT1FE3LpxDy\n"
"FuqrIvAxIhFhaZAmunjZlx/jfWardUSVc8is/+9dCopZQ+GssjoP80j812s3wWPc\n"
"3kbW20X+fSP9kOhRBx5Ro1/tSUZUfyyIxfQTnJcVPAPooTncaQwywa8WV0yUR0J8\n"
"osicfebUTVSvQpmowQTCd5zWSOTOEeAqgJnwQ3DPP3Zr0UxJqyRewg2C/Uaoq2yT\n"
"zGJSQnWS+Jr6Xl6ysGHlHx+5fwmY6D36g39HaaECAwEAAaOCAYIwggF+MBIGA1Ud\n"
"EwEB/wQIMAYBAf8CAQAwHQYDVR0OBBYEFHSFgMBmx9833s+9KTeqAx2+7c0XMB8G\n"
"A1UdIwQYMBaAFE4iVCAYlebjbuYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAd\n"
"BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQG\n"
"CCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKG\n"
"NGh0dHA6Ly9jYWNlcnRzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RH\n"
"Mi5jcnQwQgYDVR0fBDswOTA3oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29t\n"
"L0RpZ2lDZXJ0R2xvYmFsUm9vdEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwC\n"
"ATAHBgVngQwBATAIBgZngQwBAgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG\n"
"9w0BAQsFAAOCAQEAkPFwyyiXaZd8dP3A+iZ7U6utzWX9upwGnIrXWkOH7U1MVl+t\n"
"wcW1BSAuWdH/SvWgKtiwla3JLko716f2b4gp/DA/JIS7w7d7kwcsr4drdjPtAFVS\n"
"slme5LnQ89/nD/7d+MS5EHKBCQRfz5eeLjJ1js+aWNJXMX43AYGyZm0pGrFmCW3R\n"
"bpD0ufovARTFXFZkAdl9h6g4U5+LXUZtXMYnhIHUfoyMo5tS58aI7Dd8KvvwVVo4\n"
"chDYABPPTHPbqjc1qCmBaZx2vN4Ye5DUys/vZwP9BFohFrH/6j/f3IL16/RZkiMN\n"
"JCqVJUzKoZHm1Lesh3Sz8W2jmdv51b2EQJ8HmA==\n"
"-----END CERTIFICATE-----\n";

#define SerialMon Serial
#define SerialAT  Serial2

#define DEBUG_PRINT(...) { SerialMon.print(millis()); SerialMon.print(" - "); SerialMon.println(__VA_ARGS__); }
#define DEBUG_FATAL(...) { SerialMon.print(millis()); SerialMon.print(" - FATAL: "); SerialMon.println(__VA_ARGS__); delay(1000); ESP.restart(); }

#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
#include "TinyGsmClient.h"
#include "CRC32.h"
TinyGsm modem(SerialAT);
const char server[] = "https://shehabhassan.github.io";
#define URL_LINK "https://shehabhassan.github.io/FOTA_TEST/firmware.bin"
const char resource[] = "/FOTA_TEST/firmware.bin";
const int port = 443;
const char apn[]  = "internet";
const char user[] = "";
const char pass[] = "";
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 57600
TinyGsmClient client(modem);
uint32_t   knownFileSize = 240208;  // In case server does not send it

void printPercent(uint32_t readLength, uint32_t contentLength) {
  // If we know the total length
  if (contentLength != (uint32_t)-1) {
    SerialMon.print("\r ");
    SerialMon.print((100.0 * readLength) / contentLength);
    SerialMon.print('%');
  } else {
    SerialMon.println(readLength);
  }
}

void printDeviceInfo(){
  Serial.println();
  Serial.println("--------------------------");
  Serial.println(String("Build:    ") +  __DATE__ " " __TIME__);
  Serial.println(String("Flash:    ") + ESP.getFlashChipSize() / 1024 + "K");
  Serial.println(String("ESP sdk:  ") + ESP.getSdkVersion());
  Serial.println(String("Chip rev: ") + ESP.getChipRevision());
  Serial.println(String("Free mem: ") + ESP.getFreeHeap());
  Serial.println("--------------------------");
}


void startOtaUpdate(String protocol, String host, String url, int port){

  DEBUG_PRINT("protocol : " + protocol + "host : " + host + "url : " + url + "port : " + String(port));
  DEBUG_PRINT(String("Connecting to ") + host + ":" + port);

  Client* client = NULL;
  if (protocol == "http") {
    client = new TinyGsmClient(modem);
    if (!client->connect(host.c_str(), port)) {
      DEBUG_FATAL(F("Client not connected"));
    }
  }
  else if (protocol == "https") {
    client = new TinyGsmClientSecure(modem);
    if (!client->connect(host.c_str(), port)) {
      DEBUG_FATAL(F("Client not connected"));
    }
  }
  else {
    DEBUG_FATAL(String("Unsupported protocol: ") + protocol);
  }

  DEBUG_PRINT(String("Requesting ") + url);

  client->print(String("GET ") + url + " HTTP/1.0\r\n"
               + "Host: " + host + "\r\n"
               + "Connection: keep-alive\r\n"
               + "\r\n");

  long timeout = millis();
  while (client->connected() && !client->available()) {
    if (millis() - timeout > 10000L) {
      DEBUG_FATAL("Response timeout");
    }
  }

  // Collect headers
  String md5;
  int contentLength = 0;

  while (client->available()) {
    String line = client->readStringUntil('\n');
    line.trim();
    //SerialMon.println(line);    // Uncomment this to show response headers
    line.toLowerCase();
    if (line.startsWith("content-length:")) {
      contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
    } else if (line.startsWith("x-md5:")) {
      md5 = line.substring(line.lastIndexOf(':') + 1);
    } else if (line.length() == 0) {
      break;
    }
  }
  Serial.println("contentLength : " + String(contentLength));

  if (contentLength <= 0) {
    DEBUG_FATAL("Content-Length not defined");
  }

  bool canBegin = Update.begin(contentLength);
  if (!canBegin) {
    Update.printError(SerialMon);
    DEBUG_FATAL("OTA begin failed");
  }

  Serial.println("md5:" + md5);

  if (md5.length()) {
    DEBUG_PRINT(String("Expected MD5: ") + md5);
    if(!Update.setMD5(md5.c_str())) {
      DEBUG_FATAL("Cannot set MD5");
    }
  }

  DEBUG_PRINT("Flashing...");

  // The next loop does approx. the same thing as Update.writeStream(http) or Update.write(http)

  int written = 0;
  int progress = 0;
  uint8_t buff[256];

  while (client->connected() && written < contentLength) {
    timeout = millis();
    while (client->connected() && !client->available()) {
      delay(1);
      if (millis() - timeout > 100000L) {
        DEBUG_FATAL("Timeout");
      }
    }

    int len = client->read(buff, sizeof(buff));
    if (len <= 0) continue;

    Update.write(buff, len);
    written += len;

    int newProgress = (written*100)/contentLength;
    if (newProgress - progress >= 5 || newProgress == 100) {
      progress = newProgress;
      SerialMon.print(String("\r ") + progress + "%");
    }
  }
  SerialMon.println();

  if (written != contentLength) {
    Update.printError(SerialMon);
    DEBUG_FATAL(String("Write failed. Written ") + written + " / " + contentLength + " bytes");
  }

  if (!Update.end()) {
    Update.printError(SerialMon);
    DEBUG_FATAL(F("Update not ended"));
  }

  if (!Update.isFinished()) {
    DEBUG_FATAL(F("Update not finished"));
  }

  DEBUG_PRINT("========= Update successfully completed. Rebooting. =========");
  delay(5000);
  ESP.restart();
}

String cmd_at(String atcommand, uint8_t time_out){
  SerialAT.setTimeout(time_out);
  atcommand.toUpperCase();
  atcommand = atcommand + "\r\n";
  SerialAT.print(atcommand);
  String respond = SerialAT.readString();
  respond.replace("\n\r", "");
  return respond;
}

String eraseSpaceAre(String res){
  res.replace("\n","");
  res.replace("\r","");
  
  return res;
}

String getccid(){
  String respond = cmd_at("AT+CCID", 100);
  if(respond.indexOf("OK") >= 0){
    respond = respond.substring(respond.indexOf("\n"), respond.lastIndexOf("OK"));
    respond = eraseSpaceAre(respond);
    
    return respond;
  }
  return "";
}

// openssl s_client -showcerts -connect shehabhassan.github.io:443
#define BAUD_RATE 115200

static HttpsOTAStatus_t otastatus;

void HttpEvent(HttpEvent_t *event) {
  switch (event->event_id) {
    case HTTP_EVENT_ERROR:        Serial.println("Http Event Error"); break;
    case HTTP_EVENT_ON_CONNECTED: Serial.println("Http Event On Connected"); break;
    case HTTP_EVENT_HEADER_SENT:  Serial.println("Http Event Header Sent"); break;
    case HTTP_EVENT_ON_HEADER:    Serial.printf("Http Event On Header, key=%s, value=%s\n", event->header_key, event->header_value); break;
    case HTTP_EVENT_ON_DATA:      break;
    case HTTP_EVENT_ON_FINISH:    Serial.println("Http Event On Finish"); break;
    case HTTP_EVENT_DISCONNECTED: Serial.println("Http Event Disconnected"); break;
  }
}
void setup() {
  delay(2000);
  SerialMon.begin(115200);
  delay(10);
  printDeviceInfo();
  // Set GSM module baud rate
  TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);

   // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();

  // DEBUG_PRINT(F("Starting OTA update in 5 seconds..."));
  delay(5000);
   Serial.print("Waiting for network...");
    if (!modem.waitForNetwork())
    {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

    Serial.print("Connecting to ");
    Serial.print(apn);
    if (!modem.gprsConnect(apn, user, pass))
    {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

    Serial.print("Connecting to ");
    Serial.print(server);

    // if you get a connection, report back via serial:
    if (!client.connect(server, port))
    {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

  DEBUG_PRINT(F("Checking Network..."));
  while (true){
    if(modem.isNetworkConnected()){
      break;
    }
    else{
      DEBUG_PRINT(F("Waiting for network..."));
      bool network = modem.waitForNetwork(10000L);
      Serial.println("Network : " + String(network));
      Serial.println("getccid : " + getccid());
      if (network == true) {
        break;
      }
      DEBUG_PRINT(F("Network failed to connect"));
    }
  }
  

  DEBUG_PRINT(F("Get CCID..."));
  Serial.println(getccid());

  DEBUG_PRINT(F("Get Operator..."));
  Serial.println(modem.getOperator());

  DEBUG_PRINT(F("Connecting to GPRS"));
  unsigned int i = 0;
  while(true){
    if (modem.gprsConnect("internet", "", "")==true) {
      DEBUG_PRINT(F("Connected to GPRS"));  
      // delay(1000);
      break;
    }
    Serial.print(String("\r "));
    DEBUG_PRINT(F("Not connected APN"));

    if(i>10){
      DEBUG_FATAL(F("counted until 10 times"));
    }
    
    i++;
  }
  i = 0;
  SerialMon.print(F("Connecting to "));
  SerialMon.print(server);
  if (!client.connect(server, port)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  // Make a HTTP GET request:
  client.print(String("GET ") + resource + " HTTP/1.0\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");

  // Let's see what the entire elapsed time is, from after we send the request.
  uint32_t timeElapsed = millis();

  SerialMon.println(F("Waiting for response header"));

  HttpsOTA.onHttpEvent(HttpEvent);
  Serial.println("Starting OTA");
  HttpsOTA.begin(URL_LINK, server_certificate);
  // While we are still looking for the end of the header (i.e. empty line
  // FOLLOWED by a newline), continue to read data into the buffer, parsing each
  // line (data FOLLOWED by a newline). If it takes too long to get data from
  // the client, we need to exit.

  const uint32_t clientReadTimeout   = 5000;
  uint32_t       clientReadStartTime = millis();
  String         headerBuffer;
  bool           finishedHeader = false;
  uint32_t       contentLength  = 0;

  while (!finishedHeader) {
    int nlPos;

    if (client.available()) {
      clientReadStartTime = millis();
      while (client.available()) {
        char c = client.read();
        headerBuffer += c;

        // Uncomment the lines below to see the data coming into the buffer
        if (c < 16)
        // SerialMon.print('0');
        SerialMon.print(c, DEC);
        SerialMon.print(' ');
        if (isprint(c))
          SerialMon.print(reinterpret_cast<char> (c));
        else
          // SerialMon.print('*');
        SerialMon.print(' ');

        // Let's exit and process if we find a new line
        if (headerBuffer.indexOf(F("\r\n")) >= 0) break;
      }
    } else {
      if (millis() - clientReadStartTime > clientReadTimeout) {
        // Time-out waiting for data from client
        SerialMon.println(F(">>> Client Timeout !"));
        break;
      }
    }

    // See if we have a new line.
    nlPos = headerBuffer.indexOf(F("\r\n"));

    if (nlPos > 0) {
      headerBuffer.toLowerCase();
      // Check if line contains content-length
      if (headerBuffer.startsWith(F("content-length:"))) {
        contentLength = headerBuffer.substring(headerBuffer.indexOf(':') + 1).toInt();
        SerialMon.print(F("Got Content Length: "));  // uncomment for
        SerialMon.println(contentLength);            // confirmation
      }

      headerBuffer.remove(0, nlPos + 2);  // remove the line
    } else if (nlPos == 0) {
      // if the new line is empty (i.e. "\r\n" is at the beginning of the line),
      // we are done with the header.
       SerialMon.print(F("finishedHeader : "));
       SerialMon.println(F(finishedHeader));
      finishedHeader = true;
    }
  }
  uint32_t readLength = 0;
  CRC32    crc;
 uint32_t   knownCRC32    = 0x6f50d767;
  if (finishedHeader && contentLength == knownFileSize) {
    SerialMon.println(F("Reading response data"));
    clientReadStartTime = millis();

    printPercent(readLength, contentLength);
    while (readLength < contentLength && client.connected() &&
           millis() - clientReadStartTime < clientReadTimeout) {
      while (client.available()) {
        uint8_t c = client.read();
        SerialMon.print(static_cast<char>(c));  // Uncomment this to show
        // data
        // crc.update(c);
        readLength++;
        if (readLength % (contentLength / 13) == 0) {
          printPercent(readLength, contentLength);
        }
        clientReadStartTime = millis();
      }
    }
    printPercent(readLength, contentLength);
  }

  timeElapsed = millis() - timeElapsed;
  SerialMon.println();

  // Shutdown

  client.stop();
  SerialMon.println(F("Server disconnected"));
  // startOtaUpdate("https","shehabhassan.github.io","/FOTA_TEST/firmware.bin",443);
}

void loop() {
  delay(1000);
  otastatus = HttpsOTA.status();
  if (otastatus == HTTPS_OTA_SUCCESS) {
    Serial.println("Firmware written successfully. To reboot device, call API ESP.restart() or PUSH restart button on device");
  } else if (otastatus == HTTPS_OTA_FAIL) {
    Serial.println("Firmware Upgrade Fail");
  }
  delay(1000);
}
#endif
#ifdef UPDATE_OTA_AT_COMMAND

#define TINY_GSM_MODEM_SIM800      // Modem is SIM800

#include <Arduino.h>
#include <TinyGsmClient.h>

HardwareSerial SerialAT(2);

#define URL_TEST "https://trapsysapi.epcmms.com/api/Lookups/GetAllCountries"
#define URL_git "https://shehabhassan.github.io/FOTA_TEST/firmware.bin"

TinyGsm modem(SerialAT);

#define DEBUG(...) { Serial.print(" Debug statue: "); Serial.println(__VA_ARGS__); }
#define DEBUG_FETAL(...) { Serial.print(" Debug statue: "); Serial.println(__VA_ARGS__); delay(1000); ESP.restart(); }

bool isTCPConnected = false;

String ATResponse = "";

String cmd_at(String atcommand, uint8_t time_out){
  SerialAT.setTimeout(time_out);
  atcommand.toUpperCase();
  atcommand = atcommand + "\r\n";
  SerialAT.print(atcommand);
  String respond = SerialAT.readString();
  respond.replace("\n\r", "");
  return respond;
}

String eraseSpaceAre(String res){
  res.replace("\n","");
  res.replace("\r","");
  
  return res;
}

String getccid(){
  String respond = cmd_at("AT+CCID", 100);
  if(respond.indexOf("OK") >= 0){
    respond = respond.substring(respond.indexOf("\n"), respond.lastIndexOf("OK"));
    respond = eraseSpaceAre(respond);
    return respond;
  }
  return "";
}

String serverAddress = "shehabhassan.github.io";
bool sendATCommand(String command, const int timeout);
void setup(){
  Serial.begin(115200);
  SerialAT.begin(115200);
  delay(1000);
  DEBUG("Initializing SIM800...");
  sendATCommand("AT", 1000);
  sendATCommand("AT+CPIN?", 1000);
  sendATCommand("AT+CSQ", 1000);
  sendATCommand("AT+CREG?", 1000);
  sendATCommand("AT+CGATT?", 1000);

  // sendATCommand("AT+CFUN=1", 1000); // Set full functionality 
  // sendATCommand("AT+CGATT=1", 1000); // Attach to GPRS
  // sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 1000); // Set bearer profile access point name
  // sendATCommand("AT+SAPBR=3,1,\"APN\",\"\"", 1000);// Set bearer profile access point name
  // sendATCommand("AT+SAPBR=1,1", 1000);// Open bearer
  // sendATCommand("AT+SAPBR=2,1", 1000);// Query bearer

  // DEBUG("Setting up GPRS...");
  // sendATCommand("AT+CSTT=\"\",\"\",\"\"", 1000);
  // sendATCommand("AT+CIICR", 1000);
  // bool gprsConnected = false;
  // for (int i = 0; i < 5; i++) {
  //   if (sendATCommand("AT+CSTT=\"vodafone\",\"\",\"\"", 2000) &&
  //       sendATCommand("AT+CIICR", 2000)) {
  //     gprsConnected = true;
  //     DEBUG("set up GPRS connected");
  //     break;
  //   }
  //   delay(2000);
  // }

  // if(!gprsConnected) {DEBUG("Failed to set up GPRS");}

  DEBUG(F("Checking Network..."));
  while (true){
    if(modem.isNetworkConnected()){
      break;
    }
    else{
      DEBUG(F("Waiting for network..."));
      bool network = modem.waitForNetwork(5000L);
      Serial.println("Network : " + String(network));
      Serial.println("getccid : " + getccid());
      if (network == true) {
        break;
      }
      DEBUG(F("Network failed to connect"));
    }
  }

  DEBUG(F("Get CCID..."));
  Serial.println(getccid());

  DEBUG(F("Get Operator..."));
  Serial.println(modem.getOperator());

  DEBUG(F("Connecting to GPRS"));
  unsigned int i = 0;
  while(true){
    if (modem.gprsConnect("internet", "", "")==true) {
      DEBUG(F("Connected to GPRS"));  
      // delay(1000);
      break;
    }
    Serial.print(String("\r "));
    DEBUG(F("Not connected APN"));

    if(i>10){
      DEBUG(F("counted until 10 times"));
    }
    i++;
  }
  sendATCommand("AT+CIFSR", 1000);

  // sendATCommand("AT+CIPSTART=\"TCP\",\"shehabhassan.github.io\",443", 10000);
  // sendATCommand("AT+CIPSEND", 1000);
  // sendATCommand("GET /FOTA_TEST/firmware.bin HTTP/1.0\r\nHost: shehabhassan.github.io\r\n\r\n", 1000);
  // sendATCommand("AT+CIPCLOSE", 1000);
  // DEBUG("OTA update completed.");
  // if (sendATCommand(("AT+CIPSTART=\"TCP\",\"" + String(serverAddress) + "\",443").c_str(),10000))
		// {
			// Waiting for TCP to connect
			// while (isTCPConnected == false)
			// {
				// while (SerialAT.available() > 0)
				// {
				// 	char c = SerialAT.read();
				// 	ATResponse += c;
        //   if(ATResponse.length() == 0){
        //     isTCPConnected = true;
        //     return;
        //   }else{
        //     return;
        //   }
				// }
			// }
    // }
  // DEBUG(ATResponse);

  DEBUG("Configuring HTTPS...");
  sendATCommand("AT+HTTPINIT", 1000);
  sendATCommand("AT+HTTPPARA=\"CID\",1", 1000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"https://trapsysapi.epcmms.com/api/Lookups/GetAllCategories", 5000);
  sendATCommand("AT+HTTPSSL=0", 1000);  // Disable HTTPS

  DEBUG("Sending HTTPS GET request...");
  sendATCommand("AT+HTTPACTION=0", 10000);

  DEBUG("Reading response...");
  sendATCommand("AT+HTTPREAD", 10000);

  DEBUG("Terminating HTTPS...");
  sendATCommand("AT+HTTPTERM", 1000);
}

void loop(){}
//  SerialAT.println(command);
//   unsigned long start = millis();
//   String ATResponse="";
//   while (millis() - start < timeout) {
//     if (SerialAT.available()) {
//       ATResponse += (char)SerialAT.read();   
//     }    
//  }
//   if (ATResponse.length() == 0) {
//     Serial.println("No response from modem");
//   }else {
//     Serial.println(ATResponse);
//   }

bool sendATCommand(String command, const int timeout){
String response = "";
  DEBUG("Sending: " + command);
  SerialAT.println(command);  // Send the command
  unsigned long previousMillis = millis();
  while (millis() - previousMillis < timeout) {
    if (SerialAT.available()) {
      response += (char)SerialAT.read();
    }
  }
    if (response.length() > 0) {
    Serial.println(response);
    return 1;
  }else {
    Serial.println("No response from modem");
  return 0;
  }
}
  // if (response.length() > 0) {
  //   DEBUG("Response: " + response);
  //   return 1;
  // } else {
  //   DEBUG("No response or timeout.");
  //   return 0;
  // }
#endif

#ifdef AT_DEBUG_GSM_LIB

// Select your modem:
#define TINY_GSM_MODEM_SIM800

// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial

#define SerialAT Serial2

#define TINY_GSM_DEBUG SerialMon

#include <TinyGsmClient.h>

// Module baud rate
uint32_t rate = 0;  // Set to 0 for Auto-Detect

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(6000);
}

void loop() {
  if (!rate) { rate = TinyGsmAutoBaud(SerialAT); }

  if (!rate) {
    SerialMon.println(
        F("***********************************************************"));
    SerialMon.println(F(" Module does not respond!"));
    SerialMon.println(F("   Check your Serial wiring"));
    SerialMon.println(
        F("   Check the module is correctly powered and turned on"));
    SerialMon.println(
        F("***********************************************************"));
    delay(30000L);
    return;
  }

  SerialAT.begin(rate);

  // Access AT commands from Serial Monitor
  SerialMon.println(
      F("***********************************************************"));
  SerialMon.println(F(" You can now send AT commands"));
  SerialMon.println(
      F(" Enter \"AT\" (without quotes), and you should see \"OK\""));
  SerialMon.println(
      F(" If it doesn't work, select \"Both NL & CR\" in Serial Monitor"));
  SerialMon.println(
      F("***********************************************************"));

  while (true) {
    if (SerialAT.available()) { SerialMon.write(SerialAT.read()); }
    if (SerialMon.available()) { SerialAT.write(SerialMon.read()); }
    delay(0);
  }
}
#endif