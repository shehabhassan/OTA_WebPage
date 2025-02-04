#include "GSMOTAUpdater.h"




int chunkSize = 25000; // bytes
char md5Hash[33] = ""; // md5 hash for verification

bool isInitialized = false;
bool isTCPConnected = false;
bool wasConnectionLost = false;
bool waitingForData = true;
bool isHeadersRead = false;
bool chunkDownloaded = false;
bool isDownloadComplete = false;
unsigned long fileSize = 0;
unsigned long currentChunkByte = 0;
unsigned long currentByte = 0;
unsigned long rangeStart = 0;
unsigned long rangeEnd = 0;
int serverPort = 443;

HardwareSerial SerialAT(UART_SERIAL);

TinyGsm modem(SerialAT);

static String cmd_at(String atcommand, uint8_t time_out){
  SerialAT.setTimeout(time_out);
  atcommand.toUpperCase();
  atcommand += "\r\n";
  SerialAT.print(atcommand);
  String respond = SerialAT.readString();
  respond.replace("\n","");
  respond.replace("\r","");
  return respond;
}

static String getccid(){
  String respond = cmd_at("AT+CCID", 100);
  int okIndex = respond.indexOf("OK");
  if(okIndex >= 0){
    respond = respond.substring(respond.indexOf("\n"), respond.lastIndexOf("OK"));
    return respond;
  }
  return "";
}

/**
 * Sends an AT command to the modem and waits for the expected response.
 * @param command The AT command to send.
 * @param expectedResponse The expected response from the modem.
 */

static void sendATCommand(String command, unsigned long timeout) {
  SerialAT.println(command);
  unsigned long start = millis();
  String ATResponse="";
  while (millis() - start < timeout) {
    if (SerialAT.available()) {
      ATResponse += (char)SerialAT.read();   
    }    
 }
  if (ATResponse.length() == 0) {
    Serial.println("No response from modem");
  }else {
    Serial.println(ATResponse);
  }
}
/**
 * initialize the SIM800L module
 */
// Function to initialize the SIM800L module
static void initSIM800L() {
  sendATCommand("AT", 1000);
  sendATCommand("AT+CPIN?", 1000); // Check if SIM card is inserted
  sendATCommand("AT+CSQ", 1000);   // Check signal strength
  sendATCommand("AT+CREG?", 1000); // Check if connected to network
  sendATCommand("AT+CGATT?", 1000); // Check if connected to GPRS
}
static void SIM800L_GSM_init() {
  sendATCommand("AT+CFUN=1", 1000); // Set full functionality 
  sendATCommand("AT+CGATT=1", 1000); // Attach to GPRS
  sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 1000);
  sendATCommand("AT+SAPBR=3,1,\"APN\",\"\"", 1000);
  sendATCommand("AT+SAPBR=1,1", 1000);
  sendATCommand("AT+SAPBR=2,1", 1000);

  DEBUG_(F("Checking Network..."));
  while(true){
    if(modem.isNetworkConnected()){break;}
    else{
      DEBUG_(F("Waiting for network..."));
      bool network = modem.waitForNetwork(5000L);
      Serial.println("Network : " + String(network));
      Serial.println("getccid : " + getccid());
      if (network == true) {break;}
      DEBUG_(F("Network failed to connect"));
    }
  }
}
/**
 * Initializes the GSMOTAUpdater with the specified parameters.
 *
 * @param Buad_rate The Value of 
 * @param server_address The address of the server to connect to.
 * @param server_port The port number of the server.
 * @param download_path The path where the downloaded file will requested from server.
 * @param file_size The size of the file to be downloaded.
 * @param file_system The file system object for file operations.
 */
void GSMOTAUpdater_init(int Buad_rate) {
  SerialAT.begin(Buad_rate);
  while(!SerialAT);
  
  gota_log_d("GSMOTAUpdater initialized");
  isInitialized = true;
  initSIM800L();
  SIM800L_GSM_init();
}
/***
 * print the device information
 * note: this function is used to print the device information  
 */
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

bool connectGPRS(int timeout) {
  Serial.println("Setting up GPRS connection...");

  sendATCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"",timeout);
  sendATCommand("AT+SAPBR=3,1,\"APN\",\"internet\"",timeout); // Replace "internet" with your APN
  delay(2000); // Wait for settings to take effect
  sendATCommand("AT+SAPBR=1,1",timeout);
  delay(2000); // Wait for connection to establish
return 0;
}

bool downloadFile(const char* url, int timeout) {
  Serial.println("Downloading file from GitHub...");

  sendATCommand("AT+HTTPPARA=\"CID\",1",timeout);
  String command = "AT+HTTPPARA=\"URL\",\"" + String(url) + "\"";
  sendATCommand(command.c_str(),timeout);
  return 0;
}
