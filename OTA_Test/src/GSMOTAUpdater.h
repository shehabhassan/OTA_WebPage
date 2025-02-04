
#ifndef GSMOTAUPDATER_H
#define GSMOTAUPDATER_H

#define TINY_GSM_MODEM_SIM800 
// init file system variable
#define FORMAT_SPIFFS_IF_FAILED true

     // Modem is SIM800
#define URL_TEST "https://trapsysapi.epcmms.com/api/Lookups/GetAllCountries"
#define URL_git "https://shehabhassan.github.io/FOTA_TEST/firmware.bin"

#include <Arduino.h>
#include <TinyGsmClient.h>

#include <FS.h>
#include "SPIFFS.h"

#include <Update.h>
#include <MD5Builder.h>

#define DEBUG 1

#define UART_SERIAL 2

extern fs::FS *fileSystem;
extern HardwareSerial SerialAT;
extern bool isInitialized;
extern bool isTCPConnected;
extern bool wasConnectionLost;
extern bool waitingForData;
extern bool isHeadersRead;
extern bool chunkDownloaded;
extern bool isDownloadComplete;
extern unsigned long fileSize;
extern unsigned long currentChunkByte;
extern unsigned long currentByte;
extern unsigned long rangeStart;
extern unsigned long rangeEnd;
extern String serverAddress;
extern int serverPort;
extern String downloadPath;


// Function prototypes
void GSMOTAUpdater_init(int Buad_rate);
bool downloadFile(const char* url, int timeout); 
bool connectGPRS(int timeout);
#endif

#if defined ESP32
	
	#define DEBUG_(...) { Serial.print(" Debug statue: "); Serial.println(__VA_ARGS__); }
	#define DEBUG_FETAL_(...) { Serial.print(" Debug statue: "); Serial.println(__VA_ARGS__); delay(1000); ESP.restart(); }

	#define HEAP_AVAILABLE() ESP.getFreeHeap()

	#ifdef ESP32
		#define GOTA_LOG_FORMAT(letter, format)  "[" #letter "][%s:%u][H:%u] %s(): " format "\r\n", __FILE__, __LINE__, HEAP_AVAILABLE(), __FUNCTION__

		#if defined DEBUG_ESP_PORT
			#define gota_log_d(format, ...) DEBUG_ESP_PORT.printf(GOTA_LOG_FORMAT(N, format), ##__VA_ARGS__);
			#define gota_log_e(format, ...) DEBUG_ESP_PORT.printf(GOTA_LOG_FORMAT(E, format), ##__VA_ARGS__);

		#else
			#define gota_log_d(format, ...) Serial.printf(GOTA_LOG_FORMAT(N, format), ##__VA_ARGS__);
			#define gota_log_e(format, ...) Serial.printf(GOTA_LOG_FORMAT(E, format), ##__VA_ARGS__);
		#endif
	#endif
#endif