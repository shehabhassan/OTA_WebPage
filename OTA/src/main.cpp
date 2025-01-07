#include <FS.h>
#include <GSMOTAUpdater.h>
#include <SPIFFS.h> // For file system
#include <HardwareSerial.h>

// Define serial pins and setup
#define RX_PIN 16
#define TX_PIN 17
HardwareSerial SerialAT(1);

// Define server details
const char* serverAddress = "shehabhassan.github.io";
const char* downloadPath = "/FOTA_TEST/firmware.bin";
const int serverPort = 443; // HTTPS port
const unsigned long fileSize = 102400; // Adjust according to your firmware size

GSMOTAUpdater otaUpdater;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    SerialAT.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    // Initialize SPIFFS (file system)
    if (!SPIFFS.begin(true)) {
        Serial.println("An error occurred while mounting SPIFFS");
        return;
    }

    // Initialize OTA updater
    otaUpdater.init(serverAddress, serverPort, downloadPath, fileSize, &SerialAT, &SPIFFS);

    // Download firmware
    if (otaUpdater.download("/firmware.bin")) {
        Serial.println("Firmware downloaded successfully!");

        // Perform the update
        if (otaUpdater.performUpdate("/firmware.bin")) {
            Serial.println("Firmware updated successfully! Restarting...");
            ESP.restart(); // Restart to apply the firmware
        } else {
            Serial.println("Firmware update failed!");
        }
    } else {
        Serial.println("Firmware download failed!");
    }
}

void loop() {
    // Your main application code
}
