#include <Arduino.h>
// #include <SoftwareSerial.h>

#include <ArduinoJson.h>
// Define the GSM modem model
#define TINY_GSM_MODEM_SIM7600

#include <TinyGsmClient.h>
#include "../include/sim7600_alg.hpp"

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_partition.h"

#include "../include/Fota_Update.hpp"


String received_data;
// Declare global variables 
extern const char apn[];          // Declare APN
extern const char URL[];          // Declare URL for receiving data
extern const char URL2[];         // Declare URL for sending data
 // Create JSON message
   const char test_message_send[] ="{\"serlNum\":\"EG-123-2020-098164\",\"readingDate\":\"2024-10-30\",\"readingTime\":\"14:30:00\",\"readingLat\":\"40.7128\",\"readingLng\":\"-74.0060\",\"counter\":10,\"fan\":1,\"co2\":350,\"co2Val\":\"ppm\",\"readingSmall\":\"1\",\"readingLarg\":\"100\",\"readingMosuqitoes\":\"5\",\"readingTempIn\":\"22.5\",\"readingTempOut\":\"15.0\",\"readingWindSpeed\":\"3.5\",\"readingHumidty\":\"60\",\"amb_Light\":\"300\",\"battery\":\"85%\",\"reception\":\"Strong\",\"power_Draw\":\"50W\",\"readingFly\":\"2\",\"bigBattery\":\"90%\",\"smallBattery\":\"75%\",\"isDone\":\"true\",\"isClean\":\"false\"}";
  //String HTTP_STR = "AT+HTTPPARA=\"URL\",\"http://trapsnos.epcmms.com/api/Traps/LoadData2:80\"    ";



const char* jsonChunk1 = "{\"counter\":\"1\",\"readingsmall\":\"55\",\"r";
const char* jsonChunk2 = "eadingMosquitoes\":\"55\",\"readingLarg\":\"";
const char* jsonChunk3 = "55\",\"readingFly\":\"55\",\"BigBattery";
const char* jsonChunk4 = "\":\"80\",\"SmallBattery\":\"60\",\"readingTempIn\":\"25\"";
const char* jsonChunk5 = ",\"serlNum\":\"EG-123-2020-098164\",\"readingTempOut\":";
const char* jsonChunk6 = "\"false\",\"readingHumidity\":\"55\",\"readingDate\":\"";
const char* jsonChunk7 = "2024-11-06\",\"readingTime\":\"12:30\",\"readingLat\":\"";
const char* jsonChunk8 = "0.000000\",\"readingLng\":\"0.000000\",\"readingWindSpe";
const char* jsonChunk9 = "ed\":\"15\",\"co2\":\"400\",\"co2Val\":\"350\",\"isDone\":";
const char* jsonChunk10 = "\"true\",\"isClean\":\"false\",\"model\":\"your_model_";
const char* jsonChunk11 = "value\"}";
/*******************************************************************************************************************************/

  String counter = "1";
  String small_value = "5"; // Example sensor value
  String medium_value = "3";
  String large_value = "2";
  String fly_value = "4";
  int big_battery_percent = 80;
  int small_battery_percent = 60;
  int temperature_in = 25;
  String serial_number = "EG-123-2020-098164";
  bool temperature_out = false;
  int humidity = 55;
  String date = "2024-11-06";
  String readTime = "12:30";  // Renamed to avoid conflict with `time`
  String latitude = "0.000000";
  String longitude = "0.000000";
  int wind_speed = 15;
  int co2_level = 400;
  int co2_value = 350;
  bool is_done = true;
  bool is_clean = false;
  String model_value = "your_model_value"; // Model-specific information

  // Create a JSON document
  StaticJsonDocument<1024> doc;

String hamada_String; 
  // then take this str and put it inside the 
  String command ;
  String saved_json;

/**********************************************************************************************************************/

extern HardwareSerial SerialAT;  // Use UART2 for the modem 16 ,17 
#define debug_mode true
#define FOTA                  1
#define Bluetooth             2
#define flash_size            3
#define BLE_RTOS              4
#define BLE_RTOS_Dual_core    5
#define SD_card_test          6
#define Kalman_MPU6050        7
#define kalman_MPU6050_1      8
#define MPU_RTOS              9
#define MPU_RTOS_velocity     10
#define RTC_DS1307_test       11
#define low_power_modes       12  
#define Test_json_serial      13  

#define project   BLE_RTOS_Dual_core
/**********************************************************************************************************************/
#if project == FOTA
  // this function is used for filteration the data that i received from GSM 

  void processJSONData(String* command , String *hamada_json) {
    // Print the received command
    Serial.println("Processing command as JSON:");

    // Look for JSON boundaries
    // indexof and last index of are reference in c++ to find , rfind 
    int jsonStart = command->indexOf('{'); // Find the start of JSON // return the first pos of it (let say 40)
    int jsonEnd = command->lastIndexOf('}'); // Find the end of JSON // return the last pos of it (let say 140)

    if (jsonStart != -1 && jsonEnd != -1 && jsonEnd > jsonStart)  
    {
      // Extract JSON data from the command string
      String jsonData = command->substring(jsonStart, jsonEnd + 1);
      Serial.println("Extracted JSON: " + jsonData);

      #if debug_mode == true

        // Debugging output to confirm the extraction
        for (int i = 0; i < jsonData.length(); i++) 
        {
          Serial.print("Character [");
          Serial.print(i);
          Serial.print("]: ");
          Serial.println(jsonData[i]);
        }

      #endif 

      // this used to save the data to this string 
      *hamada_json = jsonData;
    } 
    else 
    {
      Serial.println("No valid JSON data found in the command.");
    }
    Serial.print(" the hamada json inside the function : ");
    Serial.print(*hamada_json);

  }

  // Create an instance of the QuectelHTTP class
  // QuectelHTTP quectel_cmds;

      // Modify a command (example: change the PDP APN config)
      // quectel_cmds.setPDPAPNConfig("AT+QICSGP=1,1,\"new_apn\",\"\",\"\",1");
      


  extern const char* gsmServerURL;
  extern const char* firmwareURL;
  extern String jsonResponse;
  extern String hexData;
  extern char ssid[32];
  extern char password[32];
  void setup() {
    Serial.begin(115200);
    SerialAT.begin(9600, SERIAL_8N1, 16, 17);  // Configure GSM TX/RX pins

    // Step 1: Fetch JSON over GSM
    if (!fetchJSONOverGSM(gsmServerURL, &jsonResponse))
    {
      Serial.println("Failed to fetch JSON over GSM.");
      return;
    }

    // Step 2: Parse JSON
    bool updateFlag = false;
    if (!parseJSONResponse(jsonResponse, ssid, password, &updateFlag)) {
      Serial.println("Failed to parse JSON.");
      return;
    }

    if (!updateFlag) {
      Serial.println("Update flag is not set. No update will be performed.");
      return;
    }

    // Step 3: Disable update flag
    if (!disableUpdateFlagOverGSM(gsmServerURL)) {
      Serial.println("Failed to disable update flag.");
      return;
    }

    // Step 4: Connect to Wi-Fi
    connectToWiFi(ssid, password);

    // Step 5: Download HEX file over Wi-Fi
    if (!downloadHexFile(firmwareURL, &hexData)) {
      Serial.println("Failed to download HEX file.");
      return;
    }

    // Step 6: Parse and flash HEX file
    if (!parseHexAndFlash(hexData)) {
      Serial.println("Failed to parse or apply HEX file.");
      return;
    }

    // Step 7: Reboot after update
    Serial.println("Firmware updated successfully. Rebooting...");
    ESP.restart();
  }

  void loop() {
    // Do nothing
  }

#elif project == Bluetooth

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// dedine the UUIDs for the Ble service and characteristics 
#define SERVICE_UUID           "12345678-1234-1234-1234-1234567890ab" // Unique ID for the BLE service
#define CHARACTERISTIC_UUID    "12345678-1234-1234-1234-1234567890cd" // Unique ID for the BLE characteristic

BLECharacteristic *pCharacteristic; // Pointer to a BLECharacteristic object this is used to configer the conection ( read , write ....)
bool deviceConnected = false; // Flag to track connection status of the BLE client

// this is call back it is inherit form the BLEServerCallbacks used for connection and disconnection 
class MyServerCallbacks : public BLEServerCallbacks 
{

  void onConnect(BLEServer* pServer)
  {
    deviceConnected = true;  // change the state to true 
    Serial.println("Client Connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;  // change the state to disconnect 
    Serial.println("Client Disconnected");
    // Restart advertising to allow new clients to connect
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();  // Restart advertising
    Serial.println("Advertising restarted, waiting for new connections...");
  }
};

// Define characteristic callback class for handling write requests from the client

String value; // this used for received data ( string , json , .....)
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) 
  {
    // Get the value written by the client and convert it to a string
    value = pCharacteristic->getValue().c_str();
    std::string stdValue = value.c_str(); // convert it into c ctring for printing 

    // this only for debug
    Serial.print("Received data: ");
    Serial.println(stdValue.c_str());
  }
};

void setup()
{
  Serial.begin(115200);
  BLEDevice::init("عين صقر 1"); // init the BLE div and set the name 

  // Create a BLE server and set its callbacks  
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create a BLE service with the defined UUID
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic with read and write properties
  pCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID,
                       BLECharacteristic::PROPERTY_READ |
                       BLECharacteristic::PROPERTY_WRITE
                     );
  // Set the characteristic callbacks to handle client interactions                   
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();  // Start the service

  // Start advertising the BLE service to make it discoverable to clients
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  Serial.println("BLE Server started and advertising...");
}
void loop()
{
  if (deviceConnected) 
  {
    // Check for Serial input to send to BLE client
    if (Serial.available()) 
    {
      String data = Serial.readString();
      // Send data to the characteristic
      pCharacteristic->setValue(data.c_str()); // Convert String to const char*
      pCharacteristic->notify();  // Notify any connected clients with the new data
      Serial.print("Sending data: ");
      Serial.println(data);  // Print the data sent to Serial Monitor
    }

    // Respond if characteristic receives a specific value
    if (value == "ahmed") 
    {
      String take_action = "{ok i received hamada and i need to test how many bytes can send in single frame so make it easy{ ";
      pCharacteristic->setValue(take_action.c_str());
      pCharacteristic->notify();
      Serial.print("Sending data: ");
      Serial.println(take_action);
      value = ""; // Clear the value to avoid repeated action
    }
  } 
  else 
  {
    // Avoid repeated "Waiting" messages
    static unsigned long lastMessage = 0;
    if (millis() - lastMessage > 1000) 
    {
      Serial.println("Waiting for client to connect...");
      lastMessage = millis();
    }
  }
}



/*
void loop()
{
  if (deviceConnected) // for BLE
  {
    if (Serial.available()) 
    {
      String data = Serial.readString();
      // Send data to the characteristic
      pCharacteristic->setValue(data.c_str()); // Convert String to const char*
      pCharacteristic->notify();  // Notify any connected clients with the new data
      Serial.print("Sending data: ");
      Serial.println(data);  // Print the data sent to Serial Monitor
    }
  } 
  else 
  {
    // Ensure advertising is on when no device is connected
    Serial.println("Waiting for client to connect...");
    delay(1000);
  }

}

*/

#elif  project == flash_size

#include <Arduino.h>
void setup()
{
  Serial.begin(115200);
  Serial.print("the flash size : ");
  Serial.print(ESP.getFlashChipSize());
  Serial.println(" MB");

}
void loop()
{}

#elif project == BLE_RTOS 

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


// BLE Service and Characteristic UUIDs
#define SERVICE_UUID           "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID    "12345678-1234-1234-1234-1234567890cd"

// LED Pins
#define LED1_PIN 2
#define LED2_PIN 0

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String value;

// Task Handles
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t Task3Handle;

// BLE Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client Connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client Disconnected");
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    value = pCharacteristic->getValue().c_str();
    Serial.print("Received: ");
    Serial.println(value);
  }
};

// Task 1: Blink LED1 every 1 second
void Task1(void *pvParameters) {
  while (1) {
    digitalWrite(LED1_PIN, !digitalRead(LED1_PIN)); // Toggle LED
    vTaskDelay(pdMS_TO_TICKS(1000));               // Delay for 1 second
  }
}

// Task 2: Blink LED2 every 2 seconds
void Task2(void *pvParameters) {
  while (1) {
    digitalWrite(LED2_PIN, !digitalRead(LED2_PIN)); // Toggle LED
    vTaskDelay(pdMS_TO_TICKS(2000));               // Delay for 2 seconds
  }
}

// Task 3: Handle BLE communication
void Task3(void *pvParameters) {
  while (1) {
    if (deviceConnected) {
      if (value == "ahmed") {
        String response = "ok i received hamada";
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
        Serial.println("Sent response: " + response);
        value = ""; // Clear the value
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Check BLE data every 100ms
  }
}

void setup() {
  Serial.begin(115200);

  // Configure LED pins
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  // BLE setup
  BLEDevice::init("BLE_FreeRTOS");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("BLE Server started...");

  // Create FreeRTOS tasks
  xTaskCreate(Task1, "Blink LED1", 1000, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "Blink LED2", 1000, NULL, 1, &Task2Handle);
  xTaskCreate(Task3, "BLE Task", 2000, NULL, 1, &Task3Handle);
}

void loop() 
{
  // No need to use loop as tasks handle the work
}

/*************************************************************************************************************/
#elif project == BLE_RTOS_Dual_core

// i change this(configUSE_TRACE_FACILITY to 1 in free rtos . h) 


#include <iostream>
#include <vector>
#include <iomanip>  // for std::setprecision

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <ArduinoJson.h>

#include <string.h>
#include <sstream>

#include <ESP32Time.h>

#include "../include/SD_CARD.hpp"

#include "rtc.h"
// #if !defined(ESP32)
// #include <avr/io.h>
// #endif



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include <RTClib.h>

// #include <rtc.h>

/***************************************************************************************************************/
// BLE Service and Characteristic UUIDs
#define SERVICE_UUID           "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID    "12345678-1234-1234-1234-1234567890cd"
/***************************************************************************************************************/

// LED Pins
#define LED1_PIN 2
#define LED2_PIN 0

// RTC_DS1307 rtc;

/***************************************************************************************************************/

BLECharacteristic *pCharacteristic; // this point used to config the communication of BLE ( read only , read write , ...)
bool deviceConnected = false; // state for BLE Connection 
String value;

ESP32Time rtc;
/***************************************************************************************************************/
// Task Handles
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t Task3Handle;
TaskHandle_t Task4Handle;
TaskHandle_t Task5Handle;
TaskHandle_t Task6Handle;
TaskHandle_t Task7Handle;

TaskHandle_t task_11_pin;
TaskHandle_t Task9Handle_kalman_MPUTask;
TaskHandle_t Task8Handle_Read_MPU_Task;
TaskHandle_t Task10Handle_Print_MPU_Task;

TaskHandle_t Task12ADCHandle;
TaskHandle_t Task13ADCSensorHandle;
TaskHandle_t Task14RTCReadHandle;
TaskHandle_t Task15RTCAdjustHandle;
TaskHandle_t Task16_SleepHandle; 
TaskHandle_t Task17_WakeupHandle;
TaskHandle_t Task18_GPS_GETDATAHandle; 
TaskHandle_t Task19_GPS_parseHandle;


/***************************************************** RTOS Queue ******************************************************/

QueueHandle_t rawDataQueue; // data that comes from sensor ( put in queue to communicate with another task to filter )
QueueHandle_t filteredDataQueue; // data that comes from filter ( put in queue to communicate with another task to print )
QueueHandle_t evendenceQueue;

QueueHandle_t GPS_velocity_parsed_Queue;
/***************************************************** semaphore create ******************************************************/
SemaphoreHandle_t XADC_Read=NULL ;
//test
int gas_senor_value=0;
int gas_value;

/**************************************** global variable ********************************************************/

char filename[20]= "/hamada1.txt"; // this file create inside SD_card and named by RTC day 
// const char* filename = "/hamada.txt";

double knots_velocity ;   // this is global variable used to save data after parsed from GPS in m/sec
double kmh_velocity;      // this is a global variable used to save data after parse from GPS in KM/H

int global_sleep_mode =0 ;

double deltaTime= 10.0; // this is used to make the integration in fuel consumption and kalman filter calculation
double total_fuel_consumption = 0.0; // Initialize once
char data_send_serial_string[256]; // json that send to site or mobile 
/**************************************** struct sleep mode  ********************************************************/

// struct for sleep mode one for event and another for sleep mode 
typedef struct 
{
  int event;
  int time_sleep = 0;

} ST_sleep_mode;

typedef struct 
{
  double speed_knots;
  double speed_kmh;
}ST_GPS_parsed_velocity;



/***************************************** global MPU6050  *******************************************************/

/*********************************************  caution for this part *****************************************************/
/*

kalman filter used to predect and correct the noise of MPU6050 (as acclerometer )
the numbers recommended from stack over flow and chatgpt 
if have any problem for it please search about it X_x


*/

/*************************************************************************************************************************/


//   // Kalman filter variables
// float angle = 0.0;              // Filtered angle
// float bias = 0.0;               // Gyroscope bias
// float P[2][2] = { { 0.1, 0 }, { 0, 0.1 } }; // Covariance matrix
// float Q_angle = 0.001;          // Process noise variance for the angle
// float Q_gyro = 0.003;           // Process noise variance for the gyroscope bias
// float R_angle = 0.03;           // Measurement noise variance
// float deltaTime = 0.01;         // Time step (seconds)

float Q_angle = 0.001;
float Q_bias = 0.003;             // float Q_gyro = 0.003;
float R_measure = 0.05;           // float R_angle = 0.03;
float angle = 0;
float bias = 0;
float P[2][2] = {0};
// float deltaTime = 0.01;         // Time step (seconds)

// Gyro scope Bias 
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

typedef struct
{
  int16_t accelX, accelY, accelZ;
  float temperture;
  int16_t gyroX,gyroY,gyroZ;
}MPUData;

typedef struct  
{
    float accAngleX, accAngleY, accAngleZ; //save the angle of the accelerometer
    float gyroRateX, gyroRateY, GyroRateZ; // save the rate of the gyroscope
    float gyrofilteredAngleX , gyrofilteredAngleY , gyrofilteredAngleZ; // save the filtered angle of the gyroscope
    float kalmanFilteredAngleX , kalmanFilteredAngleY , kalmanFilteredAngleZ;   // save the kalman filtered angle of the gyroscope
    float velocityX , velocityY , velocityZ; // this for test only 
    float linear_acceleration_x , linear_acceleration_y , linear_acceleration_z; // this for test only
    int temperature; // this for test only
}FilteredData;

// struct to calculate the linear acceleration
typedef struct 
{
  float linear_acceleration_x;
  float linear_acceleration_y;
  float linear_acceleration_z;

}ST_linear_acceleration;

#define MPU6050_ADDR            0x68
#define ACCEL_XOUT_H            0x3B
#define GYRO_XOUT_H             0x43
#define PWR_MGMT_1              0x6B
#define GYRO_CONFIG_ADD         0x1B
#define ACCEL_CONFIG_ADD        0x1C

typedef struct 
{
  double A;                      // it is the Idle fuel consimption let say is 0.5 l/ hour 
  double B;                      // it is the fuel consumption per unit speed let say is 0.02 L/ km.h 
  double C;                      // it is the fuel consumption per Acceleration  let say it is 0.01 L/M/S^2 
  double D;                      // it is the fuel consumption per gyro let say it is (0.005) L / deg / sec  
  double velocity;               // velocity of car 
  double acc_Mag;                // acceleration magnitude
  double gyro_Mag;               // gyro magnetude 

}ST_Fuel_Parameter;

/********************************************************************************** */

// moving Average Buffer 
#define MOVING_AVERAGE_SIZE 20
// this buffer used for aggressive filtering (low pass filter)
float acclBufferX[MOVING_AVERAGE_SIZE]= {0};
float acclBufferY[MOVING_AVERAGE_SIZE]= {0};
float acclBufferZ[MOVING_AVERAGE_SIZE]= {0};
int bufferIndex =0 ; // this used to point to the current index of the buffer

/************************************* velocity calculation from GPS ************************************************************/

// function for calculate the speed from knots to m/s 
double velocityCalculationFromKnots_Ms(double knots)
{
  return knots * 0.514444; // convert knots to meters/sec 
}

// return speed as km/h 
double velocityCalculationFromKmH_Ms(double kmH)
{
  // return kmH * (1000.0/3600.0); // convert km/h to meters/sec
  return kmH;
}

// this a function that return string (dynamically) and store it inside result  
std::vector <std::string> splitSentence(std::string &sentence)
{
  std::vector <std::string> result; // which save every work like in array 
  std::stringstream ss(sentence);
  std::string word ;

  while(getline(ss, word, ','))  // split the sentence by comma
  {
    result.push_back(word);
  }
  return result;
}

/***************************************************************************************************************/
/************************************* callback BLE ************************************************************/
// BLE Callbacks  , we make inherit from BLEServerCallbacks class and this handle state connect or disconnect
class MyServerCallbacks : public BLEServerCallbacks 
{
  void onConnect(BLEServer* pServer)  // this for do the same func + edit the global variable to true 
  {
    deviceConnected = true; 
    Serial.println("Client Connected");
  }

  void onDisconnect(BLEServer* pServer)     // this for do the same func + edit the global variable to false 
  {
    deviceConnected = false;
    Serial.println("Client Disconnected");

    // resitting the BLE to new connection  
    BLEAdvertising *pAdvertising = pServer->getAdvertising();   
    pAdvertising->start();
  }
};

// call back for receiving message , also inherit form BLECharacteristicCallbacks class and collect data into value 
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks 
{
  void onWrite(BLECharacteristic *pCharacteristic) 
  {
    // get value as string in c and save it into value (globale variable)
    value = pCharacteristic->getValue().c_str();
    // this for debug 
    Serial.print("Received: ");
    Serial.println(value);
  }
};


/***************************************************************************************************************/

// MPU6050 Functions Prototypes
void MPU6050_Init();
void readMPU6050RawData(MPUData& data);
void calibrateGyro(float &gyroBaisX , float &gyroBaisY , float &gyroBaisZ);
float getAccelerationAngleX( const MPUData &data);
float getAccelerationAngleY( const MPUData &data);
float getAccelerationAngleZ( const MPUData &data);
float kalmanFilter(float newAngle, float newRate, float deltaTime);
float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bais, float alpha = 0.95);
float movingAverage(float newValue, float buffer[], int size);
float exponentialMovingAverage(float newValue, float oldValue, float alpha);
ST_linear_acceleration getLinearAcceleration(const MPUData &data , FilteredData &angles);

/********************************** variable for MPU  ******************************/

// this value we get it from sensor as a radiant angle ( except temp )
int16_t accX , accY, accZ ;
int16_t gyro_X, gyro_Y, gyro_Z;
int Temp ; // there are temp sensor inside MPU

/*  accelecation config:
*0: ± 2g
*1: ± 4g
*2: ± 8g
*3: ± 16g
*/

int8_t Acc_config_val = 0 ; 

/* gyro config :
0: ± 250 °/s
1: ± 500 °/s
2: ± 1000 °/s
3: ± 2000 °/s
**/
int8_t Gyro_config_var= 0; 

/***********************************************************************************/

// function implementation 

void MPU6050_Init()
{
  // set power (wake up the mpu)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1); // go to address of power management inside the Mpu6050
  Wire.write(0);  // set all for 0 for wake up the mpu 
  Wire.endTransmission(true);

  // set the gyro config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG_ADD); // go to address of gyro config inside the Mpu6050
  Wire.write(Gyro_config_var);  // set the gyro config 
  Wire.endTransmission(true);

  // set the accel config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONFIG_ADD); // go to address of accel config inside the Mpu6050
  Wire.write(Acc_config_val);  // set the accel config
  Wire.endTransmission(true);

  // Wire.beginTransmission(MPU6050_ADDR);
  // Wire.write(0x1A); // CONFIG register
  // Wire.write(0x03); // Set DLPF to 44 Hz
  // Wire.endTransmission(true);

  // Wire.beginTransmission(MPU6050_ADDR);
  // Wire.write(0x19); // SMPLRT_DIV register
  // Wire.write(0x09); // Set sampling rate to 100 Hz
  // Wire.endTransmission(true);


}

void readMPU6050RawData(MPUData& data)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  //ISO C++ says that these are ambiguous, even though the worst conversion for the first is better than the worst conversion for the second:
  //The problem is that the Wire.requestFrom function is ambiguous. To fix this, you need to explicitly cast the third parameter to uint8_t.
  Wire.requestFrom(MPU6050_ADDR, 14, static_cast<uint8_t>(true));

  // Read accelerometer data the data is 16 bit so we need to shift it to left 8 bit and or it with the next byte
  data.accelX = Wire.read() << 8 | Wire.read();
  data.accelY = Wire.read() << 8 | Wire.read();
  data.accelZ = Wire.read() << 8 | Wire.read();

  // Read temperature
  int16_t rawTemp = Wire.read() << 8 | Wire.read() ;
  // data.temperture = rawTemp / 3400.00 ;  // Convert to Celsius
 
  data.temperture =  (float)(rawTemp / 340.0 + 36.53); // (rawTemp + 12421) / 340.0; // Convert to Celsius
// Serial.println("the reading of the temperture is : " );
// Serial.println(data.temperture);
  // Read gyroscope data
  data.gyroX = Wire.read() << 8 | Wire.read();
  data.gyroY = Wire.read() << 8 | Wire.read();
  data.gyroZ = Wire.read() << 8 | Wire.read();

        // Serial.print(" GX= ");
      // Serial.print( data.accelX / 16384.0);
      // Serial.print(" GY= ");
      // Serial.print( data.accelY / 16384.0); 
      // Serial.print(" GZ= ");
      // Serial.print( data.accelZ /16384);

      // Serial.print(" AX= ");
      // Serial.print( data.gyroX / 131.0);
      // Serial.print(" AY= ");
      // Serial.print( data.gyroY/ 131.0);
      // Serial.print(" AZ= ");
      // Serial.println( data.gyroX /131.0);
}

void calibrateGyro(float &gyroBaisX , float &gyroBaisY, float &gyroBiasZ)
{
  MPUData data;
  long baisX= 0, baisY = 0, baisZ = 0;
  const int sample =500 ;

  for (int i = 0; i < sample; i++)
  {
    readMPU6050RawData(data);
    baisX += data.gyroX;
    baisY += data.gyroY;
    baisZ += data.gyroZ;
    delay(2);
  }

  gyroBaisX = (float)baisX / (sample * 131.0) ;
  gyroBaisY = (float)baisY / (sample * 131.0) ;
  gyroBiasZ = (float)baisZ / (sample * 131.0) ;

}

// this function used to get the angle of the accelerometer by equation 
float getAccelerationAngleX( const MPUData &data)
{
  return atan2(data.accelY, sqrt(data.accelX * data.accelX + data.accelZ * data.accelZ)) * RAD_TO_DEG ; // 180 / PI
}

float getAccelerationAngleY(const MPUData &data)
{
  return atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ)) * RAD_TO_DEG ; // 180 / PI
}

float getAccelerationAngleZ(const MPUData & data)
{
  return atan2(sqrt(data.accelX * data.accelX + data.accelY * data.accelY), data.accelZ) * RAD_TO_DEG ; // 180 / PI
}

float kalmanFilter(float newAngle, float newRate, float deltaTime)
{
  // prediction update
  float rate = newRate-bias;
  angle += deltaTime* rate ; 

  // update estimation error covariance - project the error covariance ahead
  P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle); 
  P[0][1] -= deltaTime * P[1][1];
  P[1][0] -= deltaTime * P[1][1];
  P[1][1] += Q_bias * deltaTime;

  // calculate kalman gain 
  float S = P[0][0] + R_measure;
  float K[2] ;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // update the angle and bias with the measurement 
  float y = newAngle - angle;
  angle += K[0] * y; 
  bias  += K[1] * y;


  // update the estimation error covariance
  float P00_temp = P[0][0];
  float P01_temp = P[0][1]; 

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp; 

  return angle;

}


float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bais, float alpha)
{
  // float angle = alpha * (angle + gyroRate * deltaTime) + (1 - alpha) * accAngle;
  // return angle;

  static float gyroAngle = 0;
  gyroRate -= bais; 
  gyroAngle += gyroRate * deltaTime;
  return alpha * gyroAngle + (1-alpha) * accAngle;

}


// Moving Average Filter
float movingAverage(float newValue, float buffer[], int size) {
    buffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % size;

    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

// Exponential Moving Average Filter (as low pass filter )
float exponentialMovingAverage(float newValue, float oldValue, float alpha) {
    return alpha * newValue + (1 - alpha) * oldValue;
}


ST_linear_acceleration getLinearAcceleration(const MPUData &data , FilteredData &angles)
{
    const float GRAVITY = 9.81; // Gravity constant in m/s^2
    ST_linear_acceleration linAccel;

    // Calculate linear acceleration using tilt angles
    linAccel.linear_acceleration_x = (data.accelX / 16384.0) - GRAVITY * sin(angles.accAngleX * DEG_TO_RAD);
    linAccel.linear_acceleration_y = (data.accelY / 16384.0) - GRAVITY * sin(angles.accAngleY * DEG_TO_RAD);
    linAccel.linear_acceleration_z = (data.accelZ / 16384.0) - (GRAVITY * cos(angles.accAngleX * DEG_TO_RAD) * cos(angles.accAngleY * DEG_TO_RAD));

    return linAccel;
}

// this equation used to calculate the consuming of fuel i put my calculation on the idle 1600 cc car (0.8) L / H
double Fuel_Consumption(ST_Fuel_Parameter & fuel_Consumption)
{
  return fuel_Consumption.A + (fuel_Consumption.B * fuel_Consumption.velocity ) + (fuel_Consumption.C * fuel_Consumption.acc_Mag) + (fuel_Consumption.D *fuel_Consumption.gyro_Mag);
}
/***************************************************************************************************************/
// Task 1: Blink LED1 every 1 second (Core 0)
String hamada_test_send_filter_data_over_ble;

void Task1(void *pvParameters) 
{
  while (1) 
  {
    // toggle led (read pin state and change it )
    digitalWrite(LED1_PIN, !digitalRead(LED1_PIN)); // Toggle LED
    // Serial.println("iam task1");
    vTaskDelay(pdMS_TO_TICKS(1000));               // Delay for 1 second
    
  }
}

// Task 2: Blink LED2 every 2 seconds (Core 1)
void Task2(void *pvParameters) 
{
  while (1) 
  {
    digitalWrite(LED2_PIN, !digitalRead(LED2_PIN)); // Toggle LED
    // Serial.println("iam task2");
    vTaskDelay(pdMS_TO_TICKS(2000));               // Delay for 2 seconds
  }
}

// connection for BLE 
void Task3(void *pvParameters) 
{
  while (1) 
  {
    // Ensure the device is connected via BLE
    if (deviceConnected) 
    {
      // Check the received value and act accordingly
      if (value == "ahmed") 
      {
        String response = "ok i received hamada";
        pCharacteristic->setValue(response.c_str()); 
        pCharacteristic->notify();
        Serial.println("Sent response: " + response);
        
        value = ""; // Clear the value after sending response
      }
      else if (value == "1")
      {
        String response = hamada_test_send_filter_data_over_ble;
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
        Serial.println("Sent response: " + response);
        
        value = ""; // Clear the value after sending response
      }
      else if (value == "2") 
      {
        std::string response = " ok the sleep mode is activated ";
        pCharacteristic-> setValue(response.c_str());
        pCharacteristic->notify();
        Serial.println("Sent response: " + String(response.c_str()));
        global_sleep_mode =1 ;
      }
      else if (!value.isEmpty())
      {
        // Optional: Handle unexpected or other values, and clear value to avoid residual data
        Serial.println("Received unexpected value: " + value);
        value = ""; // Clear after handling
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to prevent rapid checking
  }
}




/***************** for only test but not work X_x *****************/ 
void Task4_printstate_Task(void *pvParameter)
{
  char buffer_state[1555] ; 
  while(1)
  {
    Serial.println("\nTask Name    State    Priority    Stack    Num");
    Serial.println("---------------------------------------------");
        // Clear the buffer before calling vTaskList
    memset(buffer_state, 0, sizeof(buffer_state));

    vTaskList(buffer_state); // this is the  problem 
    Serial.println(buffer_state);

    vTaskDelay(pdMS_TO_TICKS(5000)); // print state every 5 sec 
  }
  

}


// u should make this task suspend from RTC 
// create flag that indecate that the file created successfully 
// this for creating file in sd card
void Task5_Create_file_SD(void* pvParameter)
{
    while(1)
    { 
      // create file 
      SD_create_File(filename);
      vTaskDelay(pdMS_TO_TICKS(10000)); 
    }

}

void Task6_Write_tofile_SD(void* pvParameter)
{
  int static i= 0 ;
  char x [10];
  while(1)
  {
    // file name it is the name that created by RTC 
    // i need to write value and counter 
    SD_Write_append(filename, "hamada_test_send_filter_data_over_ble");
    // this function used to convert the number into char 
    snprintf(x, sizeof(x), "%d", i); 
    SD_Write_append(filename, x );
    i++;
    vTaskDelay(pdMS_TO_TICKS(1000)); 
  }

}

// this may used to read file and send over GSM

void Task7_read_file_SD(void* pvParameter)
{
    while(1)
    {
       // create file and check if not created create it and if create not create another one 
      SD_readFile(filename);
      vTaskDelay(pdMS_TO_TICKS(10000)); 
    }

}

// is task is used to read from MPU acclerormeter 
void Task8_sensorTask(void * pvParameter)
{
  MPUData rawData;
  while(1)
  {
    // read the mpu data and saved into this variable (rawdata)
    readMPU6050RawData(rawData);
    // put the data in the queue 
    xQueueSend(rawDataQueue, &rawData, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void Task9_filterTask (void * pvParameter)
{
  MPUData rawData ; // this used to received data from the queue
  FilteredData filteredData; // this used to save the filtered data and send to anothe queue for print function (task)
  unsigned long previousTime = micros(); // used for calculate delta time
  float velocityX =0 , velocityY = 0 , velocityZ = 0 ;
  float prevAccelX =0 , prevAccelY = 0 , prevAccelZ = 0 ;
  const float alpha = 0.06; // increased smoothing factor for EMA

  while (1)
  {
    if (xQueueReceive(rawDataQueue, &rawData , portMAX_DELAY))
    {
      unsigned long currentTime = micros();
      // delta time : itis the time btw the two reading of the same task ( filter task )
      deltaTime = (currentTime - previousTime) / 1000000.0; //////////

      if (deltaTime <= 0 )
      {
        deltaTime = 0.01 ; // default time is 10 ms if the time = 0 ( if u can't access time )
      }

      previousTime = currentTime; // update the previous time

      // get data angle from accelerometer
      filteredData.accAngleX= getAccelerationAngleX(rawData);
      filteredData.accAngleY= getAccelerationAngleY(rawData);
      filteredData.accAngleZ= getAccelerationAngleZ(rawData);

      // get data rate from gyroscope

      filteredData.gyroRateX = rawData.gyroX /131.0;
      filteredData.gyroRateY = rawData.gyroY /131.0;
      filteredData.GyroRateZ = rawData.gyroZ /131.0;

      // use kalman filter for predect the gyro angle
      filteredData.gyrofilteredAngleX =kalmanFilter(filteredData.accAngleX, filteredData.gyroRateX, deltaTime); 
      filteredData.gyrofilteredAngleY =kalmanFilter(filteredData.accAngleY, filteredData.gyroRateY, deltaTime);
      filteredData.gyrofilteredAngleZ =kalmanFilter(filteredData.accAngleZ, filteredData.GyroRateZ, deltaTime);

      //applay moving average filter for the accelerometer data
      // filteredData.accAngleX = movingAverage(filteredData.accAngleX, acclBufferX, MOVING_AVERAGE_SIZE);
      // filteredData.accAngleY = movingAverage(filteredData.accAngleY, acclBufferY, MOVING_AVERAGE_SIZE);
      // filteredData.accAngleZ = movingAverage(filteredData.accAngleZ, acclBufferZ , MOVING_AVERAGE_SIZE);

      // apply exponential moving average filter for the accelerometer data
        float accelX = exponentialMovingAverage(rawData.accelX / 16384.0, prevAccelX, alpha);
        float accelY = exponentialMovingAverage(rawData.accelY / 16384.0, prevAccelY, alpha);
        float accelZ = exponentialMovingAverage(rawData.accelZ / 16384.0, prevAccelZ, alpha);

        ST_linear_acceleration linearACC = getLinearAcceleration(rawData, filteredData);

        filteredData.linear_acceleration_x = linearACC.linear_acceleration_x;
        filteredData.linear_acceleration_y = linearACC.linear_acceleration_y;
        filteredData.linear_acceleration_z = linearACC.linear_acceleration_z;

        prevAccelX = accelX;
        prevAccelY = accelY;
        prevAccelZ = accelZ;


        // bias correction (assuming the sensor is stationary at the start)

        static float accelBiasX = accelX;
        static float accelBiasY = accelY;
        static float accelBiasZ = accelZ; 

        accelX -= accelBiasX;
        accelY -= accelBiasY;
        accelZ -= accelBiasZ;


        // calculate velocity
        velocityX += accelX * deltaTime;
        velocityY += accelY * deltaTime;
        velocityZ += accelZ * deltaTime;

            // Drift correction (simple example, may need more sophisticated approach)
            velocityX *= 0.99;
            velocityY *= 0.99;
            velocityZ *= 0.99;

        // save the velocity in the filtered data
        filteredData.velocityX = velocityX;
        filteredData.velocityY = velocityY;
        filteredData.velocityZ = velocityZ;

        filteredData.temperature = rawData.temperture;

        xQueueSend(filteredDataQueue, &filteredData, portMAX_DELAY);


    }
  }
}

unsigned long previousTime1 = 0; 

void Task10_display_Task(void * pvParameter)
{
  FilteredData filteredData;
  float acc_Mag_calibration =0.0 , gyro_Mag_calibration =0.0 ;
  while(true)
  {
    if(xQueueReceive(filteredDataQueue, &filteredData, portMAX_DELAY))
    {
      hamada_test_send_filter_data_over_ble = "gyro : " +  (String)filteredData.gyrofilteredAngleX + " " + (String)filteredData.gyrofilteredAngleY + " " + (String)filteredData.gyrofilteredAngleZ+ "\n\r"
      +"acc : " + (String)filteredData.accAngleX + " " + (String)filteredData.accAngleY + " " + (String)filteredData.accAngleZ + "\n\r" 
      +"velocity : " + (String)filteredData.velocityX + " " + (String)filteredData.velocityY + " " + (String)filteredData.velocityZ + "\n\r"
      +"pin state 12 : " + (String)digitalRead(12) + "\n\r"
      +"pin state 2 : " + (String)digitalRead(2) + "\n\r"
      +"ADC value : " + (String)gas_senor_value + "\n\r" 
      +"gas value : " + (String)gas_value + "\n\r"
      + "temperature : " + (String)filteredData.temperature + "\n\r"
      + "gpsvelocity : " + (String)knots_velocity + " " + (String)kmh_velocity + "\n\r"
      + "file name : " + (String)filename + "\n\r"
      + "Json file : " + (String)data_send_serial_string+ "\n\r" ;
      // + "fuel consumption : " + (String)total_fuel_consumption + "\n\r" ; // Initialize once;

      // calculate the tilt acceleration
      // Serial.print(" AXT= ");
      // Serial.print(filteredData.accAngleX);
      // Serial.print(" AYT= ");
      // Serial.print(filteredData.accAngleY); 
      // Serial.print(" AZT= ");
      // Serial.print(filteredData.accAngleZ);

      
      Serial.print(" GPS_velocity_knots= ");
      Serial.print(knots_velocity);
      Serial.print(" GPS_velocity_kmh= ");
      Serial.print(kmh_velocity);

      // Serial.print(" GX= ");
      // Serial.print(filteredData.gyroRateX);
      // Serial.print(" GY= ");
      // Serial.print(filteredData.gyroRateY);
      // Serial.print(" GZ= ");
      // Serial.print(filteredData.GyroRateZ);

      // Serial.print(" VX = ");
      // Serial.print(filteredData.velocityX);
      // Serial.print(" VY = ");
      // Serial.print(filteredData.velocityY);
      // Serial.print(" VZ = ");
      // Serial.println(filteredData.velocityZ);

      Serial.print(" AXL= ");
      Serial.print(filteredData.linear_acceleration_x);
      Serial.print(" AYL= ");
      Serial.print(filteredData.linear_acceleration_y);
      Serial.print(" AZL= ");
      Serial.print(filteredData.linear_acceleration_z);

      float accelAngleXY = sqrt(filteredData.linear_acceleration_x * filteredData.linear_acceleration_x + filteredData.linear_acceleration_y * filteredData.linear_acceleration_y + filteredData.linear_acceleration_z * filteredData.linear_acceleration_z);
      Serial.print(" AXYZ= ");
      Serial.print(accelAngleXY - acc_Mag_calibration);


            float gyro_magnitude = sqrt(filteredData.gyroRateX * filteredData.gyroRateX + filteredData.GyroRateZ * filteredData.GyroRateZ + filteredData.gyroRateY * filteredData.gyroRateY);
      Serial.print(" GXYZ= ");
      Serial.print(gyro_magnitude - gyro_Mag_calibration);


      bool iscalibrate = false; 

      // this is used to calibrate the gyro mag and acc mag in the first because the data first in 
      if (!iscalibrate)
      {
        acc_Mag_calibration = accelAngleXY+0.01;
        gyro_Mag_calibration = gyro_magnitude; 
        iscalibrate = true ;
      }

      ST_Fuel_Parameter fuel_Consumption_Para ;
      fuel_Consumption_Para.A = 0.8; // idle case for 1600 CC
      fuel_Consumption_Para.B = 0.065 ; // unit speed
      fuel_Consumption_Para.C = 0.03; // per acc 
      fuel_Consumption_Para.D = 0.007 ; // per gyro 
      fuel_Consumption_Para.velocity = (kmh_velocity); 
      fuel_Consumption_Para.acc_Mag = abs(accelAngleXY - acc_Mag_calibration);
      fuel_Consumption_Para.gyro_Mag = abs(gyro_magnitude - gyro_Mag_calibration);
      // double fuel_dt = deltaTime / 3600.0;  // Convert 10 ms to hours (10ms = 2.77e-6 hours)
     
      unsigned long previousTime1 = 0;
      previousTime1 = micros();

        unsigned long currentTime1 = micros();
        double deltaTime1 = (currentTime1 - previousTime1) / 1000000.0;
          // If first run or invalid time, set default
        if (deltaTime <= 0) 
        {
          deltaTime = 0.01; // Default to 10 ms
        }


         double fuel_dt = deltaTime / 3600.0;
       total_fuel_consumption += Fuel_Consumption(fuel_Consumption_Para) * fuel_dt;

      Serial.print("fuel cons:"); 
      Serial.println(total_fuel_consumption,6);

    StaticJsonDocument<400> dataFrame;

    deserializeJson(dataFrame, "{}");  // Not needed, the document is already empty
    dataFrame["serialNumber"] = "EG_123_475";
    dataFrame["time"] = String(rtc.getHour()) + "-" + String(rtc.getMinute()) + "-" + String(rtc.getSecond()) ;                   //"2025-02-03T11:50:00Z";  // ISO 8601 format
    dataFrame["date"] = String(rtc.getDay()) + "-" + String(rtc.getMonth()) + "-"+ String(rtc.getYear()) ;
    dataFrame["longitude"] = 31.42547;  
    dataFrame["latitude"] = 34.1124;
    dataFrame["velocity"] = knots_velocity;  
    dataFrame["fuelConsumption"] = total_fuel_consumption;
    dataFrame["temperature"] = Temp;
    dataFrame["acceleration"] = accelAngleXY;
    dataFrame["gyro"] = gyro_magnitude;

    // char data_send_serial_string[256];  // Adjust size as needed
    serializeJson(dataFrame, data_send_serial_string, sizeof(data_send_serial_string));

    Serial.println(data_send_serial_string);


      // if (abs(accelAngleXY )>0.9 && abs(accelAngleXY )< 2.1)
      // {
      //   std::string state = " the car start moving low speed ";
      //   Serial.println(state.c_str());
      // }
      // else if(abs(accelAngleXY) > 0.9 && abs(accelAngleXY )< 4.1)
      // {
      //   std::string state = " the car acceptable speed ";
      //   Serial.println(state.c_str());
      //   digitalWrite(12,LOW);
      // }
      // else if (abs(accelAngleXY) > 4.1)
      // {
      //   std::string state = " the car high speed ";
      //   Serial.println(state.c_str()); 
      //   // send state to BLE and buzz on the board
      //   digitalWrite(12,HIGH);

      // }  
      // else
      // {
      //   std::string state = " the car stop moving ";
      //   Serial.println(state.c_str());
      //   digitalWrite(12,LOW);
      // }


    }
    
    // Your task logic here
    vTaskDelay(pdMS_TO_TICKS(10));
  }


}


void Task11_read_gpio_pin(void * pvParameter)
{
  pinMode(2,INPUT);
  pinMode(12,OUTPUT);
  pinMode(0,OUTPUT);
  while(1)
  {
    // Serial.println("the pin state is : ");
    // Serial.println(digitalRead(2));
    if (digitalRead(2) == 0 )
    {
      digitalWrite(12,LOW);
      digitalWrite(0,LOW);
    }
    else
    {
      digitalWrite(12,HIGH);
      digitalWrite(0,HIGH);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

  }
}


void Task12_ADC_read(void * pvParameter)
{

  while(1)
  {
    if(xSemaphoreTake(XADC_Read, portMAX_DELAY) == pdTRUE)
    {
    gas_senor_value = analogRead(A6);
      // Serial.println("ADC value is : ");
      // Serial.println(gas_senor_value);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

  }
}

void Task13_ADC_Sensor(void * pvParameter)
{
  while(1)
  {

    xSemaphoreGive(XADC_Read);

    float voltagelevel=float(3300*gas_senor_value/4095);
    gas_value = int(voltagelevel/10.0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    // Serial.println("the gas value is : ");
    // Serial.println(gas_value);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

SemaphoreHandle_t rtcMutex;

void Task14_read_RTC(void* pvParameter)
{
    while(1)
    {
        if (xSemaphoreTake(rtcMutex, portMAX_DELAY))
        {
            Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
            struct tm timeinfo = rtc.getTimeStruct();
            xSemaphoreGive(rtcMutex);
        }
          // we need to creat file with the same day 
          sprintf (filename ,"%d_%d_%d.txt" , rtc.getDay() , rtc.getMonth()+1 ,rtc.getYear() );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

void Task15_adjust_RTC(void* pvParameter)
{
    if (xSemaphoreTake(rtcMutex, portMAX_DELAY))
    {
        Serial.println("...................................................................");
        Serial.println("RTC Adjusted");
        rtc.setTime(30, 5, 10, 2, 2, 2025); // sec , min ,hour , day , month , year 
        Serial.println("...................................................................");
        xSemaphoreGive(rtcMutex);
    }
    vTaskDelete(NULL);
}


void Task16_sleep_evidence(void* pvParameter) 
{
  // int evendence = 0 ; // mean there is no event 
  ST_sleep_mode evendence ;
  evendence.event = global_sleep_mode;
  evendence.time_sleep = 10; // default time to sleep is 10 sec

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  while (1)
  {
    // evendence.event = random(0,2); // random number between 0 and 1
     evendence.event = global_sleep_mode;
    // evedence = digitalRead(2); 
    // we detect the evedence and send it to the other task
    Serial.println("the evidence is : ");
    Serial.println(evendence.event);
    switch(wakeup_reason)
    {
      case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
      case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
      case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
      case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
      default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }

    xQueueSend(evendenceQueue, &evendence, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));

  }
}

void Task17_wakeup_action(void* pvParameter)
{
  ST_sleep_mode evendence ;

  while(1)
  { 
    if (xQueueReceive(evendenceQueue, &evendence , portMAX_DELAY)== pdPASS)
    {
      if (evendence.event ==1)
      {
        // esp_sleep_enable_timer_wakeup(10 * 1000000); // Wake up after 10 seconds
        esp_sleep_enable_timer_wakeup(evendence.time_sleep * 1000000); // Wake up after time_sleep seconds
        Serial.println("Going to sleep now");
        global_sleep_mode= 0;
        esp_deep_sleep_start(); 
      }
      else
      {
        Serial.println("no event");
      }
    }

  }
}


double stringToDouble(const std::string& str)
{
    double result = 0.0;
    bool isNegative = false;
    size_t i = 0;

    if (str[i] == '-')
    {
        isNegative = true;
        i++;
    }

    while (i < str.length() && std::isdigit(str[i]))
    {
        result = result * 10 + (str[i] - '0');
        i++;
    }

    if (i < str.length() && str[i] == '.')
    {
        i++;
        double fractionalPart = 0.0;
        double divisor = 10.0;

        while (i < str.length() && std::isdigit(str[i]))
        {
            fractionalPart += (str[i] - '0') / divisor;
            divisor *= 10.0;
            i++;
        }

        result += fractionalPart;
    }

    return isNegative ? -result : result;
}



void Task18_GPS_velocity_Calculations(void * pvParameter)
{
  ST_GPS_parsed_velocity gps_velocity;
  // "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
  std::string NEMA_sentence = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48" ;
  // std::string NEMA_sentence = "$GPVTG,054.7,T,034.4,M,005.5,N,01000.2,K*48" ;

  while(1)
  {
    // u need to get data from gps  here 

    std:: vector<std::string> parts = splitSentence(NEMA_sentence);

    gps_velocity.speed_knots = stringToDouble(parts[5]); // extract the speed in knots in the frame
    gps_velocity.speed_kmh =   stringToDouble(parts[7]);     //std::stod(parts[8]);   // extract the speed in km/h

    xQueueSend(GPS_velocity_parsed_Queue, &gps_velocity, portMAX_DELAY); 

    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}
void Task19_GPS_velocity_Get_Action(void* pvParameter)
{
  ST_GPS_parsed_velocity gps_velocity ; 
  while(1)
  {
    if(xQueueReceive(GPS_velocity_parsed_Queue, &gps_velocity, portMAX_DELAY))
    {
      knots_velocity = velocityCalculationFromKnots_Ms(gps_velocity.speed_knots);
      kmh_velocity = velocityCalculationFromKmH_Ms(gps_velocity.speed_kmh)  ; // +90 for test only 

    }
  }
}

void printWatermark(void *pvParameters)
{
  while(1)
  {
      // delay(2000);
      Serial.print("TASK: ");
      Serial.print(pcTaskGetName(Task1Handle)); // Get task name with handler
      Serial.print(", High Watermark: ");
      Serial.print(uxTaskGetStackHighWaterMark(Task1Handle));
      Serial.println();

      Serial.print("TASK: ");
      Serial.print(pcTaskGetName(Task2Handle)); // Get task name with handler
      Serial.print(", High Watermark: ");
      Serial.print(uxTaskGetStackHighWaterMark(Task2Handle));
      Serial.println();

      Serial.print("TASK: ");
      Serial.print(pcTaskGetName(Task3Handle)); // Get task name with handler
      Serial.print(", High Watermark: ");
      Serial.print(uxTaskGetStackHighWaterMark(Task3Handle));
      Serial.println();

      Serial.print("Task: ");
      Serial.print(pcTaskGetName(Task5Handle)); // Get task name with handler
      Serial.print(", High Watermark: ");
      Serial.println(uxTaskGetStackHighWaterMark(Task5Handle));

      Serial.print("Task: ");
      Serial.print(pcTaskGetName(Task6Handle)); // Get task name with handler
      Serial.print(", High Watermark: ");
      Serial.println(uxTaskGetStackHighWaterMark(Task6Handle));

      vTaskDelay(pdMS_TO_TICKS(1000)); 
  }
}


void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  // Wire.begin(8);
  // rtc.begin();
  // Configure LED pins
  
  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);



  // BLE setup
  BLEDevice::init("HamadaBLE"); // set the init and the name of bluetooth
  BLEServer *pServer = BLEDevice::createServer(); // make the BLE as a server 
  pServer->setCallbacks(new MyServerCallbacks()); // set server callback with callback function 
  BLEService *pService = pServer->createService(SERVICE_UUID); // service connection (must be unique)

  // config the connection characteristic (in this test we make read and write) you can make also read only , write without received , ....
  pCharacteristic = pService->createCharacteristic
  (
    // identifiy the services (name) ie (UUID) and the operation on this  services (read , write) 
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("BLE Server started...");

  // rtc.begin(); // Initialize RTC here
  // if (!rtc.isrunning()) {
  //   Serial.println("RTC is NOT running!");
  // } else {
  //   Serial.println("RTC is running");
  // }
  // Configure LED pins
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  // sd card setup 
  SD_card_init();

  // init mpu 
  MPU6050_Init();


  rawDataQueue = xQueueCreate(15, sizeof(MPUData)); // For raw data
  filteredDataQueue = xQueueCreate(15, sizeof(FilteredData));  // For filtered data 
  evendenceQueue = xQueueCreate(15, sizeof(ST_sleep_mode));
  GPS_velocity_parsed_Queue = xQueueCreate(10,sizeof(ST_GPS_parsed_velocity));
  //create semaphore XADC_Read 
  XADC_Read= xSemaphoreCreateBinary();
  rtcMutex = xSemaphoreCreateMutex();
  // Create FreeRTOS tasks pinned to specific cores
  xTaskCreatePinnedToCore(Task1, "Blink LED1", 1000, NULL, 1, &Task1Handle, 0);   // Core 0
  xTaskCreatePinnedToCore(Task2, "Blink LED2", 1000, NULL, 1, &Task2Handle, 0);   // Core 0
  xTaskCreatePinnedToCore(Task3, "BLE Task", 20000, NULL, 5, &Task3Handle, 0);     // Core 0

  xTaskCreatePinnedToCore(Task5_Create_file_SD, "create sd Task", 3500, NULL, 2, &Task5Handle, 0);     // Core 0
  xTaskCreatePinnedToCore(Task6_Write_tofile_SD, "Write_tofile_SD Task", 3500, NULL, 1, &Task6Handle, 0);     // Core 0
  xTaskCreatePinnedToCore(Task7_read_file_SD, "read_file Task", 5000, NULL, 1, &Task7Handle, 1);     // Core 0

  // xTaskCreatePinnedToCore(printWatermark, "watermark Task", 2000 , NULL,1,NULL, 0); // core 0
  xTaskCreatePinnedToCore(Task8_sensorTask, "sensor Task", 10048, NULL,3, &Task8Handle_Read_MPU_Task,0);
  xTaskCreatePinnedToCore(Task9_filterTask, "filter Task", 10048, NULL,3, &Task9Handle_kalman_MPUTask,0);
  xTaskCreatePinnedToCore(Task10_display_Task, "display Task", 10048, NULL,3, &Task10Handle_Print_MPU_Task,0);
  xTaskCreatePinnedToCore(Task11_read_gpio_pin, "read gpio task and take action",4048, NULL,1,&task_11_pin , 0);
  xTaskCreatePinnedToCore(Task12_ADC_read, "ADC read task", 4048, NULL, 1, &Task12ADCHandle, 0);
  xTaskCreatePinnedToCore(Task13_ADC_Sensor, "ADC sensor task", 4048, NULL, 1, &Task13ADCSensorHandle, 0);

  xTaskCreatePinnedToCore(Task14_read_RTC, "RTC read task", 2048, NULL, 5, &Task14RTCReadHandle, 0);
  xTaskCreatePinnedToCore(Task16_sleep_evidence, "evedence task" , 2048, NULL, 1, &Task16_SleepHandle,0);
  xTaskCreatePinnedToCore(Task17_wakeup_action, "wakeup action task", 2048, NULL, 1, &Task17_WakeupHandle,0);
  if (xTaskCreatePinnedToCore(Task15_adjust_RTC, "RTC adjust task", 2048, NULL, 5, &Task15RTCAdjustHandle, 0) != pdPASS)
  {
    Serial.println("Task15_adjust_RTC creation failed!");
  }
  if (xTaskCreatePinnedToCore(Task18_GPS_velocity_Calculations , "Gps velocity get data" , 10048 , NULL, 1, &Task18_GPS_GETDATAHandle ,0) !=pdPASS)
  {
       Serial.println("Task18_GPS_velocity_Calculations creation failed!");
  }
    if (xTaskCreatePinnedToCore(Task19_GPS_velocity_Get_Action , "Gps velocity parsed data" ,10048 , NULL, 1, &Task19_GPS_parseHandle , 1) !=pdPASS)
  {
       Serial.println("Task19_GPS_velocity_Get_Action creation failed!");
  }



 // xTaskCreatePinnedToCore(Task4_printstate_Task,"printstate",12000,NULL,1,&Task4Handle,0); // Core 0
}

void loop()
{
  // No code needed here; tasks handle everything
}

#elif project == SD_card_test

#include "../include/SD_CARD.hpp"

void setup()
{
  Serial.begin(115200);
  SD_card_init() ; 
  SD_create_File("/test.txt");
  SD_Write_append("/test.txt","hello from esp32 ^_^") ;
  SD_readFile("/test.txt"); 

  // Initialize SD card
} 
void loop() 
{
  // Nothing here
}

#elif project == Kalman_MPU6050

#include <Wire.h>
float RateRoll, RatePitch, RateYaw;     // roll for x axis , pitch for z and yaw for y axis 
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // this is for calibration after the SW filter (kalmen) 
int RateCalibrationNumber; 
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch; 
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}
void loop() {
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}

#elif project == kalman_MPU6050_1

#define test_code_dummy_vale  1
#define test_prepheral    2

#define Test_project  test_prepheral

    #if Test_project == test_prepheral

    #include <Wire.h>
#include <math.h>

#define MPU6050_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1   0x6B

// Kalman Filter Variables
float Q_angle = 0.001;  // Process noise covariance (angle)
float Q_gyro = 0.003;   // Process noise covariance (gyro bias)
float R_angle = 0.03;   // Measurement noise covariance
float angle = 0;        // Filtered angle
float bias = 0;         // Gyroscope bias
float P[2][2] = {0};    // Error covariance matrix

// Time management
unsigned long prevTime = 0;
float deltaTime = 0;

// Function to initialize MPU6050
void initMPU6050() {
    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1); // Access power management register
    Wire.write(0x00);       // Wake up MPU6050
    Wire.endTransmission();
}

// Function to read raw data from MPU6050
void readMPU6050RawData(int16_t &accX, int16_t &accY, int16_t &accZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H); // Start with accelerometer registers
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true); // Request 14 bytes (6 accel + 6 gyro + 2 temp)
    
    accX = (Wire.read() << 8) | Wire.read();
    accY = (Wire.read() << 8) | Wire.read();
    accZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature registers
    gyroX = (Wire.read() << 8) | Wire.read();
    gyroY = (Wire.read() << 8) | Wire.read();
    gyroZ = (Wire.read() << 8) | Wire.read();
}

// Kalman Filter Function
float kalmanFilter(float accAngle, float gyroRate) {
    // Prediction step
    angle += (gyroRate - bias) * deltaTime;
    P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= deltaTime * P[1][1];
    P[1][0] -= deltaTime * P[1][1];
    P[1][1] += Q_gyro * deltaTime;

    // Update step
    float y = accAngle - angle;
    float S = P[0][0] + R_angle;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

// Function to process MPU6050 data and apply Kalman Filter
void processMPU6050Data() {
    int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
    
    // Read raw data
    readMPU6050RawData(accX, accY, accZ, gyroX, gyroY, gyroZ);

    // Convert accelerometer data to angles
    float ax = accX / 16384.0;
    float ay = accY / 16384.0;
    float az = accZ / 16384.0;
    float accAngleX = atan2(ay, az) * 180 / M_PI;

    // Convert gyroscope data to rates
    float gyroRateX = gyroX / 131.0;

    // Calculate delta time
    unsigned long currentTime = micros();
    deltaTime = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    // Apply Kalman Filter
    float filteredAngleX = kalmanFilter(accAngleX, gyroRateX);

    // Print results
    Serial.print("Accel Angle: "); Serial.print(accAngleX);
    Serial.print(", Filtered Angle: "); Serial.println(filteredAngleX);

    // Serial.print("Raw Acc: X=");
    // Serial.print(accX);
    // Serial.print(", Y=");
    // Serial.print(accY);
    // Serial.print(", Z=");
    // Serial.println(accZ);

    // Serial.print("Raw Gyro: X=");
    // Serial.print(gyroX);
    // Serial.print(", Y=");
    // Serial.print(gyroY);
    // Serial.print(", Z=");
    // Serial.println(gyroZ);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    initMPU6050();
    prevTime = micros();
}

void loop() {
    processMPU6050Data();
    delay(10);
}

  #elif  Test_project == test_code_dummy_vale

  // Kalman filter variables
float angle = 0.0;              // Filtered angle
float bias = 0.0;               // Gyroscope bias
float P[2][2] = { { 1, 0 }, { 0, 1 } }; // Covariance matrix
float Q_angle = 0.001;          // Process noise variance for the angle
float Q_gyro = 0.003;           // Process noise variance for the gyroscope bias
float R_angle = 0.03;           // Measurement noise variance
float deltaTime = 0.01;         // Time step (seconds)

// Kalman filter function
float kalmanFilter(float accAngle, float gyroRate) {
    // Prediction step
    angle += (gyroRate - bias) * deltaTime;
    P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= deltaTime * P[1][1];
    P[1][0] -= deltaTime * P[1][1];
    P[1][1] += Q_gyro * deltaTime;

    // Update step
    float y = accAngle - angle; // Innovation
    float S = P[0][0] + R_angle; // Innovation covariance
    float K[2]; // Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * y;
    bias += K[1] * y;

    // Update the error covariance matrix
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void setup() {
    Serial.begin(115200); // Initialize serial communication
    Serial.println("Starting Kalman filter test...");
}

void loop() {
    // Generate dummy accelerometer angle and gyro rate
    float accAngle = 30.0 * sin(millis() / 1000.0); // Simulated accelerometer angle
    float gyroRate = 10.0 * cos(millis() / 1000.0); // Simulated gyroscope rate

    // Calculate filtered angle using Kalman filter
    float filteredAngle = kalmanFilter(accAngle, gyroRate);

    // Print data for Serial Plotter (tab-separated values)
    Serial.print(accAngle);        // First value: accelerometer angle
    Serial.print("\t");            // Tab separator
    Serial.print(gyroRate);        // Second value: gyroscope rate
    Serial.print("\t");            // Tab separator
    Serial.println(filteredAngle); // Third value: filtered angle (end line)

    delay(10); // Delay to match the simulation time step
}




  #endif
#elif project == MPU_RTOS

// #include <Wire.h>
// #include <Arduino.h>


// #define MPU6050_ADDR 0x68

// // Struct to hold raw data
// struct MPUData {
//     int16_t accelX, accelY, accelZ;
//     int16_t gyroX, gyroY, gyroZ;
// };

// // Struct to hold filtered data
// struct FilteredData {
//     float accAngleX, accAngleY;
//     float gyroRateX, gyroRateY, gyroRateZ;
//     float gyroFilteredAngleX, gyroFilteredAngleY;
//     float kalmanFilteredAngleX, kalmanFilteredAngleY;
// };

// // Queues
// QueueHandle_t rawDataQueue;
// QueueHandle_t filteredDataQueue;

// // Kalman Filter Variables
// float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
// float angle = 0, bias = 0, P[2][2] = {0};

// // Gyroscope Bias
// float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// // Function Prototypes
// void MPU6050_Init();
// void readMPU6050RawData(MPUData &data);
// void calibrateGyro(float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ);
// float getAccelAngleX(const MPUData &data);
// float getAccelAngleY(const MPUData &data);
// float kalmanFilter(float newAngle, float newRate, float deltaTime);
// float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bias, float alpha = 0.98);

// // Tasks
// void sensorTask(void *pvParameters);
// void filterTask(void *pvParameters);
// void displayTask(void *pvParameters);

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();

//     MPU6050_Init();
//     calibrateGyro(gyroBiasX, gyroBiasY, gyroBiasZ);

//     // Create Queues
//     rawDataQueue = xQueueCreate(10, sizeof(MPUData));
//     filteredDataQueue = xQueueCreate(10, sizeof(FilteredData));

//     // Create Tasks
//     xTaskCreate(sensorTask, "Sensor Task", 12048, NULL, 1, NULL);
//     xTaskCreate(filterTask, "Filter Task", 12048, NULL, 1, NULL);
//     xTaskCreate(displayTask, "Display Task", 12048, NULL, 1, NULL);

//     vTaskStartScheduler();
// }

// void loop() {
//     // FreeRTOS handles loop in tasks
// }

// // MPU6050 Initialization
// void MPU6050_Init() {
//     Wire.beginTransmission(MPU6050_ADDR);
//     Wire.write(0x6B); // Power Management 1
//     Wire.write(0);    // Wake up MPU6050
//     Wire.endTransmission(true);

//     Wire.beginTransmission(MPU6050_ADDR);
//     Wire.write(0x1B); // Gyroscope configuration
//     Wire.write(0x00); // ±250°/s
//     Wire.endTransmission(true);

//     Wire.beginTransmission(MPU6050_ADDR);
//     Wire.write(0x1C); // Accelerometer configuration
//     Wire.write(0x00); // ±2g
//     Wire.endTransmission(true);
// }

// // Read Raw Data
// void readMPU6050RawData(MPUData &data) {
//     Wire.beginTransmission(MPU6050_ADDR);
//     Wire.write(0x3B); // Starting register
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU6050_ADDR, 14);

//     data.accelX = Wire.read() << 8 | Wire.read();
//     data.accelY = Wire.read() << 8 | Wire.read();
//     data.accelZ = Wire.read() << 8 | Wire.read();
//     Wire.read(); Wire.read(); // Skip Temp
//     data.gyroX = Wire.read() << 8 | Wire.read();
//     data.gyroY = Wire.read() << 8 | Wire.read();
//     data.gyroZ = Wire.read() << 8 | Wire.read();
// }

// // Calibrate Gyroscope
// void calibrateGyro(float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ) {
//     MPUData data;
//     long biasX = 0, biasY = 0, biasZ = 0;
//     const int samples = 500;

//     for (int i = 0; i < samples; i++) {
//         readMPU6050RawData(data);
//         biasX += data.gyroX;
//         biasY += data.gyroY;
//         biasZ += data.gyroZ;
//         delay(2);
//     }

//     gyroBiasX = biasX / (samples * 131.0);
//     gyroBiasY = biasY / (samples * 131.0);
//     gyroBiasZ = biasZ / (samples * 131.0);
// }

// // Get Accelerometer Angles
// float getAccelAngleX(const MPUData &data) {
//     return atan2(data.accelY, sqrt(data.accelX * data.accelX + data.accelZ * data.accelZ)) * RAD_TO_DEG;
// }
// float getAccelAngleY(const MPUData &data) {
//     return atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ)) * RAD_TO_DEG;
// }

// // Kalman Filter
// float kalmanFilter(float newAngle, float newRate, float deltaTime) {
//     float rate = newRate - bias;
//     angle += deltaTime * rate;

//     P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
//     P[0][1] -= deltaTime * P[1][1];
//     P[1][0] -= deltaTime * P[1][1];
//     P[1][1] += Q_bias * deltaTime;

//     float S = P[0][0] + R_measure;
//     float K[2];
//     K[0] = P[0][0] / S;
//     K[1] = P[1][0] / S;

//     float y = newAngle - angle;
//     angle += K[0] * y;
//     bias += K[1] * y;

//     float P00_temp = P[0][0], P01_temp = P[0][1];
//     P[0][0] -= K[0] * P00_temp;
//     P[0][1] -= K[0] * P01_temp;
//     P[1][0] -= K[1] * P00_temp;
//     P[1][1] -= K[1] * P01_temp;

//     return angle;
// }

// // Complementary Filter
// float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bias, float alpha) {
//     static float gyroAngle = 0;
//     gyroRate -= bias;
//     gyroAngle += gyroRate * deltaTime;
//     return alpha * gyroAngle + (1 - alpha) * accAngle;
// }

// // Sensor Task
// void sensorTask(void *pvParameters) {
//     MPUData rawData;
//     while (true) {
//         readMPU6050RawData(rawData);
//         xQueueSend(rawDataQueue, &rawData, portMAX_DELAY);
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// // Filter Task
// void filterTask(void *pvParameters) {
//     MPUData rawData;
//     FilteredData filteredData;
//     unsigned long prevTime = micros();

//     while (true) {
//         if (xQueueReceive(rawDataQueue, &rawData, portMAX_DELAY)) {
//             unsigned long currentTime = micros();
//             float deltaTime = (currentTime - prevTime) / 1000000.0;
//             prevTime = currentTime;

//             filteredData.accAngleX = getAccelAngleX(rawData);
//             filteredData.accAngleY = getAccelAngleY(rawData);

//             filteredData.gyroRateX = rawData.gyroX / 131.0;
//             filteredData.gyroRateY = rawData.gyroY / 131.0;
//             filteredData.gyroRateZ = rawData.gyroZ / 131.0;

//             filteredData.gyroFilteredAngleX = complementaryFilter(
//                 filteredData.gyroRateX, filteredData.accAngleX, deltaTime, gyroBiasX
//             );

//             filteredData.kalmanFilteredAngleX = kalmanFilter(
//                 filteredData.accAngleX, filteredData.gyroRateX, deltaTime
//             );

//             xQueueSend(filteredDataQueue, &filteredData, portMAX_DELAY);
//         }
//     }
// }

// // Display Task
// void displayTask(void *pvParameters) {
//     FilteredData filteredData;
//     while (true) {
//         if (xQueueReceive(filteredDataQueue, &filteredData, portMAX_DELAY)) {
//             Serial.print("Accel Angle: ");
//             Serial.print(filteredData.accAngleX, 2);
//             Serial.print(", Kalman Filtered Angle: ");
//             Serial.print(filteredData.kalmanFilteredAngleX, 2);
//             Serial.print(", Gyro Filtered Angle: ");
//             Serial.println(filteredData.gyroFilteredAngleX, 2);
//         }
//     }
// }


#include <Wire.h>
#include <Arduino.h>

#define MPU6050_ADDR 0x68

// Struct to hold raw data
struct MPUData 
{
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
};

// Struct to hold filtered data
struct FilteredData 
{
    float accAngleX, accAngleY , accAngleZ;
    float gyroRateX, gyroRateY, gyroRateZ;
    float gyroFilteredAngleX, gyroFilteredAngleY, gyroFilteredAngleZ;
    float kalmanFilteredAngleX, kalmanFilteredAngleY , kalmanFilteredAngleZ;
};

// Queues
QueueHandle_t rawDataQueue; // data that take from sensor ( put in queue to comm with another task to make it filtered )
QueueHandle_t filteredDataQueue;

// Kalman Filter Variables
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.05;
float angle = 0, bias = 0, P[2][2] = {0};

// Gyroscope Bias
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Moving Average Buffers
#define BUFFER_SIZE 10
float gyroBufferX[BUFFER_SIZE] = {0};
int bufferIndex = 0;

// Function Prototypes
void MPU6050_Init();
void readMPU6050RawData(MPUData &data);
void calibrateGyro(float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ);
float getAccelAngleX(const MPUData &data);
float getAccelAngleY(const MPUData &data);
float getAccelAngleZ(const MPUData &data);
float kalmanFilter(float newAngle, float newRate, float deltaTime);
float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bias, float alpha = 0.95);
float movingAverage(float newValue, float buffer[], int size);

// Tasks
void sensorTask(void *pvParameters);
void filterTask(void *pvParameters);
void displayTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    MPU6050_Init();
    calibrateGyro(gyroBiasX, gyroBiasY, gyroBiasZ);

    // Create Queues
    rawDataQueue = xQueueCreate(10, sizeof(MPUData));
    filteredDataQueue = xQueueCreate(10, sizeof(FilteredData));

    // Create Tasks
    xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, NULL);
    xTaskCreate(filterTask, "Filter Task", 2048, NULL, 1, NULL);
    xTaskCreate(displayTask, "Display Task", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() {
    // FreeRTOS handles loop in tasks
}

// MPU6050 Initialization
void MPU6050_Init() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // Power Management 1
    Wire.write(0);    // Wake up MPU6050
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // Gyroscope configuration
    Wire.write(0x00); // ±250°/s
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); // Accelerometer configuration
    Wire.write(0x00); // ±2g
    Wire.endTransmission(true);
}

// Read Raw Data
void readMPU6050RawData(MPUData &data) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Starting register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    data.accelX = Wire.read() << 8 | Wire.read();
    data.accelY = Wire.read() << 8 | Wire.read();
    data.accelZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Skip Temp
    data.gyroX = Wire.read() << 8 | Wire.read();
    data.gyroY = Wire.read() << 8 | Wire.read();
    data.gyroZ = Wire.read() << 8 | Wire.read();
}

// Calibrate Gyroscope
void calibrateGyro(float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ) {
    MPUData data;
    long biasX = 0, biasY = 0, biasZ = 0;
    const int samples = 500;

    for (int i = 0; i < samples; i++) {
        readMPU6050RawData(data);
        biasX += data.gyroX;
        biasY += data.gyroY;
        biasZ += data.gyroZ;
        delay(2);
    }

    gyroBiasX = biasX / (samples * 131.0);
    gyroBiasY = biasY / (samples * 131.0);
    gyroBiasZ = biasZ / (samples * 131.0);
}

// Get Accelerometer Angles
float getAccelAngleX(const MPUData &data) 
{
    return atan2(data.accelY, sqrt(data.accelX * data.accelX + data.accelZ * data.accelZ)) * RAD_TO_DEG;
}
float getAccelAngleY(const MPUData &data) 
{
    return atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ)) * RAD_TO_DEG;
}

float getAccelAngleZ(const MPUData &data) 
{
    return atan2(sqrt(data.accelX * data.accelX + data.accelY * data.accelY), data.accelZ) * RAD_TO_DEG;
}



// Kalman Filter
float kalmanFilter(float newAngle, float newRate, float deltaTime) 
{
  //Predict the angle and bias:
    float rate = newRate - bias;
    angle += deltaTime * rate;

    // Update estimation error covariance - Project the error covariance ahead:
    P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= deltaTime * P[1][1];
    P[1][0] -= deltaTime * P[1][1];
    P[1][1] += Q_bias * deltaTime;

    // Calculate Kalman gain
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    //Update the angle and bias with the measurement 
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    //pdate the error covariance matrix
    float P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

// Complementary Filter
float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bias, float alpha) 
{
    static float gyroAngle = 0;
    gyroRate -= bias;
    gyroAngle += gyroRate * deltaTime;
    return alpha * gyroAngle + (1 - alpha) * accAngle;
}

// Moving Average Filter
float movingAverage(float newValue, float buffer[], int size) 
{
    buffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % size;

    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

// Sensor Task
void sensorTask(void *pvParameters) 
{
    MPUData rawData;
    while (true) {
        readMPU6050RawData(rawData);
        xQueueSend(rawDataQueue, &rawData, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Filter Task
void filterTask(void *pvParameters) 
{
    MPUData rawData; // read the data that come from sensor
    FilteredData filteredData; // save data after filter and fill the queue
    unsigned long prevTime = micros(); // used for calculate delta time 

    while (true) {
        if (xQueueReceive(rawDataQueue, &rawData, portMAX_DELAY)) 
        {
            // Delta time calculation 
            unsigned long currentTime = micros();
            float deltaTime = (currentTime - prevTime) / 1000000.0;
            prevTime = currentTime;

            // Calculate accelerometer angles
            filteredData.accAngleX = getAccelAngleX(rawData);
            filteredData.accAngleY = getAccelAngleY(rawData);
            filteredData.accAngleZ = getAccelAngleZ(rawData);

            // Calculate gyroscope rates (bias corrected)
            filteredData.gyroRateX = rawData.gyroX / 131.0 - gyroBiasX;
            filteredData.gyroRateY = rawData.gyroY / 131.0 - gyroBiasY;
            filteredData.gyroRateZ = rawData.gyroZ / 131.0 - gyroBiasZ;

            // Apply complementary filter
            // Complementary Filter for x-Axis
            filteredData.gyroFilteredAngleX = complementaryFilter( 
                filteredData.gyroRateX,
                filteredData.accAngleX,
                deltaTime,
                gyroBiasX
            );

            //// Complementary Filter for Y-Axis
            filteredData.gyroFilteredAngleY = complementaryFilter(
                filteredData.gyroRateY,
                filteredData.accAngleY,
                deltaTime,
                gyroBiasY
            );

            // Complementary Filter for Z-Axis
            filteredData.gyroFilteredAngleZ = complementaryFilter(
                filteredData.gyroRateZ,
                filteredData.accAngleZ,
                deltaTime,
                gyroBiasZ
            );

            // Apply Kalman filter
            filteredData.kalmanFilteredAngleX = kalmanFilter(
                filteredData.accAngleX,
                filteredData.gyroRateX,
                deltaTime
            );

            filteredData.kalmanFilteredAngleY = kalmanFilter(
                filteredData.accAngleY,
                filteredData.gyroRateY,
                deltaTime
            );

            filteredData.kalmanFilteredAngleZ = kalmanFilter(

                filteredData.accAngleZ, 
                filteredData.gyroRateZ, 
                deltaTime

            );

            // Send filtered data to the queue
            xQueueSend(filteredDataQueue, &filteredData, portMAX_DELAY);
        }
    }
}

// Display Task

// Display Task
void displayTask(void *pvParameters) {
    FilteredData filteredData;
    while (true) {
        if (xQueueReceive(filteredDataQueue, &filteredData, portMAX_DELAY)) 
        {
          //  before filtered
            // Serial.print("AccAngleX: ");
            // Serial.print(filteredData.accAngleX);
            // Serial.print(" | AccAngleY: ");
            // Serial.print(filteredData.accAngleY);
            // Serial.print(" | AccAngleZ: ");
            // Serial.print(filteredData.accAngleZ);

          //  after filtered
            Serial.print(" | KalmanAngleX: ");
            Serial.print(filteredData.kalmanFilteredAngleX);
            Serial.print(" | KalmanAngleY: ");
            Serial.print(filteredData.kalmanFilteredAngleY);
            Serial.print(" | KalmanAngleZ: ");
            Serial.print(filteredData.kalmanFilteredAngleZ);

            Serial.print(" | GyroRateX: ");
            Serial.print(filteredData.gyroRateX);
            Serial.print(" | GyroRateY: ");
            Serial.println(filteredData.gyroRateY);
            // Serial.print(" | GyroFilteredAngleZ: ");
            // Serial.println(filteredData.gyroFilteredAngleZ);
        }
    }
}

#elif project == MPU_RTOS_velocity


#include <Wire.h>
#include <Arduino.h>

#define MPU6050_ADDR 0x68

// Struct to hold raw data
struct MPUData {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
};

// Struct to hold filtered data
struct FilteredData {
    float accAngleX, accAngleY, accAngleZ;
    float gyroRateX, gyroRateY, gyroRateZ;
    float gyroFilteredAngleX, gyroFilteredAngleY, gyroFilteredAngleZ;
    float kalmanFilteredAngleX, kalmanFilteredAngleY, kalmanFilteredAngleZ;
    float velocityX, velocityY, velocityZ;  // New velocity fields
};

// Queues
QueueHandle_t rawDataQueue; // data that comes from sensor ( put in queue to communicate with another task to filter )
QueueHandle_t filteredDataQueue;

// Kalman Filter Variables
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.05;
float angle = 0, bias = 0, P[2][2] = {0};

// Gyroscope Bias
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Moving Average Buffers
#define BUFFER_SIZE 20  // Increased buffer size for more aggressive filtering
float accelBufferX[BUFFER_SIZE] = {0};
float accelBufferY[BUFFER_SIZE] = {0};
float accelBufferZ[BUFFER_SIZE] = {0};
int bufferIndex = 0;

// Function Prototypes
void MPU6050_Init();
void readMPU6050RawData(MPUData &data);
void calibrateGyro(float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ);
float getAccelAngleX(const MPUData &data);
float getAccelAngleY(const MPUData &data);
float getAccelAngleZ(const MPUData &data);
float kalmanFilter(float newAngle, float newRate, float deltaTime);
float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bias, float alpha = 0.95);
float movingAverage(float newValue, float buffer[], int size);
float exponentialMovingAverage(float newValue, float oldValue, float alpha);

// Tasks
void sensorTask(void *pvParameters);
void filterTask(void *pvParameters);
void displayTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    MPU6050_Init();
    calibrateGyro(gyroBiasX, gyroBiasY, gyroBiasZ);

    // Create Queues
    rawDataQueue = xQueueCreate(10, sizeof(MPUData));
    filteredDataQueue = xQueueCreate(10, sizeof(FilteredData));

    // Create Tasks
    xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, NULL);
    xTaskCreate(filterTask, "Filter Task", 2048, NULL, 1, NULL);
    xTaskCreate(displayTask, "Display Task", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() {
    // FreeRTOS handles loop in tasks
}

// MPU6050 Initialization
void MPU6050_Init() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // Power Management 1
    Wire.write(0);    // Wake up MPU6050
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // Gyroscope configuration
    Wire.write(0x00); // ±250°/s
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); // Accelerometer configuration
    Wire.write(0x00); // ±2g
    Wire.endTransmission(true);
}

// Read Raw Data
void readMPU6050RawData(MPUData &data) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Starting register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    data.accelX = Wire.read() << 8 | Wire.read();
    data.accelY = Wire.read() << 8 | Wire.read();
    data.accelZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Skip Temp
    data.gyroX = Wire.read() << 8 | Wire.read();
    data.gyroY = Wire.read() << 8 | Wire.read();
    data.gyroZ = Wire.read() << 8 | Wire.read();
}

// Calibrate Gyroscope
void calibrateGyro(float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ) {
    MPUData data;
    long biasX = 0, biasY = 0, biasZ = 0;
    const int samples = 500;

    for (int i = 0; i < samples; i++) {
        readMPU6050RawData(data);
        biasX += data.gyroX;
        biasY += data.gyroY;
        biasZ += data.gyroZ;
        delay(2);
    }

    gyroBiasX = biasX / (samples * 131.0);
    gyroBiasY = biasY / (samples * 131.0);
    gyroBiasZ = biasZ / (samples * 131.0);
}

// Get Accelerometer Angles
float getAccelAngleX(const MPUData &data) {
    return atan2(data.accelY, sqrt(data.accelX * data.accelX + data.accelZ * data.accelZ)) * RAD_TO_DEG;
}

float getAccelAngleY(const MPUData &data) {
    return atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ)) * RAD_TO_DEG;
}

float getAccelAngleZ(const MPUData &data) {
    return atan2(sqrt(data.accelX * data.accelX + data.accelY * data.accelY), data.accelZ) * RAD_TO_DEG;
}

// Kalman Filter
float kalmanFilter(float newAngle, float newRate, float deltaTime) {
    // Predict the angle and bias:
    float rate = newRate - bias;
    angle += deltaTime * rate;

    // Update estimation error covariance - Project the error covariance ahead:
    P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= deltaTime * P[1][1];
    P[1][0] -= deltaTime * P[1][1];
    P[1][1] += Q_bias * deltaTime;

    // Calculate Kalman gain
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update the angle and bias with the measurement
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Update the error covariance matrix
    float P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

// Complementary Filter
float complementaryFilter(float gyroRate, float accAngle, float deltaTime, float bias, float alpha) {
    static float gyroAngle = 0;
    gyroRate -= bias;
    gyroAngle += gyroRate * deltaTime;
    return alpha * gyroAngle + (1 - alpha) * accAngle;
}

// Moving Average Filter
float movingAverage(float newValue, float buffer[], int size) {
    buffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % size;

    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

// Exponential Moving Average Filter
float exponentialMovingAverage(float newValue, float oldValue, float alpha) {
    return alpha * newValue + (1 - alpha) * oldValue;
}

// Sensor Task
void sensorTask(void *pvParameters) {
    MPUData rawData;
    while (true) {
        readMPU6050RawData(rawData);
        xQueueSend(rawDataQueue, &rawData, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10)); // Reduce delay to 10ms
    }
}

// Filter Task
void filterTask(void *pvParameters) 
{
    MPUData rawData; // Read the data that comes from sensor
    FilteredData filteredData; // Save data after filtering and send to queue
    unsigned long prevTime = micros(); // Used for calculating delta time
    float velocityX = 0, velocityY = 0, velocityZ = 0; // Velocity variables
    float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0; // Previous accelerometer values
    const float alpha = 0.05; // Increased smoothing factor for EMA

    while (true) {
        if (xQueueReceive(rawDataQueue, &rawData, portMAX_DELAY)) {
            // Delta time calculation
            unsigned long currentTime = micros();
            float deltaTime = (currentTime - prevTime) / 1000000.0;
            if (deltaTime <= 0) {
                deltaTime = 0.01; // Default to 10ms if deltaTime is invalid
            }
            prevTime = currentTime;

            // Apply filters for angles
            filteredData.accAngleX = getAccelAngleX(rawData);
            filteredData.accAngleY = getAccelAngleY(rawData);
            filteredData.accAngleZ = getAccelAngleZ(rawData);

            filteredData.gyroRateX = rawData.gyroX / 131.0;
            filteredData.gyroRateY = rawData.gyroY / 131.0;
            filteredData.gyroRateZ = rawData.gyroZ / 131.0;

            // Kalman filter for angles
            filteredData.gyroFilteredAngleX = kalmanFilter(filteredData.accAngleX, filteredData.gyroRateX, deltaTime);
            filteredData.gyroFilteredAngleY = kalmanFilter(filteredData.accAngleY, filteredData.gyroRateY, deltaTime);
            filteredData.gyroFilteredAngleZ = kalmanFilter(filteredData.accAngleZ, filteredData.gyroRateZ, deltaTime);

            // Apply exponential moving average filter to accelerometer data
            float accelX = exponentialMovingAverage(rawData.accelX / 16384.0, prevAccelX, alpha);
            float accelY = exponentialMovingAverage(rawData.accelY / 16384.0, prevAccelY, alpha);
            float accelZ = exponentialMovingAverage(rawData.accelZ / 16384.0, prevAccelZ, alpha);

            prevAccelX = accelX;
            prevAccelY = accelY;
            prevAccelZ = accelZ;

            // Bias correction (assuming the sensor is stationary at the start)
            static float accelBiasX = accelX;
            static float accelBiasY = accelY;
            static float accelBiasZ = accelZ;

            accelX -= accelBiasX;
            accelY -= accelBiasY;
            accelZ -= accelBiasZ;

            // Velocity calculation (integrating accelerometer data)
            velocityX += accelX * deltaTime;
            velocityY += accelY * deltaTime;
            velocityZ += accelZ * deltaTime;

            // Drift correction (simple example, may need more sophisticated approach)
            velocityX *= 0.99;
            velocityY *= 0.99;
            velocityZ *= 0.99;

            filteredData.velocityX = velocityX;
            filteredData.velocityY = velocityY;
            filteredData.velocityZ = velocityZ;

            // Send filtered data to queue
            xQueueSend(filteredDataQueue, &filteredData, portMAX_DELAY);
        }
    }
}

// Display Task
void displayTask(void *pvParameters) {
    FilteredData filteredData;
    while (true) {
        if (xQueueReceive(filteredDataQueue, &filteredData, portMAX_DELAY)) {
            // Display data (printing or LED output)
            Serial.print(" AX = ");
            Serial.print(filteredData.accAngleX);
            Serial.print(" AY = ");
            Serial.print(filteredData.accAngleY);
            Serial.print(" AZ = ");
            Serial.print(filteredData.accAngleZ);
            
            Serial.print(" GX = ");
            Serial.print(filteredData.gyroFilteredAngleX);
            Serial.print(" GY = ");
            Serial.print(filteredData.gyroFilteredAngleY);
            Serial.print(" GZ = ");
            Serial.print(filteredData.gyroFilteredAngleZ);
            
            Serial.print(" VX = ");
            Serial.print(filteredData.velocityX);
            Serial.print(" VY = ");
            Serial.print(filteredData.velocityY);
            Serial.print(" VZ = ");
            Serial.println(filteredData.velocityZ);

            // delay(500);  // Slow down serial output
        }
    }
}
#elif project == RTC_DS1307_test


#include <Wire.h>
#include <RTClib.h>

RTC_DS1307 rtc;

void setup() {
  Serial.begin(115200);
  if (!rtc.begin()) {
    Serial.println("RTC initialization failed!");
    while (1);
  }
  else
  {
    Serial.println("RTC initialization done.");
  }
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    rtc.adjust(DateTime(2025, 1, 5, 12, 0, 0));
  }
  
}

void loop() {
  DateTime now = rtc.now();
  if (!rtc.isrunning()) 
  {
    Serial.println("RTC is NOT running!");
  }
  else
  {
     Serial.println("RTC is  running perfect ");
  }
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  delay(1000);
}

#elif project == low_power_modes

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(100); // Allow time for Serial output to initialize
  
  // Check the reason for wake-up
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }

  // Configure wake-up sources
  esp_sleep_enable_timer_wakeup(10 * 1000000); // Wake up after 10 seconds
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0); // Wake up on LOW signal at GPIO 4

  Serial.println("Entering deep sleep...");
  delay(100); // Allow time for Serial output to flush
  esp_deep_sleep_start();
}

void loop() {
  // The loop will not run due to deep sleep behavior
}
#elif  project == Test_json_serial

#include <Arduino.h>
#include <ArduinoJson.h>
void setup()
{
   JsonDocument doc;


    String input =
      "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
  deserializeJson(doc, input);
  doc["sensor"] = "test";
  doc["value"] = 10;

  // You can use a String as a key to get a member from JsonDocument
  // No duplication is done.
  long time = doc[String("time")];

  String json_to_string ;
  serializeJson(doc, json_to_string);
  Serial.begin(115200);

  Serial.println(json_to_string);
  // deserializeJson(doc, json_to_string);
  // doc.clear();
  // deserializeJson(doc, json_to_string);

  // Serial.println(json_to_string);
  // Serial.println(json_to_string);

  Serial.println(doc["sensor"].as<String>());
  Serial.println(doc["value"].as<int>());
  Serial.println(doc["time"].as<long>());



}

void loop()
{
  int static i=0;
   String input =
      "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
      deserializeJson(doc, input);

      if(i%2 ==0 )
      {
        doc["sensor"] = "test";
        doc["value"] = 10;  
      }
      else
      {
        doc["sensor"] = "test2";
        doc["value"] = 20;  
      }
      String json_to_string;
      serializeJson(doc, json_to_string);
      Serial.println(json_to_string);
      // doc["value"] = 10;
      Serial.println(doc["sensor"].as<String>());
      Serial.println(doc["value"].as<int>());
      Serial.println(doc["time"].as<long>());
      i++;
      Serial.println(i);
      delay(1000);
}

#endif

// void setup() {

//   // add the value to 
//   // Populate JSON document with sensor values ( this value now for testing )
// /*********************************************************************************/
//   doc["counter"] = counter; // 
//   doc["readingsmall"] = small_value;
//   doc["readingMosquitoes"] = medium_value;
//   doc["readingLarg"] = large_value;
//   doc["readingFly"] = fly_value;
//   doc["BigBattery"] = String(big_battery_percent);           // Convert int to String
//   doc["SmallBattery"] = String(small_battery_percent);       // Convert int to String
//   doc["readingTempIn"] = String(temperature_in);             // Convert int to String
//   doc["serlNum"] = serial_number;
//   doc["readingTempOut"] = temperature_out ? "true" : "false";
//   doc["readingHumidity"] = String(humidity);                 // Convert int to String
//   doc["readingDate"] = date;
//   doc["readingTime"] = readTime;                             // Use renamed variable
//   doc["readingLat"] = latitude;
//   doc["readingLng"] = longitude;
//   doc["readingWindSpeed"] = String(wind_speed);              // Convert int to String
//   doc["co2"] = String(co2_level);                            // Convert int to String
//   doc["co2Val"] = String(co2_value);                         // Convert int to String
//   doc["isDone"] = is_done ? "true" : "false";
//   doc["isClean"] = is_clean ? "true" : "false";
//   doc["model"] = model_value;

//   // Serialize JSON to string this need to use to send to url2 ^_9
//   String json_to_string ;
//   serializeJson(doc, json_to_string);

//   // then take this str and put it inside the 
  
// /********************************************************************************************************/

//   //then we need to convert the json to string for sending 

//   Serial.begin(115200);   // this for testing

//   SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX); // select the serial (uart that be used)

//   //SerialAT.write("");
//   Serial.println("Starting GSM7600 HTTP JSON Test");
//   // SIM7600ConnectionGPRS ();
//   // SIM7600_Check_GPRS_Conn();
//   // SIM7600_modem_details();
//   // SIM7600_check_signal_quality();
//   // Prepare a JSON document to store response
//   delay(1000);
//  // setupsim7600();
//   // setup_HTTP();
//   //sendPostRequest_1111(test_message_send);
//   // delay(1000);
//   // Call function to get data from server
//   // GSM7600_Get_Data_from_Server_update(URLTEST);
//   // take_Action_and_parse(&received_data);
//   //received_data="";
//   // GSM7600_Send_Data_to_Server_update(URL2, json_to_string);
//   // take_Action_and_parse_for_sending(&json_to_string);
//   // SerialAT.println(received_data);
//   // setupsim7600();  // Function to initialize the SIM7600
//   // GSM7600_Get_Data_from_Server(URL);  // Call function to get data from server
//   // String GPS_op_parse ="" ;
//   // GPS_Enable() ;
//   // GPS_GETDATA(&GPS_op_parse);
//   // GPS_Disable();

// }

// void loop() 
// {
//   //  SerialAT.write("hi");
//   //  delay(10000);
//   //  transCoordinates();
//   // String GPS_op_parse ="" ;
//   // GPS_GETDATA(&GPS_op_parse);
//   // delay(5000);

//   // GPS_Disable();

// // use to write to serial (any command or send data by uart)
//   if (Serial.available()) 
//   {
//     command += Serial.readStringUntil('\n');  // Read command from PuTTY
//     Serial.println("Sent to SIM7600: " + command);  // Print command to Serial Monitor for confirmation
//     SerialAT.println(command);                      // Send command to SIM7600
//       // Handle JSON or other data processing
//       processJSONData(&command , &saved_json );
//       Serial.print("out side the func : ");
//       Serial.println(saved_json) ;
//   }

//    // Check for data from SIM7600 (SerialAT) and send it to PuTTY (Serial)
//    if (SerialAT.available()) {
//       String command = SerialAT.readStringUntil('\n');  // Read response from SIM7600
//       Serial.println("Received from SIM7600: " + command);  // Print response to Serial Monitor
//           SerialAT.println(command);                      // Send command to SIM7600
//       // Handle JSON or other data processing
//       processJSONData(&command , &saved_json );
//       Serial.print("out side the func2 : ");
//       Serial.println(saved_json) ;
//    }
//   //delay(10000);  // Wait for 10 seconds
//   //GSM7600_Get_Data_from_Server(URL);  // Call function to get data from server
//   // setupsim7600();
// }




