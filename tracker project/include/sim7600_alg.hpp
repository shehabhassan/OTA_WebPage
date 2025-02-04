#ifndef SIM7600_ALG_H
#define SIM7600_ALG_H

// #include <Arduino.h>  // Include Arduino core library for access to Arduino functions
// #include <SoftwareSerial.h>
// #include <TinyGsmClient.h>

// Declare Serial for AT communication with the SIM7600
// extern SoftwareSerial SerialAT;  // Declare but do not define here
//extern TinyGsm modem;             // Declare the modem

#include <HardwareSerial.h>

#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 17
#define MODEM_RX 16

// Define Serial port for SIM7600
// HardwareSerial SerialAT(2);  // Use UART1 for the modem

// // Declare global variables



typedef struct 
{
  uint8_t init_flag ; 

} ST_GSM_Flags;


// Function declarations (prototypes)
void initializeSIM7600();
void checkNetworkStatus();

/*
  // Wait until the module is ready to accept AT commands for 10 sec
  // AT+CSQ This command sends a query to the SIM800L module, requesting it to report the current signal quality.
  // "AT+CREG?"  This command queries the GSM network registration status of the SIM800L module
  // process of configuring GPRS (General Packet Radio Service) on the SIM800L module
  // Setting the APN: Using the AT+SAPBR command to configure the GPRS bearer profile.
  // Starting the GPRS: Using AT+CGATT=1 to attach to the GPRS service.

*/


void SIM7600ConnectionGPRS ();
void SIM7600_Check_GPRS_Conn();
void SIM7600_modem_details();

void setupModuleGSM7600() ; 
void GSM7600_Get_Data_from_Server(const char* URL);
void SIM7600_check_signal_quality();
int parseSignalStrength(String response);
/********************************************************************************/
void setup_HTTP();
void GSM7600_Get_Data_from_Server_update(const char* URL );
int parseHTTPResponse(const String &buffer) ;
void take_Action_and_parse(String * response);

void GSM7600_Send_Data_to_Server_update(const char* URL  ,String message_send);
void take_Action_and_parse_for_sending(String * response);
void GPS_Enable() ; 
void GPS_GETDATA(String * GPS_OP);
void GPS_Disable();
/********************************************************************************/

void GSM7600_send_Data_to_Server(const char* URL ,JsonObject& doc ,const char * data_send);

void printSerialResponse(); // helper func
void sendATcommand (const char* ATcommand, unsigned int timeout);
void sendPostRequest_1111(String jsonMessage);
#endif
