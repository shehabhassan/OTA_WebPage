#include "WString.h"
// #include "HardwareSerial.h"
// #include <Arduino.h>

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "../include/sim7600_alg.hpp"
#define TINY_GSM_MODEM_SIM7600

#include <TinyGsmClient.h>
// #include <SoftwareSerial.h>
extern char Serialnumber[];
extern const char* jsonChunk1;
extern const char* jsonChunk2;
extern const char* jsonChunk3;
extern const char* jsonChunk4;
extern const char* jsonChunk5;
extern const char* jsonChunk6;
extern const char* jsonChunk7;
extern const char* jsonChunk8;
extern const char* jsonChunk9;
extern const char* jsonChunk10;
extern const char* jsonChunk11;
//char apn[30] = "internet.vodafone.net" ;  // APN for GPRS
// SIM7600 configuration
  
const char gprsUser[] = "";         // GPRS User, usually empty
const char gprsPass[] = "";         // GPRS Password, usually empty
HardwareSerial SerialAT(2);  // Use UART1 for the modem


const char apn[] = "internet.vodafone.net";  // APN for GPRS
const char URL[] =    "https://trapsysapi.epcmms.com/api/Traps/LoadData2"; //"https://trapsysapi.epcmms.com/api/Traps/LoadData2"; //"http://trapsnos.epcmms.com/api/Traps/LoadData2";  // For receiving data
const char URL2[] =  "https://trapsysapi.epcmms.com/api/Reading"; //"http://trapsnos.epcmms.com/api/Reading";         // For sending data
const char URLTEST[] = "https://trapsnos.epcmms.com/api/Traps/LoadData2";
const char URLTESTsend[] = "https://trapsnos.epcmms.com/api/Reading";
char Serialnumber[] = "EG-123-2020-098165";  // Example serial number




// global variable for flags error 
ST_GSM_Flags GSM7600 ;

//SoftwareSerial SerialAT(10, 11); // RX, TX

// we should create modem object for tinyGSM ---> look in github page of this module ^_^
TinyGsm modem(SerialAT);

// debug macro ( to print in the serial) u can remove the code will run normal 
#define DBG(x)    Serial.println(x)      

//const char Serialnumber[] = "\"EG-123-2020-098165\"";  // Your Serial Number

/****************************************************************************************************/
  uint8_t check[100] ;
 
  String  Path_Command , Content_Command , Json_Data_receive;
  String receive_response ; // this the output of receive the response 200 ,400 , 404 , 
  int response_parsed = 0 ; 
  String json_response;


    String responseData = "";


  uint8_t check2[100] ;
/****************************************************************************************************/
// this function is used for init baud rate
void initializeSIM7600()
{
  // init the communication baud rate 9600
  Serial.begin(9600) ;
}

void checkNetworkStatus()
{

  Serial.println("AT\r\n"); 
  // wait for 15 sec 
  delay(15000);
  /*
    AT+CREG?: This AT command asks the SIM module to return the current network registration status. The response will tell you whether the module is connected to a network, trying to connect, or not registered.
  */
  Serial.println("AT+CREG?"); 
  delay(1000); 
  while (Serial.available()) 
  {
    char creg =  Serial.read() ; 
    Serial.write(creg);  // Print network status
    
  }
  

} 

// this function is use to connect to GPRS 
void SIM7600ConnectionGPRS ()
{
  // delay one sec 
  delay(1000); 
  if (!modem.gprsConnect(apn,gprsUser,gprsPass))
  {
      // failed to connect to user 
      DBG("Failed to connect to GPRS"); 
      delay(10000); 
      return ; 
  }
}


// check the GPRS connection 
void SIM7600_Check_GPRS_Conn()
{
  // creat a var for get the connection status
  uint8_t status = modem.isGprsConnected(); 
  // print the status as connected or not
 // DBG("GPRS status = " , status ? "connected" : "not connected") ; // error due to the macros takes 1 arg
  DBG("GPRS status = ");
  DBG(status ? "connected" : "not connected");
  DBG(String(status));

}

int parseSignalStrength(String response) {
  // Look for the signal strength value in the response
  int index = response.indexOf("+CSQ:"); // Find the colon character
  if (index != -1) 
  {
    // Extract the part after the colon
    String value = response.substring(index + 6);
    value.trim(); // Remove any whitespace
     int commaIndex = value.indexOf(',');
      if (commaIndex != -1)
       {
        String rssi = value.substring(0, commaIndex);
        int signalStrength = rssi.toInt();  // Convert to integer
        
        // Display signal strength
        DBG("Signal Strength: ");
        DBG(signalStrength);
       }
    return value.toInt();
  }
  return -1;  // Return -1 if parsing failed
}


void SIM7600_check_signal_quality()
{
   // Send the AT command to check signal quality
  SerialAT.println("AT+CSQ") ; 
  delay(1000); 
  if(SerialAT.available())
  {
    // Check if there is data available from the SIM7600
    String Signal = SerialAT.readString() ;     // read the resp
    DBG(Signal);
    int signalStrength = parseSignalStrength(Signal) ;
    if (signalStrength>0)
    {
      DBG("serial strength : "); 
      DBG(signalStrength) ;
    }
  }
}

void SIM7600_modem_details()
 {
  String ccid = modem.getSimCCID();
  DBG("CCID: " + ccid);

  String imei = modem.getIMEI();
  DBG("IMEI: " + imei);

  String imsi = modem.getIMSI();
  DBG("IMSI: " + imsi);

  String cop = modem.getOperator();
  DBG("Operator: " + cop);

  // Convert the IPAddress to a human-readable format
  IPAddress local = modem.localIP();
  DBG("Local IP: " + String(local[0]) + "." + String(local[1]) + "." + String(local[2]) + "." + String(local[3]));
  
  int csq = modem.getSignalQuality();
  DBG("Signal quality: " );
  DBG(String(csq));
}



void setupsim7600()
{
  // init the baud rate as 115200 
  Serial.begin(115200); 
  SerialAT.begin(115200);

  DBG("init the  modem ...."); 

  // init the gsm to be ready
  Serial.write("AT\r\n");
  // delay for 10 sec to get response
  delay(1000);
  if(!modem.init())
  {
    // return there are a problem in init 
    DBG("Failed to init modem , retrying .... ");
    // wait for 5 sec 
    delay(5000); 
    // try init again 
    modem.init();
  }
  else 
  {
    // error handling
  }
  // set the connection of network with gsm 
  // check if the connection was failed  
  if(!modem.waitForNetwork(60000L , true))
  {
    DBG("the connection failed to the network"); 
    // wait for like 10 sec 
    delay(10000); 
    return ; 
  }
  else
  {
    // this connection is true 
  }

  // another func for check the network 
  if ( modem.isNetworkConnected())
  {
    // print the connection is sucess 
    DBG("Network Connected") ; 
  }
  
  checkNetworkStatus();
  delay(1000); 
  SIM7600ConnectionGPRS ();
  delay(1000); 
  SIM7600_Check_GPRS_Conn();
  delay(1000); 
  SIM7600_check_signal_quality();
  delay(1000); 
  /*
  if()
  {
    int x = 0;  // Declare and initialize retry counter
    while (Serial.read() != 'ok' && x < 10)
    {
      Serial.write("AT\r\n");
      Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
      delay(1000);
      x++;
      GSM7600.init_flag = 1;
    }
    
    if (GSM7600.init_flag)
    {
      Serial.println("error in init");
      GSM7600.init_flag = 0;
      return;  // Exit the function instead of break
    }
    
    Serial.println(F("setup complete"));
    //If using cellular, establish the GPRS or EPS data connection after your are successfully registered on the network
    modem.gprsConnect(apn)
  }
  */
}

//////////////////////////////////////////////////////////////////////////////////////////////////

/*
void GSM7600_Get_Data_from_Server(const char * URL )
{

  if (modem.isNetworkConnected())
  {
    DBG("network is connected , proceeding with HTTP to get request ..."); 
    
    // create object for http request 
    TinyGsmHttpClient http(modem); 

    // start the get request 
    int response = http.get(URL);

    if(response> 0 )
    {
      DBG("HTTP Get Successful , server resp code : "); 
      DBG(String(response)); 

    }
    else
    {
      // the connection is failed  
      DBG("failed to send http request , Response : ");
      DBG(String(response));

    }
      // the connection is success now we need to read the data 
      String responseBody = http.getResponseBody();
      DBG("server Responce Body : "); 
      DBG(responseBody); 
  }
  else
  {
    // the connection of network failed 
    DBG("network not connected , http request can not be sent ..."); 
  }

}
*/

 ///Receiving Data from LoadData2 API (GET Request)
 
void GSM7600_Get_Data_from_Server(const char* URL)
{
  TinyGsmClient client(modem);
  // we need buffer to hold the json payload
  char check[100] ;
  //format the payload ( ie .. rc =200 response after send serial number )
  sprintf(check ,"{\"Serial\":%s}" , Serialnumber);

  if (modem.isNetworkConnected())
   {
    DBG("Network is connected, proceeding with HTTP GET request.");
    int  test ;
    // Connect to the server
    Serial.write("AT+HTTPPARA=”URL”,http://trapsnos.epcmms.com/api/Traps/LoadData2:80");
   
    if (test = Serial.read()) //80 for http port number 
    {
     // DBG(String(test));
      DBG("Connected to server!");

      // Send the HTTP GET request
      // client.print(String("GET ") + URL + " HTTP/1.1\r\n" +
      //              "Host: trapsnos.epcmms.com\r\n" +
      //              "Connection: close\r\n\r\n");
      // Send the HTTP GET request
      client.print(String("POST ") + URL + " HTTP/1.1\r\n" +
                  "Host: trapsnos.epcmms.com\r\n" +
                  "Content-Type: application/json\r\n" +
                  "Content-Length: " + String(strlen(check)) + "\r\n" +
                  "Connection: close\r\n\r\n" +
                  check);
      int responseCode = 0;
      // Wait for the response
      while (client.connected() || client.available()) 
      {
        if (client.available()) 
        {
          String line = client.readStringUntil('\n');
          DBG(line);  // Print the response line by line

          // check the code of http req = 200 as ok 
          if (line.indexOf("HTTP/1.1 200 OK") !=-1)
          {
            responseCode = 200; 
            DBG("HTTP POST Successful . Response code: 200") ;

          }
        }
      }

      if (responseCode == 200) 
      {
          String responseBody; 
        
        while (client.available())
        {
          responseBody += client.readStringUntil('\n');
        }
        DBG("Received Data:");
        DBG(responseBody);
        // Parse the JSON response
        /*
        DynamicJsonDocument jsonBuffer(1024);  // Adjust buffer size as needed
        DeserializationError error = deserializeJson(jsonBuffer, responseBody);
        
        
        if (error) 
        {
          DBG("Failed to parse JSON.");
        }
        */
      }
      client.stop();  // Close the connection
      DBG("Disconnected from server.");
    } else 
    {
      DBG(String(test));
      DBG("Failed to connect to the server.");
    }
  }
  else 
  {
    DBG("Network not connected, HTTP request cannot be sent.");
  }
}




///for Sending Data to Reading API (POST Request)

void GSM7600_Send_Data_to_Server(const char* URL, const char* payload) {
  TinyGsmClient client(modem);

  if (modem.isNetworkConnected()) {
    DBG("Network is connected, proceeding with HTTP POST request.");

    // Connect to the server
    if (client.connect(URL, 80)) {
      DBG("Connected to server!");

      // Send the HTTP POST request
      client.print(String("POST ") + URL + " HTTP/1.1\r\n" +
                   "Host: trapsnos.epcmms.com\r\n" +
                   "Content-Type: application/json\r\n" +
                   "Content-Length: " + String(strlen(payload)) + "\r\n" +
                   "Connection: close\r\n\r\n" +
                   payload);

      // Wait for the response
      while (client.connected() || client.available()) {
        if (client.available()) {
          String line = client.readStringUntil('\n');
          DBG(line);  // Print the response line by line
        }
      }

      client.stop();  // Close the connection
      DBG("Disconnected from server.");
    } else {
      DBG("Failed to connect to the server.");
    }
  } else {
    DBG("Network not connected, HTTP request cannot be sent.");
  }
}


void SIM7600_Send_data_to_server (const char * URL) 
{
  // create local buffer  to save the data 
  char check[100];
  // Format the JSON payload with Serialnumber
  sprintf(check, "{\"Serial\":%s}", Serialnumber);  
}

// helper function for debug
void printSerialResponse() 
{
  while (SerialAT.available()) 
  {
    String response = SerialAT.readStringUntil('\n');
    Serial.println("Received from SIM7600: " + response);
  }
}

void setup_HTTP()
{
  char define_PDP_str[50] ; 
  //SerialAT.println("AT+CCID");
  // network regesteration : This command is used to control the presentation of an unsolicited result code look page 85 data sheet v3.0.0
  SerialAT.println("AT+CREG?"); 
  Serial.println("Sent to SIM7600:  + AT+CREG?");
  delay(3000);
  printSerialResponse(); 
  // Packet domain attach we need to activate it (attached)
  // The write command is used to attach the MT to, or detach the MT from, the Packet Domain service. 
  // The read command returns the current Packet Domain service state.
  SerialAT.println("AT+CGATT=1") ; 
  Serial.println("Sent to SIM7600: + AT+CGATT=1");
  delay(2000); 
  printSerialResponse();
  // GPRS network registration status
  // Indicates the state of PDP context activate , 
  // and the sec one is (PDP Context Identifier) a numeric parameter which specifies a particular
  // PDP context definition. The parameter is local to the TE-MT interface and is
  // used in other PDP context-related commands. The range of permitted
  // values (minimum value = 1) is returned by the test form of the command.
  SerialAT.println("AT+CGACT=1,1");
  Serial.println("Sent to SIM7600: +AT+CGACT=1,1");
  delay(2000); 
  printSerialResponse();
  // values (minimum value = 1) is returned by the test form of the command.
  // IP Internet Protocol , apn add 
  // this command is used to concate the command and put in PDP_str
  sprintf(define_PDP_str, "AT+CGDCONT=1,\"IP\",\"%s\"",apn) ; 
  SerialAT.println(define_PDP_str) ;
  Serial.println("Sent to SIM7600: "+ (String) define_PDP_str );
  delay(1000);
  printSerialResponse();

}

void timedelay(int timeout)
{
    long int time = millis();
  while ( (time + timeout) > millis())
  {

  }
}
void timedelay_response(int timeout)
{
    long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (SerialAT.available()) 
  {
      // receive_response = SerialAT.readString();
        receive_response = SerialAT.readString(); 
  }
  }
}

/********************************************************************************************************/
void GSM7600_Get_Data_from_Server_update(const char* URL )
{

  // we need to init the communication 
  SerialAT.println("AT+HTTPINIT");
  DBG("AT+HTTPINIT");
  //delay(1500);
  timedelay(1500);
  //debug
  printSerialResponse();
  // then we need to connect with url (site) and its https not http X_x

  Path_Command = "AT+HTTPPARA=\"URL\",\"";
  // we add the url to the path 
  Path_Command += URL; 
  // end the url 
  Path_Command +="\"" ; 
  //snprintf(Path_Command, sizeof(Path_Command), "AT+HTTPPARA=\"URL\",\"%s\"", URL);
  SerialAT.println(Path_Command);
  Serial.println("Sent to SIM7600: " + Path_Command);
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  // then we need to send json command so we init content , 
  // then data size , then the serial send  
  // Content_Command = "AT+HTTPPARA=\"CONTENT\",\"" ; 
  // Content_Command += "application/json" ; 
   Content_Command = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
  SerialAT.println(Content_Command) ;
  Serial.println("Sent to SIM7600: " + Content_Command);
 // delay(1500); 
  timedelay(1500);
  printSerialResponse();
 
  SerialAT.println("AT+HTTPDATA=33,10000"); 
  Serial.println("Sent to SIM7600: AT+HTTPDATA=33,10000");
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
   
  // this function used to concate the data (serial num JSON) i.e json payload
  sprintf((char*)check, "{\"Serial\":\"%s\"}",Serialnumber);
  SerialAT.println((char*)check) ; 
  Serial.println("Sent to SIM7600: " + String((char*)check));
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  
}



int parseHTTPResponse(const String &buffer) 
{
  if (buffer.indexOf("+HTTPACTION: 1,200") != -1)
  {
    return 200;
  }
  return 0;
}

void take_Action_and_parse(String * response)
{
  SerialAT.println("AT+HTTPACTION=1");
  delay(10000);
  receive_response=  SerialAT.readString();
  response_parsed= parseHTTPResponse(receive_response);
 // Serial.println(receive_response);
  if(response_parsed!=200)
  {
      SerialAT.println("AT+HTTPTERM");  // Terminate HTTP service
     return ;
  }
   SerialAT.println("AT+HTTPREAD=0,102");
   Serial.println("AT+HTTPREAD=0,102");
    delay(1500);
    json_response = SerialAT.readString();
    *response = json_response;
    SerialAT.println("AT+HTTPTERM");
    delay(10000);
}

/*
    * the lastt update this function is used to send data to the site 
    * it take 2 arg one is URL and String message ( json after change from json format to string)
    * make init , set url , content , and length of data , then the json (string) 
    * impotant note don't make the length of data larger than the data it well think the command that after string as a string(tkmla ll json sba7 al fol ) X_x
    * you need to have logic analyzer for good responed to see  
*/
void GSM7600_Send_Data_to_Server_update(const char* URL  ,String message_send)
{
  
  // we need to init the communication 
  SerialAT.println("AT+HTTPINIT");
  DBG("AT+HTTPINIT");
  //delay(1500);
  timedelay(1500);
  //debug
  printSerialResponse();
  // then we need to connect with url (site) and its https not http X_x

  Path_Command = "AT+HTTPPARA=\"URL\",\"";
  // we add the url to the path 
  Path_Command += URL; 
  // end the url 
  Path_Command +="\"" ; 
  //snprintf(Path_Command, sizeof(Path_Command), "AT+HTTPPARA=\"URL\",\"%s\"", URL);
  SerialAT.println(Path_Command);
  Serial.println("Sent to SIM7600: " + Path_Command);
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  // then we need to send json command so we init content , 
  // then data size , then the serial send  
  // Content_Command = "AT+HTTPPARA=\"CONTENT\",\"" ; 
  // Content_Command += "application/json" ; 
   Content_Command = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
  SerialAT.println(Content_Command) ;
  Serial.println("Sent to SIM7600: " + Content_Command);
 // delay(1500); 
  timedelay(1500);
  printSerialResponse();
 // we need to set of data as my project ( bio trap we need 500)
  SerialAT.println("AT+HTTPDATA=442,10000"); 
  Serial.println("Sent to SIM7600: AT+HTTPDATA=500,10000");
  //delay(1500);
  timedelay(1500);
  printSerialResponse();

  // when we test we see some error in memory (ram) so we make ,solution with make the data 
  // as a chunk with length not exceed 50       X_x

  int chunkSize =50 ;
  int jsonlength = message_send.length(); // get the length of string after convert
  // this for loop used for get the data from the string and subtracte from the total length
  int remainingChars = 0 , currentChunkSize=0;
  String chunkString; // the actual data as string (50 length )
  
  // this loop is used to chunk the data into 50 50 50 .... and send it 
  for ( int counter=0 ; counter< jsonlength ; counter +=chunkSize)
  {
     remainingChars = jsonlength - counter ;  // as numbers 
     // used to if data not 50 get the remainder of data ex: 80 - 50 = 30 next 30,  take the 30 not 50 bec there is no 50
     currentChunkSize = remainingChars> chunkSize ? chunkSize : remainingChars ; 
     chunkString = message_send.substring(counter,counter+currentChunkSize) ; // used to cut the data from the full  str and but it inside the chunkstring 
     SerialAT.print(chunkString);
  }
  SerialAT.println(); // end of sending

  // this for test 
    // SerialAT.print(jsonChunk1);
    // SerialAT.print(jsonChunk2);
    // SerialAT.print(jsonChunk3);
    // SerialAT.print(jsonChunk4);
    // SerialAT.print(jsonChunk5);
    // SerialAT.print(jsonChunk6);
    // SerialAT.print(jsonChunk7);
    // SerialAT.print(jsonChunk8);
    // SerialAT.print(jsonChunk9);
    // SerialAT.print(jsonChunk10);
    // SerialAT.println(jsonChunk11);
    
  timedelay(1500);
  printSerialResponse();
  

  /*
    int messageLength=0, data_length=0;
    // we need to init the communication 
    SerialAT.println("AT+HTTPINIT");
    DBG("AT+HTTPINIT");
    //delay(1500);
    timedelay(2000);
    //debug
    printSerialResponse();
    // then we need to connect with url (site) and its https not http X_x

    Path_Command = "AT+HTTPPARA=\"URL\",\"";
    // we add the url to the path 
    Path_Command += URL; 
    // end the url 
    Path_Command +="\"" ;
  
    // Path_Command = "AT+HTTPPARA=\"URL\",\"" + String(URL) + "\""; 
    //snprintf(Path_Command, sizeof(Path_Command), "AT+HTTPPARA=\"URL\",\"%s\"", URL);
    SerialAT.println(Path_Command);
    Serial.println("Sent to SIM7600: " + Path_Command);
    //delay(1500);
    timedelay(2000);
    printSerialResponse();
    // then we need to send json command so we init content , 
    // then data size , then the serial send  
    // Content_Command = "AT+HTTPPARA=\"CONTENT\",\"" ; 
    // Content_Command += "application/json" ; 
    Content_Command = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
    SerialAT.println(Content_Command) ;
    Serial.println("Sent to SIM7600: " + Content_Command);
    // delay(1500); 
    timedelay(2000);
    printSerialResponse();

    // messageLength = message_send.length();  // Use length() for String type
    //  sprintf(data_length, "AT+HTTPDATA=%d,10000", messageLength);
    //  SerialAT.println(data_length);
    //  Serial.println("Sent to SIM7600: " + String(data_length));
    //  timedelay(1500);
    // //delay(1500);
  
    printSerialResponse();
    SerialAT.println("AT+HTTPDATA=500,10000");
    timedelay(2500);
    // this function used to concate the data (serial num JSON) i.e json payload
    // sprintf((char*)check, "{\"Serial\":\"%s\"}",Serialnumber);
    // SerialAT.println((char*)check) ; 
    SerialAT.println(message_send);
    Serial.println("Sent to SIM7600: " + message_send);
    //delay(1500);
    timedelay(3000);
    printSerialResponse();
  */
}

//  this take action and get the response of https 
void take_Action_and_parse_for_sending(String * response)
{
  // after take send data to buffer you need to take action 
  SerialAT.println("AT+HTTPACTION=1");
  delay(10000);
  receive_response=  SerialAT.readString();
  response_parsed= parseHTTPResponse(receive_response); // used to get response 

  if(response_parsed!=200)
  {
      SerialAT.println("AT+HTTPTERM");  // Terminate HTTP service
     return ;
  }
    // this for test i need to received the response 200 and save it to response ^_^
    json_response = SerialAT.readString();
    *response = json_response;
    SerialAT.println("AT+HTTPTERM");
    delay(1500);
}




/***************************************************************************************************************************************************************************************/

/*
void checkNetworkStatus()
{

  Serial.println("AT"); 
  // wait for 15 sec 
  delay(1000);
  if(SerialAT.find("ok"))
  {
    Serial.println(F("Setup Complete!"));
    Save_Serial_Data("Sim Setup Complete");
  }
  else
  {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }
  //
   // AT+CREG?: This AT command asks the SIM module to return the current network registration status. The response will tell you whether the module is connected to a network, trying to connect, or not registered.
  //
  Serial.println("AT+CREG?"); 
  delay(1000); 
  while (Serial.available()) 
  {
    char creg =  Serial.read() ; 
    Serial.write(creg);  // Print network status
    
  }
  

} 



void GSM7600_Get_Data_from_Server_update(const char* URL )
{

  // we need to init the communication 
  SerialAT.println("AT+HTTPINIT");
  DBG("AT+HTTPINIT");
  //delay(1500);
  timedelay(1500);
  //debug
  printSerialResponse();
  // then we need to connect with url (site) and its https not http X_x

  Path_Command = "AT+HTTPPARA=\"URL\",\"";
  // we add the url to the path 
  Path_Command += URL; 
  // end the url 
  Path_Command +="\"" ; 
  //snprintf(Path_Command, sizeof(Path_Command), "AT+HTTPPARA=\"URL\",\"%s\"", URL);
  SerialAT.println(Path_Command);
  Serial.println("Sent to SIM7600: " + Path_Command);
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  // then we need to send json command so we init content , 
  // then data size , then the serial send  
  // Content_Command = "AT+HTTPPARA=\"CONTENT\",\"" ; 
  // Content_Command += "application/json" ; 
   Content_Command = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
  SerialAT.println(Content_Command) ;
  Serial.println("Sent to SIM7600: " + Content_Command);
 // delay(1500); 
  timedelay(1500);
  printSerialResponse();
 
  SerialAT.println("AT+HTTPDATA=33,10000"); 
  Serial.println("Sent to SIM7600: AT+HTTPDATA=33,10000");
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
   
  // this function used to concate the data (serial num JSON) i.e json payload
  sprintf((char*)check, "{\"Serial\":%s}",Serialnumber);
  SerialAT.println((char*)check) ; 
  Serial.println("Sent to SIM7600: " + String((char*)check));
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  
}

void setup_HTTP()
{
  char define_PDP_str[50] ; 
  //SerialAT.println("AT+CCID");
  // network regesteration : This command is used to control the presentation of an unsolicited result code look page 85 data sheet v3.0.0
  SerialAT.println("AT+CREG?"); 
  Serial.println("Sent to SIM7600:  + AT+CREG?");
  delay(3000);
  printSerialResponse(); 
  // Packet domain attach we need to activate it (attached)
  // The write command is used to attach the MT to, or detach the MT from, the Packet Domain service. 
  // The read command returns the current Packet Domain service state.
  SerialAT.println("AT+CGATT=1") ; 
  Serial.println("Sent to SIM7600: + AT+CGATT=1");
  delay(2000); 
  printSerialResponse();
  // GPRS network registration status
  // Indicates the state of PDP context activate , 
  // and the sec one is (PDP Context Identifier) a numeric parameter which specifies a particular
  // PDP context definition. The parameter is local to the TE-MT interface and is
  // used in other PDP context-related commands. The range of permitted
  // values (minimum value = 1) is returned by the test form of the command.
  SerialAT.println("AT+CGACT=1,1");
  Serial.println("Sent to SIM7600: +AT+CGACT=1,1");
  delay(2000); 
  printSerialResponse();
  // values (minimum value = 1) is returned by the test form of the command.
  // IP Internet Protocol , apn add 
  // this command is used to concate the command and put in PDP_str
  sprintf(define_PDP_str, "AT+CGDCONT=1,\"IP\",\"%s\"",apn1) ;     // de ana 3deltaha (apn1 ?) // please check
  SerialAT.println(define_PDP_str) ;
  Serial.println("Sent to SIM7600: "+ (String) define_PDP_str );
  delay(1000);
  printSerialResponse();

}


int parseHTTPResponse(const String &buffer) 
{
  if (buffer.indexOf("+HTTPACTION: 1,200") != -1)
  {
    return 200;
  }
  return 0;
}


void processJSONData(String* command , String* hamada_json) 
{
  // Print the received command
  Serial.println("Processing command as JSON:");

  // Look for JSON boundaries
  // indexof and last index of are reference in c++ to find , rfind 
  int jsonStart = command->indexOf('{'); // Find the start of JSON // return the first pos of it (let say 40)
  int jsonEnd = command->lastIndexOf('}'); // Find the end of JSON // return the last pos of it (let say 140)
  String jsonData;
  *hamada_json = "";  // Clear out the previous dat
  if (jsonStart != -1 && jsonEnd != -1 && jsonEnd > jsonStart)  
  {
    // Extract JSON data from the command string
    jsonData = command->substring(jsonStart, jsonEnd + 1);
    Serial.println("Extracted JSON: " + jsonData);
     
    // Debugging output to confirm the extraction
    for (int i = 0; i < jsonData.length(); i++) 
    {
      Serial.print("Character [");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(jsonData[i]);
      // *hamada_json += jsonData[i];
    }
   *hamada_json = jsonData;
  } 
  
  else 
  {
    Serial.println("No valid JSON data found in the command.");
  }
  Serial.print(" the hamada json inside the function : ");
  Serial.println(*hamada_json);
  // Serial.print(" the hamada json inside the function jsonData : ");
  // Serial.println(jsonData);

}





void take_Action_and_parse(String* response)
{
  SerialAT.println("AT+HTTPACTION=1");
  delay(5000);
  receive_response = SerialAT.readString();
  response_parsed = parseHTTPResponse(receive_response);
  
  if (response_parsed != 200)
  {
    Serial.println("Response not 200. Terminating HTTP.");
    SerialAT.println("AT+HTTPTERM");
    return;
  }

  SerialAT.println("AT+HTTPREAD?");
  delay(500);
  String size_response = SerialAT.readString();
  int json_size = 0;
  int len_index = size_response.indexOf("LEN,");
  if (len_index != -1) {
    String size_str = size_response.substring(len_index + 4);
    json_size = size_str.toInt();
  }

  if (json_size <= 0) {
    Serial.println("Invalid response size.");
    SerialAT.println("AT+HTTPTERM");
    return;
  }

  Serial.print("Length of JSON: ");
  Serial.println(json_size);

  SerialAT.print("AT+HTTPREAD=0,");
  SerialAT.println(String(json_size));   

  json_response = "";  // Clear global buffer before appending
  unsigned long start_time = millis();
  while (millis() - start_time < 3000)
  {
      if (SerialAT.available())
      {
          // Read until newline or any other delimiter that ensures the full JSON response is captured
          String line = SerialAT.readStringUntil('\n');  // or you can try '\r' or even use a timeout if the newline is not used
          
          // If the line contains valid JSON data, append it
          if (line.startsWith("{"))  // Only append if the line starts with a '{'
          {
              json_response += line;
          }
      }
  }

  // Output the complete JSON response
  Serial.println("Complete JSON response: " + json_response);

  // Now process the JSON response
  processJSONData(&json_response, (response));
  Serial.println("Extracted JSON outside function: " + *response);

  SerialAT.println("AT+HTTPTERM");

}


void take_Action_and_parse_update(char *response, int size) {
    SerialAT.println("AT+HTTPACTION=1");
    delay(5000);

    // Check if the HTTP action response is 200
    receive_response = SerialAT.readString();
    response_parsed = parseHTTPResponse(receive_response);
    if (response_parsed != 200) {
        Serial.println("The response is not 200");
        SerialAT.println("AT+HTTPTERM");
        return;
    }

    // Request the total size of the HTTP response data
    SerialAT.println("AT+HTTPREAD?");
    delay(500);
    String size_response = SerialAT.readString();
    int json_size = 0;
    int len_index = size_response.indexOf("LEN,");
    if (len_index != -1) {
        String size_str = size_response.substring(len_index + 4);
        json_size = size_str.toInt();
    }

    if (json_size <= 0) {
        Serial.println("Invalid response size.");
        SerialAT.println("AT+HTTPTERM");
        return;
    }
    Serial.print("The length equal = ");
    Serial.println(json_size);

    // Request to read the entire JSON response in one go
    SerialAT.print("AT+HTTPREAD=0,");
    SerialAT.print(json_size);
    delay(500);

    size_t index = 0;
    unsigned long startMillis = millis();
    const unsigned long timeout = 9000;  // Set timeout for data reading

    bool json_started = false;
    // Clear the buffer to start fresh
    memset(response, 0, size);

     // Read loop with timeout
    while (millis() - startMillis < timeout && index < size - 1) {
        if (SerialAT.available()) {
            char c = SerialAT.read();

            // Start capturing JSON when the opening brace '{' is found
            if (c == '{') json_started = true;

            // Capture JSON content only
            if (json_started) {
                response[index++] = c;

                // Stop capturing after finding the closing brace '}'
                if (c == '}') {
                    break;
                }
            }
        }
    }

    // bool json_complete = false;
    // String hamada_full_data ;

    // // Use do-while loop to read until the JSON data is fully received
    // do {
    //     while (SerialAT.available()) {
    //       hamada_full_data= SerialAT.readStringUntil("}");
    //       Serial.println(hamada_full_data);
    //         char c = SerialAT.read();

    //         // Start capturing JSON content after finding the opening brace '{'
    //         if (c == '{') json_started = true;

    //         // Only capture content when JSON data has started
    //         if (json_started) {
    //             response[index++] = c;
    //             Serial.println("iam in the loop : ");
    //             Serial.println(c);
                
    //             // Stop reading after finding the closing brace '}'
    //             if (c == '}') {
    //                 json_complete = true;
    //                 break;
    //             }
                
    //             // Prevent overflow in the buffer
    //             if (index >= size - 1) {
    //                 Serial.println("Buffer size exceeded.");
    //                 json_complete = true;
    //                 break;
    //             }
    //         }
    //     }
        
    //     // Timeout check to avoid infinite loop
    //     if (millis() - startMillis >= timeout) {
    //         Serial.println("Read timeout reached.");
    //         break;
    //     }
        
    // } while (!json_complete);

    // Null-terminate the response to make it a valid C-string
    response[index] = '\0';

    // Debug: print the filtered JSON response
    Serial.print("Filtered JSON response: ");
    Serial.println(response);

    SerialAT.println("AT+HTTPTERM");
    delay(500);
}

*/

/*
    * the lastt update this function is used to send data to the site 
    * it take 2 arg one is URL and String message ( json after change from json format to string)
    * make init , set url , content , and length of data , then the json (string) 
    * impotant note don't make the length of data larger than the data it well think the command that after string as a string(tkmla ll json sba7 al fol ) X_x
    * you need to have logic analyzer for good responed to see  
*/
/*
void GSM7600_Send_Data_to_Server_update(const char* URL  ,String message_send)
{
                // Serial.print("inside the func send data: ");
                // Serial.println(message_send);
  
  // we need to init the communication 
  SerialAT.println("AT+HTTPINIT");
  DBG("AT+HTTPINIT");
  //delay(1500);
  timedelay(1500);
  //debug
  printSerialResponse();
  // then we need to connect with url (site) and its https not http X_x

  Path_Command = "AT+HTTPPARA=\"URL\",\"";
  // we add the url to the path 
  Path_Command += URL; 
  // end the url 
  Path_Command +="\"" ; 
  //snprintf(Path_Command, sizeof(Path_Command), "AT+HTTPPARA=\"URL\",\"%s\"", URL);
  SerialAT.println(Path_Command);
  Serial.println("Sent to SIM7600: " + Path_Command);
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  // then we need to send json command so we init content , 
  // then data size , then the serial send  
  // Content_Command = "AT+HTTPPARA=\"CONTENT\",\"" ; 
  // Content_Command += "application/json" ; 
   Content_Command = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
  SerialAT.println(Content_Command) ;
  Serial.println("Sent to SIM7600: " + Content_Command);
 
  timedelay(1500);
  printSerialResponse();
  int jsonlength = message_send.length(); // get the length of string after convert
 // we need to set of data as my project ( bio trap we need 500)
  Serial.println("the length of data i received ");
  Serial.println(jsonlength); // Outputs the length of message_send

  char Data_size[30];
  sprintf(Data_size ,"AT+HTTPDATA=%d,10000" ,jsonlength );
            // SerialAT.println("AT+HTTPDATA=442,10000"); 
            // Serial.println("Sent to SIM7600: AT+HTTPDATA=500,10000");
  SerialAT.println(Data_size); 
 // Serial.println(Data_size); 

  timedelay(1500);
  printSerialResponse();
                      //SerialAT.print(message_send);
  Serial.print("free memory inside the function ") ;
  Serial.println(freeMemory());
  // when we test we see some error in memory (ram) so we make ,solution with make the data 
  // as a chunk with length not exceed 50       X_x

  int chunkSize =50 ;
  // this for loop used for get the data from the string and subtracte from the total length
  int remainingChars = 0 , currentChunkSize=0;
  String chunkString; // the actual data as string (50 length )
  
  // this loop is used to chunk the data into 50 50 50 .... and send it 
  for ( int counter=0 ; counter< jsonlength ; counter +=chunkSize)
  { 

    // debug part X_x
    Serial.print("iam the counter inside the chunk function : "); 
    Serial.println(counter);


     remainingChars = jsonlength - counter ;  // as numbers 
     // used to if data not 50 get the remainder of data ex: 80 - 50 = 30 next 30,  take the 30 not 50 bec there is no 50
     currentChunkSize = remainingChars> chunkSize ? chunkSize : remainingChars ; 
     chunkString = message_send.substring(counter,counter+currentChunkSize) ; // used to cut the data from the full  str and but it inside the chunkstring 
     SerialAT.print(chunkString);
     
     //debug 
     Serial.println(chunkString);
  }
  SerialAT.println(); // end of sending

  // this for test 
    // SerialAT.print(jsonChunk1);
    // SerialAT.print(jsonChunk2);
    // SerialAT.print(jsonChunk3);
    // SerialAT.print(jsonChunk4);
    // SerialAT.print(jsonChunk5);
    // SerialAT.print(jsonChunk6);
    // SerialAT.print(jsonChunk7);
    // SerialAT.print(jsonChunk8);
    // SerialAT.print(jsonChunk9);
    // SerialAT.print(jsonChunk10);
    // SerialAT.println(jsonChunk11);
    
  timedelay(1500);
  printSerialResponse();
  

}

//  this take action and get the response of https 
void take_Action_and_parse_for_sending(String * response)
{
  // after take send data to buffer you need to take action 
  SerialAT.println("AT+HTTPACTION=1");
  delay(10000);
  receive_response=  SerialAT.readString();
  response_parsed= parseHTTPResponse(receive_response); // used to get response 

  if(response_parsed!=200)
  {
      Serial.println("error in sending data to server");
      SerialAT.println("AT+HTTPTERM");  // Terminate HTTP service
     return ;
  }
  else
  {
    Serial.println("data post successfully to the server the resp = 200 ");
  }
    // this for test i need to received the response 200 and save it to response ^_^
    json_response = SerialAT.readString();
    *response = json_response;
    SerialAT.println("AT+HTTPTERM");
    delay(1000);
}

void SiM7600_SDCard_check_User_request(String SD_Request)
{
  if(SD_Request != " ")
  sendsd(SD_Request);
}
*/
/*********************************************************************** GPS Function ************************************************************************************************/







// this function is used to enable  the GPS of Sim7600 
void GPS_Enable() 
{
  
    // send enable and wait for 1 sec 
    SerialAT.println("AT"); 
    timedelay(500);
    // you need to off first
    SerialAT.println("AT+CGPS=0");
    timedelay(500);
      // set the NMEA rate output which is 1 hz or 10hz 
    SerialAT.println("AT+CGPSNMEARATE=1"); // 10hz
    timedelay(500); 
    SerialAT.println("AT+CGPS=1,1");
    timedelay(500);
} 

// this function used to read the GPS output and save it into the GPS_op string you can use it and parsed out side the function 
void GPS_GETDATA(String * GPS_OP)
{

  SerialAT.println("AT+CGPSINFO"); // get the data 
   timedelay(500); 
  * GPS_OP = SerialAT.readString();

}

// disable function u need to disable aftr reading the GPS 
void GPS_Disable()
{
      SerialAT.println("AT+CGPS=0");
      timedelay(500); 
}




/********************************************************************************************************/
void sendATcommand(const char* ATcommand, unsigned int timeout)
 {

    uint8_t x=0,  answer=0;
    char response[150];
    unsigned long previous;

    memset(response, '\0', 150);    // Initialize the string
    
    delay(100);
    
    while( Serial.available() > 0) Serial.read();    // Clean the input buffer
    
    Serial.println(ATcommand);    // Send the AT command 
    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(Serial.available() != 0){    
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = Serial.read();      
//            Serial.print(response[x]);
            x++;
        }
         // Waits for the asnwer with time out
    }while(((millis() - previous) < timeout));
    
   Serial.print(response);   
    // return  response ;
}


void GSM7600_send_Data_to_Server(const char* URL ,JsonObject& doc)
{
  // we need to init the communication 
  SerialAT.println("AT+HTTPINIT");
  DBG("AT+HTTPINIT");
  //delay(1500);
  timedelay(1500);
  //debug
  printSerialResponse();
  // then we need to connect with url (site) and its https not http X_x

  Path_Command = "AT+HTTPPARA=\"URL\",\"";
  // we add the url to the path 
  Path_Command += URL; 
  // end the url 
  Path_Command +="\"" ; 
  SerialAT.println(Path_Command);
  Serial.println("Sent to SIM7600: " + Path_Command);
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  // then we need to send json command so we init content , 
  // then data size , then the serial send  
  // Content_Command = "AT+HTTPPARA=\"CONTENT\",\"" ; 
  // Content_Command += "application/json" ; 
   Content_Command = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
  SerialAT.println(Content_Command) ;
  Serial.println("Sent to SIM7600: " + Content_Command);
 // delay(1500); 
  timedelay(1500);
  printSerialResponse();
 
  SerialAT.println("AT+HTTPDATA=300,10000"); 
  Serial.println("Sent to SIM7600: AT+HTTPDATA=300,10000");
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  
}


void GSM7600_send_Data_to_Server(const char* URL ,JsonObject& doc ,const char * data_send)
{
  // we need to init the communication 
  SerialAT.println("AT+HTTPINIT");
  DBG("AT+HTTPINIT");
  //delay(1500);
  timedelay(1500);
  //debug
  printSerialResponse();
  // then we need to connect with url (site) and its https not http X_x

  Path_Command = "AT+HTTPPARA=\"URL\",\"";
  // we add the url to the path 
  Path_Command += URL; 
  // end the url 
  Path_Command +="\"" ; 
  SerialAT.println(Path_Command);
  Serial.println("Sent to SIM7600: " + Path_Command);
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  // then we need to send json command so we init content , 
  // then data size , then the serial send  
  // Content_Command = "AT+HTTPPARA=\"CONTENT\",\"" ; 
  // Content_Command += "application/json" ; 
   Content_Command = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
  SerialAT.println(Content_Command) ;
  Serial.println("Sent to SIM7600: " + Content_Command);
 // delay(1500); 
  timedelay(1500);
  printSerialResponse();
 
  SerialAT.println("AT+HTTPDATA=300,10000"); 
  Serial.println("Sent to SIM7600: AT+HTTPDATA=300,10000");
  //delay(1500);
  timedelay(1500);
  printSerialResponse();

  // this function used to concate the data (serial num JSON) i.e json payload
  sprintf((char*)check2, "%s",data_send );
  SerialAT.println((char*)check2) ; 
  Serial.println("Sent to SIM7600: " + String((char*)check2));
  //delay(1500);
  timedelay(1500);
  printSerialResponse();
  // this to take action 
  SerialAT.println("AT+HTTPACTION=1");

  //delay(10000);
  timedelay(10000);
  Serial.println("Sent to SIM7600: AT+HTTPACTION=1");
  timedelay_response(1000) ; 
  timedelay(1000); 
  // this for parsing
      Serial.println("HTTP Response e: ");
      Serial.println("HTTP Response Code: " + String(receive_response));
      timedelay(1000);
      response_parsed = receive_response.substring(receive_response.indexOf(',') + 1 ,receive_response.lastIndexOf(',')).toInt();
      Serial.println("HTTP Response Code  parsed : " + String(response_parsed));
 
  if (response_parsed == 200)
  {
    Serial.println("success to send data to server");
  }
  else
  {
    Serial.println("Failed to send data to server");
  }

}

// #include <TinyGsmClient.h>

// #include <SoftwareSerial.h>


// #define MODEM_SERIAL Serial
// //TinyGsm modem(MODEM_SERIAL);
// TinyGsmClient gsmClient(modem);

// const char* serverUrl = "http://trapsnos.epcmms.com/api/Reading"; // Your server URL

// void sendPostRequest_1111(String jsonMessage) {
//     // Start the connection to the server using HTTPS
//     if (gsmClient.connect(serverUrl, 443)) { // Port 443 for HTTPS
//         // Send HTTP POST request
//         gsmClient.println("POST /api/Reading HTTP/1.1");
//         gsmClient.println("Host: trapsnos.epcmms.com");
//         gsmClient.println("Content-Type: application/json");
//         gsmClient.print("Content-Length: ");
//         gsmClient.println(jsonMessage.length());
//         gsmClient.println("Connection: close");
//         gsmClient.println(); // End of headers
        
//         // Send the JSON payload
//         gsmClient.println(jsonMessage);

//         // Wait for response
//         while (gsmClient.connected() || gsmClient.available()) {
//             if (gsmClient.available()) {
//                 String responseLine = gsmClient.readStringUntil('\n');
//                 Serial.println(responseLine); // Print server response
//             }
//         }
        
//         // Disconnect
//         gsmClient.stop();
//     } else {
//         Serial.println("Connection to server failed.");
//     }
// }


