#include "../include/quectel_EC200U.hpp"
#include "HardwareSerial.h"


extern HardwareSerial SerialAT;  // Use UART2 for the modem 16 ,17 


// this function is used to print the response at serial  
void Quectel_PrintSerialResponse() 
{
  while (SerialAT.available()) 
  {
    String response = SerialAT.readStringUntil('\n');
    Serial.println("Received from Quectel EC200u : " + response);
  }
}

void Quectel_PrintSerialResponse(String * resp) 
{
  while (SerialAT.available()) 
  {
    String response = SerialAT.readStringUntil('\n');
    Serial.println("Received from Quectel EC200u : " + response);
    *resp= response;
  }
}

int Quectel_Parse_HTTP_Response(const String buffer)
{
 
    if(buffer.indexOf("QHTTPPOST: 0,200,") != -1)
    {
      return connected_http ; 
    }
    else if (buffer.indexOf("QHTTPPOST: 0,400,"))
    {
      return bad_resp;
    }
     else if (buffer.indexOf("QHTTPPOST: 0,404,"))
    {
      return not_found;
    }
     else if (buffer.indexOf("QHTTPPOST: 0,500,"))
    {
      return server_error;
    }
    return 0; // error not config 

}

String Get_Error_message(int Error_code)
{
  switch (Error_code )
  {
    case connected_http : return "successfully connected " ; break ; 
    case bad_resp       : return "error 400 in connection check the message it can be error in frame "; break;
    case not_found      : return "http(s) error 404 not found  " ; break;
    case server_error   : return "server response error 500 " ; break;
    default             : return "error not handling yet X_x" ; 
  }
return "";
}

void Quectel_http_config()
{
    //check the connection 
    SerialAT.println(quectel_cmds.AT_test_connection); 
    Serial.println("Sent to SIM7600: AT ");
    delay(1000);
    Quectel_PrintSerialResponse(); 

    SerialAT.println(quectel_cmds.AT_test_connection); 
    Serial.println("Sent to SIM7600: AT ");
    delay(1000);
    Quectel_PrintSerialResponse();


    // config the pdp 
    SerialAT.println (quectel_cmds.Configure_PDP);
    delay(1000);
    Quectel_PrintSerialResponse();
    // get the apn and config it 
    SerialAT.println(quectel_cmds.PDP_APN_config) ; 
    delay(1000);
    Quectel_PrintSerialResponse();

}

void Quectel_http_post_received_data_from_server(String URL  ,String message_send)
{
    String resp ; // this is used for received resp 
    int timeout = 80  ; // time out for module 
    int URL_length= URL.length();  
    int message_length = message_send.length();
    String datatimeout = ",80,80";
    int response_code;
   
    SerialAT.print(quectel_cmds.Set_URL + String(URL_length) + String (timeout) ); 
    
    Quectel_PrintSerialResponse(&resp);
    if (resp == "CONNECT") 
    {
        // this mean you can add(send) the URL 
        SerialAT.println(URL) ; 
        Quectel_PrintSerialResponse();
       
    }
    else
    {
        SerialAT.println("failed to connect to URL ") ; 
    }
      // reset the data inside the resp 
     resp="";
    // send as the app/ json format 
    SerialAT.println(quectel_cmds.content_type_send) ; 
    delay(1000);

    SerialAT.println(quectel_cmds.set_http_post + String(message_length) + datatimeout) ; 
    delay(4000); 
    Quectel_PrintSerialResponse(&resp);
    if (resp == "CONNECT") 
    {
        // this mean you can add(send) the data (json serial , message , any thing , .....)
        SerialAT.println(message_send) ; 
        Quectel_PrintSerialResponse();  // should receive  OK 
        Quectel_PrintSerialResponse(&resp);  // should receive the resp EX (200 400 404 500 .....)

       response_code = Quectel_Parse_HTTP_Response(resp); // parse  the resp and return the parsed code
       resp = Get_Error_message(response_code); // get the code message 
       Serial.println(resp); // print the message 
       
       if (response_code == 200)
       {
          // get data length and get data 
          SerialAT.println(quectel_cmds.read_length_json);
          Quectel_PrintSerialResponse(&resp);
          SerialAT.println(quectel_cmds.read_data_json + String(resp)); 
          // this is for receiving message 
          Quectel_PrintSerialResponse(&resp);
       }
       
    }
    else
    {
       SerialAT.println("failed to send the data ") ; 
    }


}
