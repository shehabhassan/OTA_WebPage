#ifndef QUECTEL_EC200U_HPP
#define QUECTEL_EC200U_HPP

#include <Arduino.h>
// this struct used for HTTP Post request 

typedef struct 
{

const char * AT_test_connection ; 
const char * Configure_PDP;         // the step no.1 for config ,Configure the PDP context ID as 1.
const char * content_type_send ;        // we use 4 application json //Query the state of PDP context.
const char * Query_state_PDP;       // the step no.2 for config ,  Only returning OK means that there is no activated PDP context currently
const char * PDP_APN_config;    // Configure PDP context 1. APN, step 3 
const char * show_ip_sim ;         //Query the state of context.
const char * Set_URL ;      
const char * set_http_post ;    // use to post message after connect message received from RX then add the data you will received ok then received the resp like (200 400 404 500 .....).
const char * read_data_json ;   // use to read the data after resp 200 
const char * read_length_json;      // know the length of received json 
const char * read_file ; 
const char * read_lenght_file ; 
}ST_HTTP ; 

// this enum for web code response (+ error )
typedef enum 
{
    connected_http = 200,
    bad_resp = 400, 
    not_found = 404,
    server_error= 500,
    
}EN_HTTP_code;

// this is variable from struct  it is used for  send command va serialAT
const ST_HTTP quectel_cmds = 
{
    .AT_test_connection = "AT" ,                           // test the  connection ( response of sim )
    .Configure_PDP="AT+QHTTPCFG=\"contextid\",1",         // config thee PDP  
    .content_type_send= "AT+QHTTPCFG=\"contenttype\",4",  // we use application json so we select 4 u can change by looking at datasheet ^_^
    .Query_state_PDP = "AT+QIACT?",                         
    .PDP_APN_config="AT+QICSGP=1,1,\"internet\","","",1", // we don't have user name or password so make them empty 
    .show_ip_sim= "AT+QIACT?" ,
    .Set_URL = "AT+QHTTPURL=" ,         //u need to add length and time out as string        //"AT+QHTTPURL=59,80",         // u need to add the actual length 59 remove , 80 time out 
    .set_http_post= "AT+QHTTPPOST=" ,        // u need to add the max data (data length) and 2 time out for uart and resp    EX : AT+QHTTPPOST= 20 , 80 ,80 --> 20 legth data , 80 uart time out , 80 time out for resp  
    .read_data_json = "AT+QHTTPREAD=" ,      // u need to add length 
    .read_length_json= "AT+QHTTPREAD=?" ,    // use to receive the length data 
    .read_file = "AT+QHTTPREADFILE=" , 
    .read_lenght_file ="AT+QHTTPREADFILE=?",  // use to receive the length data inside the file 
};


/*******************************  function prototype *********************/

void Quectel_PrintSerialResponse() ; 

void Quectel_PrintSerialResponse(String * resp) ;

int Quectel_Parse_HTTP_Response(const String buffer);

String Get_Error_message(int Error_code);

void Quectel_http_config();

/******************************* GPS Module ****************************** */

class Quectel_GPS
{
private:
    String m_Configure_GNSS;       // AT_QGPSCFG Configure_GNSS parameters
    String m_Turn_on;              // Turn on GNSS
    String m_Acquire_Pos_Info;     // AT_QGPSLOC Acquire_Positioning_Information    
    String m_Turn_off;             // Turn off the GPS

public:
    // Constructor
    Quectel_GPS() {
        m_Configure_GNSS = "AT_QGPSCFG=1";
        m_Turn_on = "AT_QGPS=1";
        m_Acquire_Pos_Info = "AT_QGPSLOC=0";
        m_Turn_off = "AT_QGPSEND";
    }

    // Destructor
    ~Quectel_GPS() {}

    // Getter methods
    String getConfigure_GNSS() const { return m_Configure_GNSS; }
    String getTurn_on() const { return m_Turn_on; }
    String getAcquire_Pos_Info() const { return m_Acquire_Pos_Info; }
    String getTurn_off() const { return m_Turn_off; }
    
    // Setter methods to modify commands
    void setConfigure_GNSS(const String &cmd) { m_Configure_GNSS = cmd; }
    void setTurn_on(const String &cmd) { m_Turn_on = cmd; }
    void setAcquire_Pos_Info(const String &cmd) { m_Acquire_Pos_Info = cmd; }
    void setTurn_off(const String &cmd) { m_Turn_off = cmd; }
};



// class Quectel_GPS
// {
// private:
//     /* data */
//     String m_Configure_GNSS;       //AT_QGPSCFG Configure_GNSS parameters
//     String Turn_on;              // Turn_on GNSS
//     String Acquire_Pos_Info;     // AT_QGPSLOC Acquire_Positioning_Information    
//     String Turn_off;           // Turn_off the GPS
// public:
//    Quectel_GPS(){
//                 m_Configure_GNSS=      "AT_QGPSCFG=1",
//                 Turn_on       =      "AT_QGPS=1"     ,
//                 Acquire_Pos_Info =   "AT_QGPSLOC=0",
//                 Turn_off =           "AT_QGPSEND";
//                 };
//   ~Quectel_GPS();
//    // Getter methods
//     String Configure_GNSS() const { return m_Configure_GNSS; }
//     String Turn_on() const { return Turn_on; }
//     String Acquire_Pos_Info() const { return Acquire_Pos_Info; }
//     String Turn_off() const { return Turn_off; }
    
//     // Setter methods to modify commands
//     void Configure_GNSS(const String &cmd) {m_Configure_GNSS = cmd; }
//     void Turn_on(const String &cmd) { Turn_on = cmd; }
//     void Acquire_Pos_Info(const String &cmd) { Acquire_Pos_Info = cmd; }
//     void Turn_off(const String &cmd) { Turn_off = cmd; }
// };

  //String AT_QGPSGNMEA;         //   
  //String AT_QGPSCFG;           //1: enable acquisition of specified NMEA 0 :disable acquisition


/*

#include <iostream>
#include <string>

class QuectelHTTP {
public:
    // Constructor to initialize all AT commands
    QuectelHTTP() {
        AT_test_connection = "AT";
        Configure_PDP = "AT+QHTTPCFG=\"contextid\",1";
        content_type_send = "AT+QHTTPCFG=\"contenttype\",4"; // Application JSON
        Query_state_PDP = "AT+QIACT?";
        PDP_APN_config = "AT+QICSGP=1,1,\"internet\",\"\",\"\",1"; // No username/password
        show_ip_sim = "AT+QIACT?";
        Set_URL = "AT+QHTTPURL="; // URL + length + timeout
        set_http_post = "AT+QHTTPPOST="; // Data length + UART timeout + Response timeout
        read_data_json = "AT+QHTTPREAD="; // Expected response length
        read_length_json = "AT+QHTTPREAD=?";
        read_file = "AT+QHTTPREADFILE=";
        read_lenght_file = "AT+QHTTPREADFILE=?";
    }

    // Getter methods
    std::string getATTestConnection() const { return AT_test_connection; }
    std::string getConfigurePDP() const { return Configure_PDP; }
    std::string getContentTypeSend() const { return content_type_send; }
    std::string getQueryStatePDP() const { return Query_state_PDP; }
    std::string getPDPAPNConfig() const { return PDP_APN_config; }
    std::string getShowIPSIM() const { return show_ip_sim; }
    std::string getSetURL() const { return Set_URL; }
    std::string getSetHttpPost() const { return set_http_post; }
    std::string getReadDataJson() const { return read_data_json; }
    std::string getReadLengthJson() const { return read_length_json; }
    std::string getReadFile() const { return read_file; }
    std::string getReadLengthFile() const { return read_lenght_file; }

    // Setter methods to modify commands if needed
    void setATTestConnection(const std::string &cmd) { AT_test_connection = cmd; }
    void setConfigurePDP(const std::string &cmd) { Configure_PDP = cmd; }
    void setContentTypeSend(const std::string &cmd) { content_type_send = cmd; }
    void setQueryStatePDP(const std::string &cmd) { Query_state_PDP = cmd; }
    void setPDPAPNConfig(const std::string &cmd) { PDP_APN_config = cmd; }
    void setShowIPSIM(const std::string &cmd) { show_ip_sim = cmd; }
    void setSetURL(const std::string &cmd) { Set_URL = cmd; }
    void setSetHttpPost(const std::string &cmd) { set_http_post = cmd; }
    void setReadDataJson(const std::string &cmd) { read_data_json = cmd; }
    void setReadLengthJson(const std::string &cmd) { read_length_json = cmd; }
    void setReadFile(const std::string &cmd) { read_file = cmd; }
    void setReadLengthFile(const std::string &cmd) { read_lenght_file = cmd; }

    // Method to output all AT commands for debugging purposes
    void printCommands() const {
        std::cout << "AT Test Connection: " << AT_test_connection << std::endl;
        std::cout << "Configure PDP: " << Configure_PDP << std::endl;
        std::cout << "Content Type Send: " << content_type_send << std::endl;
        std::cout << "Query State PDP: " << Query_state_PDP << std::endl;
        std::cout << "PDP APN Config: " << PDP_APN_config << std::endl;
        std::cout << "Show IP SIM: " << show_ip_sim << std::endl;
        std::cout << "Set URL: " << Set_URL << std::endl;
        std::cout << "Set HTTP Post: " << set_http_post << std::endl;
        std::cout << "Read Data JSON: " << read_data_json << std::endl;
        std::cout << "Read Length JSON: " << read_length_json << std::endl;
        std::cout << "Read File: " << read_file << std::endl;
        std::cout << "Read Length File: " << read_lenght_file << std::endl;
    }

private:
    // Private member variables for the AT commands
    std::string AT_test_connection;
    std::string Configure_PDP;
    std::string content_type_send;
    std::string Query_state_PDP;
    std::string PDP_APN_config;
    std::string show_ip_sim;
    std::string Set_URL;
    std::string set_http_post;
    std::string read_data_json;
    std::string read_length_json;
    std::string read_file;
    std::string read_lenght_file;
};

// Example Usage
int main() {
    // Instantiate the QuectelHTTP class
    QuectelHTTP quectel_cmds;

    // Access and print the commands for debugging
    quectel_cmds.printCommands();

    // Modify a command (for example, changing the PDP APN config)
    quectel_cmds.setPDPAPNConfig("AT+QICSGP=1,1,\"new_apn\",\"\",\"\",1");

    // Print the updated commands
    std::cout << "\nUpdated Commands:" << std::endl;
    quectel_cmds.printCommands();

    return 0;
}
 */
/*********************************************************************** */


#endif 