#include <SPI.h>
#include <SD.h>
#define CS_PIN 5        // Define Chip Select pin
bool SD_card_init()
{
  
  Serial.println("init the SD_card.....");
  if(!SD.begin(CS_PIN))
  {
      Serial.println("failed to init the SD X_x"); 
  }
  return false ;
}

bool SD_create_File(const char * fileName)
{
  // check if the file is exist 
  if(SD.exists(fileName))
  {
    // if exist said it exist and out of the function 
    Serial.print("file name is already exist "); 
    Serial.println(fileName);
    return true;
  }

  // if not we create the file 
  File file = SD.open(fileName, FILE_WRITE);

  // if file created 
  if (file)
  {
    // close the file and print in serial it is found 
    file.close(); 
    Serial.println("file create successfully");
    Serial.print("file_name = ");
    Serial.println(fileName);
    return true;
  }
  // else failed to creation 
  else
  {
    Serial.print("failed to create file X_x = "); 
    Serial.println(fileName); 
    return false;
  }
}

bool SD_Write_append(const char * filename , const char * message)
{
  if (SD.exists(filename))
  {
    Serial.println("file found and we append the data") ;
    File file = SD.open(filename,FILE_APPEND); 
      if (file)
      {
        file.println(message);
        file.close(); 
        Serial.println("we successfully add the data ");
        return true ;
      } 
      else 
      {
      Serial.println("Failed to open file for append new record X_x .");
      return false;
      }
    
  }
  else 
  {
    Serial.println("file not created or exist X_x "); 
    return false ; 
  }

}

void SD_readFile(const char * filename)
{
  if (SD.exists(filename))
  {
    Serial.print("Reading file: ");
    Serial.println(filename);
    File file = SD.open(filename, FILE_READ);
    while (file.available()) 
    {
        Serial.write(file.read());
    }
    file.close(); 
  
  }
  else
  {
    Serial.println("the file not found or error in reading X_x") ;
  }
}

void SD_Delete_file(const char * filename)
{
    // check if the file is exist 
    if(SD.exists(filename))
    {
        if(SD.remove(filename))
        {
            Serial.println("the file is successfully removed ");
        }
        else
        {
            Serial.println("the file is has can't remove X_x ");
        }
    }
    else
    {
        Serial.println( "file not exist (6__6) ");
    }
}