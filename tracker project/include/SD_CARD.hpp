#ifndef SDCARD_HPP
#define DECARD_HPP


bool SD_card_init();
bool SD_create_File(const char * fileName);
bool SD_Write_append(const char * filename , const char * message);
void SD_readFile(const char * filename);
void SD_Delete_file();

#endif