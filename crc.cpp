#include <FastCRC.h>

FastCRC8 CRC8;

bool parseCRC(const uint8_t msg[], int size,uint8_t motors[4], uint8_t *dir){
  if(CRC8.smbus(msg, size)==0){
    *dir = msg[0];
    for(int i = 1; i<size-1; i++){
      motors[i-1] = msg[i];
    }

    return true;
  }
  
  return false;
}