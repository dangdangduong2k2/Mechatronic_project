#ifndef __READ_ENCODER_DATA_H
#define __READ_ENCODER_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

  
#include "stm32f4xx_hal.h"


typedef struct 
{
    int encoder_data[5];
    int encoder_overflow[5];  
    int enable;
} Encoder_Data;

extern Encoder_Data encoder_data;

void Encoder_Init(void);  
void Encoder_read(uint8_t i);

#ifdef __cplusplus
}
#endif

#endif
