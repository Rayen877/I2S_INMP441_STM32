
#ifndef I2S_H
#define I2S_H

#include "stm32f4xx.h"

#define AUDIO_BUFFER_SIZE 256

extern int32_t i2s_rx_buffer[AUDIO_BUFFER_SIZE];
extern int16_t audio_tx_buffer[AUDIO_BUFFER_SIZE/2];

extern volatile uint8_t half_complete;
extern volatile uint8_t full_complete;

void i2s2_dma_init(void);

#endif
