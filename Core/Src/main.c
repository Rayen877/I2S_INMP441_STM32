#include "UART.h"
#include "I2S.h"

int main(void)
{
	usart_tx_rx_init();
    i2s2_dma_init();

    while (1)
    {
    	if (half_complete)
    	{
    	    half_complete = 0;

    	    for(int i = 0; i < AUDIO_BUFFER_SIZE/2; i++){
    	        audio_tx_buffer[i] = (int16_t)(i2s_rx_buffer[i] >> 8);}

    	        uart_send_buffer(USART2,(uint8_t*)audio_tx_buffer, (AUDIO_BUFFER_SIZE/2) * sizeof(int16_t));
    	}

    	if (full_complete)
    	{
    	    full_complete = 0;

    	    for(int i = AUDIO_BUFFER_SIZE/2; i < AUDIO_BUFFER_SIZE; i++){
    	        audio_tx_buffer[i - AUDIO_BUFFER_SIZE/2] =(int16_t)(i2s_rx_buffer[i] >> 8);}

    	    uart_send_buffer(USART2,(uint8_t*)audio_tx_buffer, (AUDIO_BUFFER_SIZE/2) * sizeof(int16_t));
    	}
    }
}

