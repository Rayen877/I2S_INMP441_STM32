#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
#include <stdint.h>
#define APB1_CLK 16000000

uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t bauderate);
void set_uart_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t baudrate);
void uart_write(USART_TypeDef *USARTx,uint8_t ch);
void usart_tx_rx_init(void);
char uart_read(USART_TypeDef *USARTx);
void uart_send_string(USART_TypeDef *USARTx, const char *str);
void uart_send_buffer(USART_TypeDef *USARTx,uint8_t* buf,uint16_t size);
#endif
