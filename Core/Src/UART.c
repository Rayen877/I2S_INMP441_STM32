#include <UART.h>


void usart_tx_rx_init(void){
	RCC->AHB1ENR |=(1<<0);
	//PA2
	GPIOA->MODER |= (0<<4);
	GPIOA->MODER |= (1<<5);
	GPIOA->AFR[0] |= (1<<8);
	GPIOA->AFR[0] |= (1<<9);
	GPIOA->AFR[0] |= (1<<10);
	GPIOA->AFR[0] &= ~(1<<11);
	//PA3
	GPIOA->MODER |= (0<<6);
	GPIOA->MODER |= (1<<7);
	GPIOA->AFR[0] |= (1<<12);
    GPIOA->AFR[0] |= (1<<13);
	GPIOA->AFR[0] |= (1<<14);
	GPIOA->AFR[0] &=~ (1<<15);

	//configure uart
	RCC->APB1ENR |= (1<<17);
	set_uart_baudrate(USART2,APB1_CLK,460800);
	USART2->CR1 = (1<<3) |(1<<2);
	USART2->CR1 |= (1<<13);

}
void set_uart_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t baudrate){
	USARTx->BRR = compute_uart_div(PeriphClk,baudrate);
}


uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t baudrate){
	return ((PeriphClk + (baudrate/2))/baudrate);
}
void uart_write(USART_TypeDef *USARTx,uint8_t ch){
	while(!(USARTx->SR & 1<<7));
    USARTx->DR = (ch & 0xFF);
}
char uart_read(USART_TypeDef *USARTx){
  while(!(USARTx->SR & (1<<5)));
  return (USARTx->DR);

}

void uart_send_string(USART_TypeDef *USARTx, const char *str){

    while (*str)
    {
        uart_write(USARTx, *str++);
    }
}
void uart_send_buffer(USART_TypeDef *USARTx,uint8_t* buf,uint16_t size){
	for(uint16_t i=0;i<size;i++){
		uart_write(USARTx,*(buf+i));
	}
}
