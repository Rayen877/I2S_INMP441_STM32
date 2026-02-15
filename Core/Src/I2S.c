
#include "I2S.h"

#define CLK_GPIOB_EN      (1U << 1)
#define CLK_GPIOC_EN      (1U << 2)
#define CLK_DMA1_EN       (1U << 21)
#define CLK_SPI2_EN       (1U << 14) //APB1
#define CR_PLLI2S_ON       (1U << 26)
#define CR_PLLI2S_RDY      (1U << 27)

#define DMA_EN            (1U << 0)
#define DMA_HTIE          (1U << 3)
#define DMA_TCIE          (1U << 4)
#define DMA_CIRC          (1U << 8)
#define DMA_MINC          (1U << 10)

#define DMA_PL_HIGH       (2U << 16)
#define DMA_HTIF3         (1U << 10)
#define DMA_TCIF3         (1U << 11)

#define DMA_CHTIF3        (1U << 10)
#define DMA_CTCIF3        (1U << 11)

#define SPI_RxDMAEN       (1U << 0)
#define I2S_ENABL       (1U << 10)
#define DMA1_STREAM3_IRQn   13
#define NVIC_ISER0   (*(volatile uint32_t*)0xE000E100U)
#define NVIC_ENABLE_IRQ(x)  (NVIC_ISER0 = (1U << (x)))

int32_t i2s_rx_buffer[AUDIO_BUFFER_SIZE];
int16_t audio_tx_buffer[AUDIO_BUFFER_SIZE/2];

volatile uint8_t half_complete = 0;
volatile uint8_t full_complete = 0;


void i2s2_dma_init(void)
{
	RCC->AHB1ENR |= CLK_GPIOB_EN ;
	RCC->AHB1ENR |= CLK_GPIOC_EN;
	//pb10
/*	GPIOB->MODER |=  (2 << (10*2));
	GPIOB->AFR[1] &= ~(0xF << ((10-8)*4));
	GPIOB->AFR[1] |=  (5 << ((10-8)*4));*/

	// High speed
	GPIOB->OSPEEDR |= (3 << (10*2));
	//pb13
	GPIOB->MODER &= ~(3 << (13 * 2));
	GPIOB->MODER |=  (2 << (13 * 2));      // AF
	GPIOB->AFR[1] &= ~(0xF << ((13 - 8) * 4));
	GPIOB->AFR[1] |=  (5 << ((13 - 8) * 4)); // AF5
	GPIOB->OSPEEDR |= (3 << (13 * 2));
	//pb12
	GPIOB->MODER &= ~(3 << (12*2));
	GPIOB->MODER |=  (2 << (12*2));

	GPIOB->AFR[1] &= ~(0xF << ((12-8)*4));
	GPIOB->AFR[1] |=  (5 << ((12-8)*4));

	GPIOB->OSPEEDR |= (3 << (12*2));
    //pc3
	GPIOC->MODER &= ~(3 << (3*2));
	GPIOC->MODER |=  (2 << (3*2));

	GPIOC->AFR[0] &= ~(0xF << (3*4));
	GPIOC->AFR[0] |=  (5 << (3*4));

	GPIOC->OSPEEDR |= (3 << (3*2));

    RCC->APB1ENR |= CLK_SPI2_EN;      // SPI2
    RCC->AHB1ENR |= CLK_DMA1_EN;      // DMA1
    RCC->PLLCFGR &= ~(0x3F);
    RCC->PLLCFGR |= 8; // PLLM = 8
    RCC->PLLI2SCFGR = (192 << 6) |   // PLLI2SN
                      (5   << 28);   // PLLI2SR

    RCC->CR |= CR_PLLI2S_ON;
    while(!(RCC->CR & CR_PLLI2S_RDY));

    SPI2->I2SCFGR =
    		    (1 << 11) |    // I2S mode
    		    (0b10 << 8) |  // master receiver
    		    (0 << 4) |     // Philips
    		    (0b10 << 1)|   // 32bit data length
                (0 << 7);       // cpol low
    SPI2->I2SPR = (12) |      // i2sDiv
                  (1 << 8);   // ODD bit
    SPI2->CR2 |= SPI_RxDMAEN;
    DMA1_Stream3->CR &= ~DMA_EN;
    while(DMA1_Stream3->CR & DMA_EN);
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)i2s_rx_buffer;
    DMA1_Stream3->NDTR = AUDIO_BUFFER_SIZE;

    DMA1_Stream3->CR =
          (0 << 25) |               // Channel 0
		  DMA_MINC |           // memory increment
		  DMA_CIRC |           // circular mode
          DMA_TCIE |           // transfer complete interupt
          DMA_HTIE |           // half transfer interrupt
		  DMA_PL_HIGH  |           // high priority
		  (2 << 11) |       // peripheral size 32bit
		  (2 << 13) |     // memory size 32-bit
          (0 << 6);                 // peripheral to memory
    NVIC_ENABLE_IRQ(DMA1_STREAM3_IRQn);
    SPI2->I2SCFGR |= I2S_ENABL;
    DMA1_Stream3->CR |= DMA_EN;

}

void DMA1_Stream3_IRQHandler(void)
{
    if(DMA1->LISR & DMA_HTIF3)
    {
        DMA1->LIFCR |= DMA_CHTIF3;
        half_complete = 1;
    }

    if(DMA1->LISR & DMA_TCIF3)
    {
        DMA1->LIFCR |= DMA_CTCIF3;
        full_complete = 1;
    }
}

