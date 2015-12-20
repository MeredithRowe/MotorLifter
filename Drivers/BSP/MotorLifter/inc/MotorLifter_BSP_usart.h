#ifndef __MOTORLIFTER_BSP_USART_H
#define __MOTORLIFTER_BSP_USART_H

#include "stm32f4xx.h"

/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USART1 clock resources */
#define usrUSART    USART1
#define usrUSART_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define usrUSART_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define usrUSART_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define usrUSART_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define usrUSART_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for usrUSART Pins */
#define usrUSART_TX_PIN                    GPIO_PIN_9
#define usrUSART_TX_GPIO_PORT              GPIOA  
#define usrUSART_TX_AF                     GPIO_AF7_USART1
#define usrUSART_RX_PIN                    GPIO_PIN_10
#define usrUSART_RX_GPIO_PORT              GPIOA 
#define usrUSART_RX_AF                     GPIO_AF7_USART1

extern UART_HandleTypeDef Usart2Handle;
extern uint8_t gRxBuf[];
extern uint8_t gRxFlag;

typedef struct{
    uint8_t motorId;
    uint8_t usartCmd;
    uint32_t stepToGo;
}MotorLifter_t;
extern MotorLifter_t motorLifter;

void BSP_UsartInit(void);


#endif


