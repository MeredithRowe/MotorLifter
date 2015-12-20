#include "MotorLifter_BSP_usart.h"
#include "MotorLifter.h"
#include "motorcontrol.h"
#include "stm32f4xx_hal.h"

#define MY_UART_RX_DATA_LENTH   5

// static uint8_t gTxBuf;

UART_HandleTypeDef Usart2Handle;
UART_HandleTypeDef Usart1Handle;

uint8_t gRxBuf[MY_UART_RX_DATA_LENTH] = { 0 };
uint8_t gRxFlag = 0;
 
static volatile uint16_t gLastError;

static void Error_Handler(uint16_t error);
 
 #ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void BSP_USART1_Init(void)
{
  Usart1Handle.Instance = usrUSART;
  
  Usart1Handle.Init.BaudRate = 9600;
  Usart1Handle.Init.WordLength = UART_WORDLENGTH_8B;
  Usart1Handle.Init.StopBits = UART_STOPBITS_1;
  Usart1Handle.Init.Parity = UART_PARITY_NONE;
  Usart1Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  Usart1Handle.Init.Mode         = UART_MODE_TX_RX;
  Usart1Handle.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if(HAL_UART_Init(&Usart1Handle) !=HAL_OK)
  {
      Error_Handler(2);
  }
  
  if(HAL_UART_Receive_IT(&Usart1Handle, gRxBuf, MY_UART_RX_DATA_LENTH) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(2);
  }
  
  HAL_NVIC_SetPriority((IRQn_Type)(USART1_IRQn), 0x0D, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(USART1_IRQn));
}

void BSP_USART2_Init(void)
{
    HAL_UART_DeInit(&Usart2Handle);
    
    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART1 configured as follow:
      - Word Length = 9 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
    Usart2Handle.Instance          = USARTx;

    Usart2Handle.Init.BaudRate     = 9600;
    Usart2Handle.Init.WordLength   = UART_WORDLENGTH_8B;
    Usart2Handle.Init.StopBits     = UART_STOPBITS_1;
    Usart2Handle.Init.Parity       = UART_PARITY_NONE;
    Usart2Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    Usart2Handle.Init.Mode         = UART_MODE_TX_RX;
    Usart2Handle.Init.OverSampling = UART_OVERSAMPLING_16;

    if(HAL_UART_Init(&Usart2Handle) != HAL_OK){
        /* Initialization Error */
        Error_Handler(1);
    }

    if(HAL_UART_Receive_IT(&Usart2Handle, gRxBuf, MY_UART_RX_DATA_LENTH) != HAL_OK){
        /* Initialization Error */
        Error_Handler(1);
    }

    /* Enable and set USART2 Interrupt to the low priority */
    HAL_NVIC_SetPriority((IRQn_Type)(USART2_IRQn), 0x0E, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(USART2_IRQn));
}

void BSP_UsartInit(void)
{
    BSP_USART2_Init();

    printf("USART Init OK\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    gRxFlag = 1;
    
    if(huart->Instance == USARTx)
        HAL_UART_Receive_IT(&Usart2Handle, gRxBuf, MY_UART_RX_DATA_LENTH);
    if(huart->Instance == usrUSART)
        HAL_UART_Receive_IT(&Usart1Handle, gRxBuf, MY_UART_RX_DATA_LENTH);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&Usart2Handle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void MotorLifter_UsartCmdProc(void)
{
    uint32_t steptogo;
    
    steptogo = ((gRxBuf[1]-'0')<<24)+((gRxBuf[2]-'0')<<16)+((gRxBuf[3]-'0')<<8)+(gRxBuf[4]-'0');
    
    switch(gRxBuf[0]-'0'){
        case MOTOR_LIFTER_X_AXIS_DEVICE_ID:
            BSP_MotorControl_GoTo(MOTOR_LIFTER_X_AXIS_DEVICE_ID, steptogo);
            break;
        case MOTOR_LIFTER_Y_AXIS_DEVICE_ID:
            BSP_MotorControl_GoTo(MOTOR_LIFTER_Y_AXIS_DEVICE_ID, steptogo);
            break;
        case MOTOR_LIFTER_Z_AXIS_DEVICE_ID:
            BSP_MotorControl_GoTo(MOTOR_LIFTER_Z_AXIS_DEVICE_ID, steptogo);
            break;
        case MOTOR_LIFTER_XY_AXIS_DEVICE_ID_SELECT:
            
            break;
        default:break;
    }
    steptogo = 0;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
static void Error_Handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;

  /* Infinite loop */
  while(1)
  {
  }
}
