#include "MotorLifter_BSP_usart.h"
#include "MotorLifter.h"
#include "motorcontrol.h"
#include "stm32f4xx_hal.h"

#define MY_UART_RX_DATA_LENTH   6

UART_HandleTypeDef Usart1Handle;
UART_HandleTypeDef Usart2Handle;

uint8_t gRxBuf[MY_UART_RX_DATA_LENTH] = { 0 };
uint8_t gRxFlag = 0;

typedef struct{
    enum MOTOR_LIFTER_AXIS_ID id;
    enum MOTORLIFTER_UART_INS_CMD ins;
    uint16_t para;
}MOTORLIFTER_UsartCmd_t;
static MOTORLIFTER_UsartCmd_t motorlifterUsartCmd;
 
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

static uint8_t MotorLifter_Value2Hex(uint8_t value)
{
    uint8_t temp;
    
    if('0' <= value && '9' >= value){
        temp = value - '0';
    }else if('a' <= value && 'f' >= value){
        temp = value - 'a' + 10;
    }else if('A' <= value && 'F' >= value){
        temp = value - 'A' + 10;
    }else{
        
    }
    
    return temp;
}

static uint16_t MotorLifter_UsartCmdParaProc(void)
{
    uint8_t bit3,bit2,bit1,bit0;
    uint16_t temp;
    
    bit3 = MotorLifter_Value2Hex(gRxBuf[2]);
    bit2 = MotorLifter_Value2Hex(gRxBuf[3]);
    bit1 = MotorLifter_Value2Hex(gRxBuf[4]);
    bit0 = MotorLifter_Value2Hex(gRxBuf[5]);
    
    temp = ((uint16_t)bit3<<12) + ((uint16_t)bit2<<8) + ((uint16_t)bit1<<4) + ((uint16_t)bit0);
    
    return temp;
}

static void MotorLifter_UsartCmdInsProc(MOTORLIFTER_UsartCmd_t usartCmd)
{
    int32_t temp;
    
    switch(usartCmd.ins){
        case UART_INS_CMD_FORWARD:
//            BSP_MotorControl_SetDirection(usartCmd.id, FORWARD);
            temp = usartCmd.para;
            BSP_MotorControl_GoTo(usartCmd.id, temp);
            break;
        case UART_INS_CMD_BACKWARD:
//            BSP_MotorControl_SetDirection(usartCmd.id, BACKWARD);
            temp = ~((uint32_t)usartCmd.para)+1;
            BSP_MotorControl_GoTo(usartCmd.id, temp);
            break;
        case UART_INS_CMD_HARDSTOP:
            BSP_MotorControl_HardStop(usartCmd.id);
            break;
        case UART_INS_CMD_SOFTSTOP:
            BSP_MotorControl_SoftStop(usartCmd.id);
            break;
        case UART_INS_CMD_SETHOME:
            BSP_MotorControl_SetHome(usartCmd.id);
            break;
        case UART_INS_CMD_SETMARK:
            BSP_MotorControl_SetMark(usartCmd.id);
            break;
        case UART_INS_CMD_GOTOPOS:
            BSP_MotorControl_GoTo(usartCmd.id, (int32_t)usartCmd.para);
            break;
        case UART_INS_CMD_GOHOME:
            BSP_MotorControl_GoHome(usartCmd.id);
            break;
        case UART_INS_CMD_GOMARK:
            BSP_MotorControl_GoMark(usartCmd.id);
            break;
        case UART_INS_CMD_RESETPOS:
            BSP_MotorControl_Run(MOTOR_LIFTER_X_AXIS_DEVICE_ID, FORWARD);
            BSP_MotorControl_Run(MOTOR_LIFTER_Y_AXIS_DEVICE_ID, FORWARD);
            break;
        case UART_INS_CMD_ACCELERATION:
            BSP_MotorControl_SetAcceleration(usartCmd.id, usartCmd.para);
            break;
        case UART_INS_CMD_DECELERATION:
            BSP_MotorControl_SetDeceleration(usartCmd.id, usartCmd.para);
            break;
        case UART_INS_CMD_MAXSPEED:
            BSP_MotorControl_SetMaxSpeed(usartCmd.id, usartCmd.para);
            break;
        case UART_INS_CMD_MINSPEED:
            BSP_MotorControl_SetMinSpeed(usartCmd.id, usartCmd.para);
            break;
        case UART_INS_CMD_STEPMODE:
            BSP_MotorControl_SelectStepMode((uint8_t)usartCmd.id, (motorStepMode_t)usartCmd.para);
            break;
        default:break;
    }
}

void MotorLifter_UsartCmdProc(void)
{
    motorlifterUsartCmd.id = (enum MOTOR_LIFTER_AXIS_ID)(gRxBuf[0] - '0');
    if(motorlifterUsartCmd.id >= MOTOR_LIFTER_AXIS_DEVICE_ID_BOUND){
        return;
    }
    
    motorlifterUsartCmd.ins = (enum MOTORLIFTER_UART_INS_CMD)MotorLifter_Value2Hex(gRxBuf[1]);
    if(motorlifterUsartCmd.ins >= UART_INS_CMD_BOUND){
        return;
    }
    
    motorlifterUsartCmd.para = MotorLifter_UsartCmdParaProc();
    
    MotorLifter_UsartCmdInsProc(motorlifterUsartCmd);

    motorlifterUsartCmd.id = MOTOR_LIFTER_AXIS_DEVICE_ID_BOUND;
    motorlifterUsartCmd.ins = UART_INS_CMD_BOUND;
    motorlifterUsartCmd.para = 0;
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
