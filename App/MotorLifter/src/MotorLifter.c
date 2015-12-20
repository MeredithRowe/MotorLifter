#include "MotorLifter.h"
#include "stm32f4xx.h"
#include "motorcontrol.h"

#define MOTOR_LIFTER_X_AXIS_MAX_SPEED   1024
#define MOTOR_LIFTER_Y_AXIS_MAX_SPEED   1024
#define MOTOR_LIFTER_Z_AXIS_MAX_SPEED   2048

#define MOTOR_LIFTER_X_AXIS_MIN_SPEED   128
#define MOTOR_LIFTER_Y_AXIS_MIN_SPEED   128
#define MOTOR_LIFTER_Z_AXIS_MIN_SPEED   1024

#define MOTOR_LIFTER_X_AXIS_ACCELERATION    64
#define MOTOR_LIFTER_Y_AXIS_ACCELERATION    64
#define MOTOR_LIFTER_Z_AXIS_ACCELERATION    256

#define MOTOR_LIFTER_X_AXIS_DECELERATION    256
#define MOTOR_LIFTER_Y_AXIS_DECELERATION    256
#define MOTOR_LIFTER_Z_AXIS_DECELERATION    1024

static void MotorLifter_XaxisBoundDetectGPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    HAL_GPIO_DeInit(GPIOA, BSP_MOTOR_X_AXIS_BOUND_DETECT_PIN);
    
    GPIO_InitStructure.Pin = BSP_MOTOR_X_AXIS_BOUND_DETECT_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    
    HAL_GPIO_Init(BSP_MOTOR_X_AXIS_BOUND_DETECT_GPIO, &GPIO_InitStructure);
}

static void MotorLifter_YaxisBoundDetectGPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    HAL_GPIO_DeInit(GPIOA, BSP_MOTOR_Y_AXIS_BOUND_DETECT_PIN);
    
    GPIO_InitStructure.Pin = BSP_MOTOR_Y_AXIS_BOUND_DETECT_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    
    HAL_GPIO_Init(BSP_MOTOR_Y_AXIS_BOUND_DETECT_GPIO, &GPIO_InitStructure);
}

static void MotorLifter_ZaxisBoundDetectGPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    HAL_GPIO_DeInit(GPIOA, BSP_MOTOR_Z_AXIS_BOUND_DETECT_PIN);
    
    GPIO_InitStructure.Pin = BSP_MOTOR_Z_AXIS_BOUND_DETECT_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    
    HAL_GPIO_Init(BSP_MOTOR_Z_AXIS_BOUND_DETECT_GPIO, &GPIO_InitStructure);
}

static void MotorLifter_AxisBoundDetectGPIOInit(void)
{
    MotorLifter_XaxisBoundDetectGPIOInit();
    MotorLifter_YaxisBoundDetectGPIOInit();
    MotorLifter_ZaxisBoundDetectGPIOInit();
}

GPIO_PinState MotorLifter_XaxisBoundDetectGet(void)
{
     return HAL_GPIO_ReadPin(BSP_MOTOR_X_AXIS_BOUND_DETECT_GPIO, BSP_MOTOR_X_AXIS_BOUND_DETECT_PIN);
}

GPIO_PinState MotorLifter_YaxisBoundDetectGet(void)
{
     return HAL_GPIO_ReadPin(BSP_MOTOR_Y_AXIS_BOUND_DETECT_GPIO, BSP_MOTOR_Y_AXIS_BOUND_DETECT_PIN);
}

GPIO_PinState MotorLifter_ZaxisBoundDetectGet(void)
{
     return HAL_GPIO_ReadPin(BSP_MOTOR_Z_AXIS_BOUND_DETECT_GPIO, BSP_MOTOR_Z_AXIS_BOUND_DETECT_PIN);
}

void MotorLifter_AxisBoundDetectProc(void)
{
    if(!MotorLifter_XaxisBoundDetectGet()){
        BSP_MotorControl_HardStop(MOTOR_LIFTER_X_AXIS_DEVICE_ID);
        BSP_MotorControl_SetHome(MOTOR_LIFTER_X_AXIS_DEVICE_ID);
    }
    if(!MotorLifter_YaxisBoundDetectGet()){
        BSP_MotorControl_HardStop(MOTOR_LIFTER_Y_AXIS_DEVICE_ID);
        BSP_MotorControl_SetHome(MOTOR_LIFTER_Y_AXIS_DEVICE_ID);
    }
}

static void MotorLifter_XY_AxisInit(void)
{
    uint8_t id;
    
    /* Set X and Y axis 1/128 step mode */
    for(id=MOTOR_LIFTER_X_AXIS_DEVICE_ID;id<=MOTOR_LIFTER_Y_AXIS_DEVICE_ID;id++)
        BSP_MotorControl_SelectStepMode(id, STEP_MODE_1_64);
    
    BSP_MotorControl_SetMaxSpeed(MOTOR_LIFTER_X_AXIS_DEVICE_ID, MOTOR_LIFTER_X_AXIS_MAX_SPEED);
    BSP_MotorControl_SetMaxSpeed(MOTOR_LIFTER_Y_AXIS_DEVICE_ID, MOTOR_LIFTER_Y_AXIS_MAX_SPEED);
    
    BSP_MotorControl_SetMinSpeed(MOTOR_LIFTER_X_AXIS_DEVICE_ID, MOTOR_LIFTER_X_AXIS_MIN_SPEED);
    BSP_MotorControl_SetMinSpeed(MOTOR_LIFTER_Y_AXIS_DEVICE_ID, MOTOR_LIFTER_Y_AXIS_MIN_SPEED);
}

static void MotorLifter_Z_AxisInit(void)
{
    BSP_MotorControl_SelectStepMode(MOTOR_LIFTER_Z_AXIS_DEVICE_ID, STEP_MODE_FULL);
    BSP_MotorControl_SetAcceleration(MOTOR_LIFTER_Z_AXIS_DEVICE_ID, MOTOR_LIFTER_Z_AXIS_ACCELERATION);
    BSP_MotorControl_SetDeceleration(MOTOR_LIFTER_Z_AXIS_DEVICE_ID, MOTOR_LIFTER_Z_AXIS_DECELERATION);
    BSP_MotorControl_SetMaxSpeed(MOTOR_LIFTER_Z_AXIS_DEVICE_ID, MOTOR_LIFTER_Z_AXIS_MAX_SPEED);
    BSP_MotorControl_SetMinSpeed(MOTOR_LIFTER_Z_AXIS_DEVICE_ID, MOTOR_LIFTER_Z_AXIS_MIN_SPEED);
}

void MotorLifter_Init(void)
{
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 3);
    
    MotorLifter_AxisBoundDetectGPIOInit();
    
    MotorLifter_XY_AxisInit();
    
    MotorLifter_Z_AxisInit();
}




