/*
 * stepper_motor.c
 *
 *  Created on: Nov 7, 2024
 *      Author: Ahmed Bouras
 */


#include "stepper_motor.h"
#include "stm32f7xx_hal.h"  // Adjust according to your STM32 series

uint32_t prescaler = 107;
int32_t MotorX1StepCounter = 0;
int32_t MotorX2StepCounter = 0;
uint8_t Motor_Moving = 0;
uint32_t Target_Steps = 0;

void StepperMotor_Init(void) {

    SetMotorSpeed(1, MOTOR1_MIN_SPEED);
    SetMotorSpeed(2, MOTOR1_MIN_SPEED);
}

void SetMotorDirection(uint8_t motorID, GPIO_PinState direction) {
    if (motorID == 1) {
        HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, direction);
        HAL_Delay(1);
    } else if (motorID == 2) {
        HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, direction);
        HAL_Delay(1);
    }
}

void SetMotorSpeed(uint8_t motorID, uint32_t stepFrequency) {
		uint32_t timerClock = HAL_RCC_GetPCLK1Freq()  / prescaler+1;
   	    uint32_t ARR = (timerClock / stepFrequency ) - 1;
    if (motorID == 1) {
        __HAL_TIM_SET_AUTORELOAD(&htim2, ARR);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_TIM_CHANNEL, ARR / 2);
    }
    else if(motorID == 2){
        __HAL_TIM_SET_AUTORELOAD(&htim3, ARR);
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR2_TIM_CHANNEL, ARR / 2);
    }
}



void StartMotor(uint8_t motorID) {
    if (motorID == 1) {
    	HAL_TIM_PWM_Start(&htim2, MOTOR1_TIM_CHANNEL);
    } else if (motorID == 2) {
    	HAL_TIM_PWM_Start(&htim3, MOTOR2_TIM_CHANNEL);
    }
}

void StopMotor(uint8_t motorID) {
    if (motorID == 1) {
        HAL_TIM_PWM_Stop(&htim2, MOTOR1_TIM_CHANNEL);

    } else if (motorID == 2) {
        HAL_TIM_PWM_Stop(&htim3, MOTOR2_TIM_CHANNEL);
    }
}


void MoveMotorSteps(uint8_t motorID, int32_t steps, GPIO_PinState direction) {
    Target_Steps = steps * 8;
    Motor_Moving = 1;

    uint32_t lastCounterValue = (motorID == 1) ? __HAL_TIM_GET_COUNTER(&htim2) : __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t currentCounterValue;
    int32_t StepCounter = 0;

    SetMotorDirection(motorID, direction);
    StartMotor(motorID);

    while (StepCounter < Target_Steps) {
        currentCounterValue = (motorID == 1) ? __HAL_TIM_GET_COUNTER(&htim2) : __HAL_TIM_GET_COUNTER(&htim3);

        // Calculate elapsed ticks (handle overflow)
        uint32_t tickCount;
        if (currentCounterValue >= lastCounterValue) {
            tickCount = currentCounterValue - lastCounterValue;
        } else {
            // Timer overflow
            tickCount = (htim2.Init.Period + 1) - lastCounterValue + currentCounterValue;
        }

        StepCounter += tickCount;  // Increment steps based on ticks
        lastCounterValue = currentCounterValue;


        printf("StepCounter: %ld, TickCount: %lu\n", StepCounter, tickCount);
    }

   StopMotor(motorID);  // Stop motor after achieving Target_Steps
    Motor_Moving = 0;    // Clear movement flag
}

