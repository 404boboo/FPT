/*
 * stepper_motor.h
 *
 *  Created on: Nov 7, 2024
 *      Author: Ahmed Bouras
 */

#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "stm32f7xx_hal.h"  // Adjust according to your STM32 series

// TIMER CONFIGURATIONS
extern TIM_HandleTypeDef htim2;  //  Motor 1 (X-axis)
extern TIM_HandleTypeDef htim3;  //  Motor 2 (X-axis)

// Motor Pin Definitions (You can adjust these for your setup)
#define MOTOR1_PWM_PIN        GPIO_PIN_10 // TIM2 channel 3
#define MOTOR1_PWM_PORT       GPIOB
#define MOTOR1_DIR_PIN        GPIO_PIN_9
#define MOTOR1_DIR_PORT       GPIOF
#define MOTOR1_TIM_CHANNEL    TIM_CHANNEL_3
#define MOTOR1_INDEX		  GPIO_PIN_14

#define MOTOR2_PWM_PIN        GPIO_PIN_6  // TIM3 channel 1
#define MOTOR2_PWM_PORT       GPIOA
#define MOTOR2_DIR_PIN        GPIO_PIN_10
#define MOTOR2_DIR_PORT       GPIOF
#define MOTOR2_TIM_CHANNEL	  TIM_CHANNEL_1
#define MOTOR2_INDEX		  GPIO_PIN_15

// Motor Parameters

// Speed is the period here which is in steps per second
#define MOTOR1_MIN_SPEED	  1000
#define MOTOR1_MAX_SPEED      5000
#define MOTOR2_MIN_SPEED	  1000
#define MOTOR2_MAX_SPEED      5000
extern uint32_t prescaler;
extern int32_t MotorX1StepCounter;
extern int32_t MotorX2StepCounter;
extern uint8_t Motor_Moving;
extern uint32_t Target_Steps;
extern uint32_t timerTick;






// MACROS
#define MOTOR_DIR_CW          GPIO_PIN_SET   // clockwise
#define MOTOR_DIR_CCW         GPIO_PIN_RESET  // counterclockwise


// Function Prototypes
void StepperMotor_Init(void);
void SetMotorSpeed(uint8_t motorID, uint32_t stepFrequency);
void SetMotorDirection(uint8_t motorID, GPIO_PinState direction);
void StartMotor(uint8_t motorID);
void StopMotor(uint8_t motorID);
void MoveMotorSteps(uint8_t motorID, int32_t steps, GPIO_PinState direction);
#endif /* __STEPPER_MOTOR_H */
#endif /* STEPPER_MOTOR_H_ */
