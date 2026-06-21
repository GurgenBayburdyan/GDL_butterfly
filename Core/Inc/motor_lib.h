#ifndef __MOTOR_LIB_H
#define __MOTOR_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define MOTOR_LIB_MAX_STEPPERS   8

/* =========================================================
   STEPPER
   ========================================================= */
typedef enum{
    STEPPER_DIR_CW = 0,
    STEPPER_DIR_CCW = 1
} StepperDirection_t;

typedef struct{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    GPIO_TypeDef *dirPort;
    uint16_t dirPin;

    uint32_t timerClockHz;      // after PSC
    uint32_t stepPulseUs;       // STEP pulse width in us

    volatile uint32_t targetSteps;
    volatile uint32_t currentSteps;
    volatile uint8_t isRunning;
    volatile uint8_t continuousMode;

    uint32_t speedStepsPerSec;

    /* --- Acceleration / deceleration (trapezoid profile) --- */
    uint32_t minSpeedStepsPerSec;   // start/end speed (e.g. 10–20 steps/s)
    uint32_t maxSpeedStepsPerSec;   // cruise speed
    uint32_t accelSteps;            // steps used for ramp-up (and ramp-down)
    volatile uint32_t currentSpeed; // current speed in steps/s (updated in ISR)
} StepperMotor_t;

HAL_StatusTypeDef Stepper_Init(StepperMotor_t *motor,
                               TIM_HandleTypeDef *htim,
                               uint32_t channel,
                               GPIO_TypeDef *dirPort,
                               uint16_t dirPin,
                               uint32_t timerClockHz,
                               uint32_t stepPulseUs);

/* Speed */
HAL_StatusTypeDef Stepper_SetSpeed(StepperMotor_t *motor, uint32_t stepsPerSec);

/* Move fixed steps — accelSteps=0 means no ramp (legacy behaviour) */
HAL_StatusTypeDef Stepper_MoveSteps(StepperMotor_t *motor,
                                    uint32_t steps,
                                    StepperDirection_t dir,
                                    uint32_t stepsPerSec);

/* Move with trapezoid profile */
HAL_StatusTypeDef Stepper_MoveStepsAccel(StepperMotor_t *motor,
                                         uint32_t steps,
                                         StepperDirection_t dir,
                                         uint32_t minSpeed,
                                         uint32_t maxSpeed,
                                         uint32_t accelSteps);

/* Continuous run */
HAL_StatusTypeDef Stepper_RunContinuous(StepperMotor_t *motor,
                                        StepperDirection_t dir,
                                        uint32_t stepsPerSec);

/* Stop */
HAL_StatusTypeDef Stepper_Stop(StepperMotor_t *motor);

/* Status */
uint8_t Stepper_IsRunning(StepperMotor_t *motor);
uint32_t Stepper_GetCurrentSteps(StepperMotor_t *motor);
void Stepper_ResetCounter(StepperMotor_t *motor);

/* Call these from the corresponding HAL callbacks in main.c */
void MotorLib_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void MotorLib_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


/* =========================================================
   SERVO
   ========================================================= */
typedef struct{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    uint32_t timerClockHz;
    uint32_t pwmFreqHz;

    uint32_t minPulseUs;
    uint32_t maxPulseUs;

    float minAngle;
    float maxAngle;

    uint8_t started;
} ServoMotor_t;


HAL_StatusTypeDef Servo_Init(ServoMotor_t *servo,
                             TIM_HandleTypeDef *htim,
                             uint32_t channel,
                             uint32_t timerClockHz,
                             uint32_t pwmFreqHz,
                             uint32_t minPulseUs,
                             uint32_t maxPulseUs,
                             float minAngle,
                             float maxAngle);

HAL_StatusTypeDef Servo_Start(ServoMotor_t *servo);
HAL_StatusTypeDef Servo_Stop(ServoMotor_t *servo);

HAL_StatusTypeDef Servo_SetPulseUs(ServoMotor_t *servo, uint32_t pulseUs);
HAL_StatusTypeDef Servo_SetAngle(ServoMotor_t *servo, float angle);

uint32_t Servo_PulseUsToCompare(ServoMotor_t *servo, uint32_t pulseUs);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_LIB_H */
