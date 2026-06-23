#ifndef ENGINE_H
#define ENGINE_H

#include "motor_lib.h"
#include <stdbool.h>

typedef struct {
	StepperMotor_t leftMotor;
	StepperMotor_t rightMotor;

	ServoMotor_t hand;
	ServoMotor_t arm;
	ServoMotor_t hand2;
	ServoMotor_t arm2;
} Engine_t;

typedef struct {
	StepperMotor_t carrierMotor;
} Carrier_t;

typedef enum {
	SERVO_HAND,
	SERVO_ARM,
	SERVO_HAND2,
	SERVO_ARM2
} ServoSelect_t;

void Engine_Init(Engine_t *eng);
void EngineStop(Engine_t *eng);
uint8_t EngineIsRunning(Engine_t *eng);

/* Stepper — constant speed */
void EngineMove(Engine_t *eng, int steps, uint16_t speed);
void EngineRotate(Engine_t *eng, uint16_t degree, uint16_t speed, bool IsLeft);

/* Stepper — PD continuous */
void EngineRunPD(Engine_t *eng, int32_t leftSpeed, int32_t rightSpeed);
void EngineTurnAccel(Engine_t *eng, uint16_t degree, bool IsLeft, uint16_t maxSpeed);
void EngineMoveAccel(Engine_t *eng, int steps, uint16_t maxSpeed, uint8_t followLine, uint8_t isStopWhenBlack);
void EngineRotateAccel(Engine_t *eng, uint16_t degree, bool IsLeft, uint16_t maxSpeed);
void EngineFral(Engine_t *eng, int steps1,int steps2, uint16_t maxSpeed1);
void MoveServo(Engine_t *eng, ServoSelect_t servo, uint16_t degree, uint8_t hold);
void DetachServo(Engine_t *eng, ServoSelect_t servo);

#endif /* ENGINE_H */
