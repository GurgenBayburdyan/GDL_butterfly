#include "engine.h"
#include "tim.h"
#include "gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* ===================== SPEED CONFIG ===================== */
#define BASE_SPEED   50
#define MAX_SPEED    150
#define MIN_SPEED    5

#define KP  20.0f
#define KD  10.0f

/* ===================== ADC / SENSOR CONFIG ===================== */
#define LEFT_CORNER_BLACK   2430
#define LEFT_CORNER_WHITE   1120

#define LEFT_LINE_BLACK     1400
#define LEFT_LINE_WHITE     480

#define RIGHT_LINE_BLACK    1400
#define RIGHT_LINE_WHITE    480

#define RIGHT_CORNER_BLACK  2430
#define RIGHT_CORNER_WHITE  1120


extern volatile uint16_t adcValues[4];
static float lastPosition = 0.0f;

/* ===================== SENSOR HELPERS ===================== */

static uint8_t NORMALIZE(uint16_t raw, uint16_t black, uint16_t white) {
	uint16_t value;

	if (black > white) {
		if (raw >= black) {
			return 100;
		} else if (raw <= white) {
			return 0;
		} else {
			value = ((uint16_t)(raw - white) * 100U) / (uint16_t)(black - white);
			return (uint8_t)value;
		}
	} else {
		if (raw <= black) {
			return 0;
		} else if (raw >= white) {
			return 100;
		} else {
			value = ((uint16_t)(raw - black) * 100U) / (uint16_t)(white - black);
			return (uint8_t)value;
		}
	}
}

static float ReadLinePosition(uint8_t ll, uint8_t rl) {
	int weightedSum = rl - ll;
	int totalWeight = ll + rl;

	if (totalWeight == 0)
		return lastPosition;   /* hold last known position if line is lost */

	lastPosition = (float)weightedSum / (float)totalWeight;
	return lastPosition;
}

/* ===================== APPLY PD ===================== */
static void ApplyPD(Engine_t *eng, float correction, uint16_t baseSpeed) {
	static int lastLeftSpeed  = -1000;
	static int lastRightSpeed = -1000;

	int leftSpeed  = (int)((float)baseSpeed - correction);
	int rightSpeed = (int)((float)baseSpeed + correction);

	if (leftSpeed  > MAX_SPEED) leftSpeed  = MAX_SPEED;
	if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
	if (leftSpeed  < MIN_SPEED) leftSpeed  = MIN_SPEED;
	if (rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;

	/* Do not restart motor every loop if speed did not change */
	if (leftSpeed != lastLeftSpeed || rightSpeed != lastRightSpeed) {
		EngineRunPD(eng, leftSpeed, rightSpeed);
		lastLeftSpeed  = leftSpeed;
		lastRightSpeed = rightSpeed;
	}
}

static void LineFollow_Tick(Engine_t *eng, uint16_t baseSpeed) {
	uint8_t leftLine  = NORMALIZE(adcValues[3], LEFT_LINE_BLACK,  LEFT_LINE_WHITE);
	uint8_t rightLine = NORMALIZE(adcValues[1], RIGHT_LINE_BLACK, RIGHT_LINE_WHITE);

	float error      = ReadLinePosition(leftLine, rightLine);
	static float lastError = 0.0f;
	float derivative = error - lastError;
	float correction = KP * error + KD * derivative;
	lastError        = error;

	ApplyPD(eng, correction, baseSpeed);
}


/* ================= ENGINE INIT / STOP ================= */

void Engine_Init(Engine_t *eng) {
	Stepper_Init(&eng->leftMotor,  &htim3, TIM_CHANNEL_1, GPIOB, GPIO_PIN_7, 1000000, 10);
	Stepper_Init(&eng->rightMotor, &htim4, TIM_CHANNEL_1, GPIOA, GPIO_PIN_7, 1000000, 10);

	Servo_Init(&eng->arm,   &htim1, TIM_CHANNEL_2, 1000000, 50, 500, 2500, 0.0f, 180.0f);
	Servo_Init(&eng->hand, &htim1, TIM_CHANNEL_3, 1000000, 50, 500, 2500, 0.0f, 180.0f);
	Servo_Init(&eng->arm2,  &htim2, TIM_CHANNEL_2, 1000000, 50, 500, 2500, 0.0f, 180.0f);
	Servo_Init(&eng->hand2, &htim1, TIM_CHANNEL_4, 1000000, 50, 500, 2500, 0.0f, 180.0f);

	Servo_Start(&eng->arm);
	Servo_Start(&eng->hand);
	Servo_Start(&eng->arm2);
	Servo_Start(&eng->hand2);
}

void EngineStop(Engine_t *eng) {
	Stepper_Stop(&eng->leftMotor);
	Stepper_Stop(&eng->rightMotor);
}

uint8_t EngineIsRunning(Engine_t *eng) {
	return Stepper_IsRunning(&eng->leftMotor) || Stepper_IsRunning(&eng->rightMotor);
}

/* ================= ENGINE MOVE — CONSTANT SPEED ================= */

void EngineMove(Engine_t *eng, int steps, uint16_t speed) {
	if (steps > 0) {
		Stepper_MoveSteps(&eng->leftMotor,  (uint16_t)steps, STEPPER_DIR_CCW, speed);
		Stepper_MoveSteps(&eng->rightMotor, (uint16_t)steps, STEPPER_DIR_CCW, speed);
	} else {
		uint16_t s = (uint16_t)(-steps);
		Stepper_MoveSteps(&eng->leftMotor,  s, STEPPER_DIR_CW, speed);
		Stepper_MoveSteps(&eng->rightMotor, s, STEPPER_DIR_CW, speed);
	}
}

void EngineRotate(Engine_t *eng, uint16_t degree, uint16_t speed, bool IsLeft) {
	uint16_t steps = degree * 3;

	if (IsLeft) {
		Stepper_MoveSteps(&eng->leftMotor,  steps, STEPPER_DIR_CCW, speed);
		Stepper_MoveSteps(&eng->rightMotor, steps, STEPPER_DIR_CW,  speed);
	} else {
		Stepper_MoveSteps(&eng->leftMotor,  steps, STEPPER_DIR_CW,  speed);
		Stepper_MoveSteps(&eng->rightMotor, steps, STEPPER_DIR_CCW, speed);
	}
}

void EngineRunPD(Engine_t *eng, int32_t leftSpeed, int32_t rightSpeed) {
	if (leftSpeed == 0) {
		Stepper_Stop(&eng->leftMotor);
	} else if (leftSpeed > 0) {
		Stepper_RunContinuous(&eng->leftMotor, STEPPER_DIR_CW, (uint32_t)leftSpeed);
	} else {
		Stepper_RunContinuous(&eng->leftMotor, STEPPER_DIR_CW, (uint32_t)(-leftSpeed));
	}

	if (rightSpeed == 0) {
		Stepper_Stop(&eng->rightMotor);
	} else if (rightSpeed > 0) {
		Stepper_RunContinuous(&eng->rightMotor, STEPPER_DIR_CCW, (uint32_t)rightSpeed);
	} else {
		Stepper_RunContinuous(&eng->rightMotor, STEPPER_DIR_CCW, (uint32_t)(-rightSpeed));
	}
}
void EngineFral(Engine_t *eng, int steps1,int steps2, uint16_t maxSpeed1){
	uint16_t minSpeed   = 10;
	uint16_t accelSteps = 150;


	int motorSteps1 = 0;
	int motorSteps2 = 0;
	int maxSpeed2 = 0;
	motorSteps1 = (int)((float)steps1 / 0.363f);
	motorSteps2 = (int)((float)steps2 / 0.363f);
	motorSteps1 = abs(motorSteps1);
	motorSteps2 = abs(motorSteps2);
	maxSpeed2 = abs((int)((float)steps2 / steps1 * maxSpeed1));

	if(steps1 > 0) Stepper_MoveStepsAccel(&eng->leftMotor,  (uint16_t)motorSteps1, STEPPER_DIR_CW, minSpeed, maxSpeed1, accelSteps);
	else Stepper_MoveStepsAccel(&eng->leftMotor,  (uint16_t)motorSteps1, STEPPER_DIR_CCW, minSpeed, maxSpeed1, accelSteps);
	if(steps2 > 0) Stepper_MoveStepsAccel(&eng->rightMotor, (uint16_t)motorSteps2, STEPPER_DIR_CCW, minSpeed, maxSpeed2, accelSteps);
	else Stepper_MoveStepsAccel(&eng->rightMotor, (uint16_t)motorSteps2, STEPPER_DIR_CW, minSpeed, maxSpeed2, accelSteps);

	while (EngineIsRunning(eng)) {
		HAL_Delay(1);
	}
}

