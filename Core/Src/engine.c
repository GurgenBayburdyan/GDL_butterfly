#include "engine.h"
#include "tim.h"
#include "gpio.h"
#include <stdbool.h>
#include <stdint.h>

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

/* ================= ENGINE MOVE — ACCELERATION ================= */

void EngineMoveAccel(Engine_t *eng, int steps, uint16_t maxSpeed, uint8_t followLine, uint8_t isStopWhenBlack) {
	uint16_t minSpeed   = 10;
	uint16_t accelSteps = 150;

	uint16_t leftLight = 0;
	uint16_t rightLight = 0;

	int motorSteps = 0;

	if (!followLine) {
		motorSteps = (int)((float)steps / 0.363f);
		if (motorSteps > 0) {
			Stepper_MoveStepsAccel(&eng->leftMotor,  (uint16_t)motorSteps, STEPPER_DIR_CW, minSpeed, maxSpeed, accelSteps);
			Stepper_MoveStepsAccel(&eng->rightMotor, (uint16_t)motorSteps, STEPPER_DIR_CCW, minSpeed, maxSpeed, accelSteps);
		} else {
			uint16_t s = (uint16_t)(-motorSteps);
			Stepper_MoveStepsAccel(&eng->leftMotor,  s, STEPPER_DIR_CCW, minSpeed, maxSpeed, accelSteps);
			Stepper_MoveStepsAccel(&eng->rightMotor, s, STEPPER_DIR_CW, minSpeed, maxSpeed, accelSteps);
		}

		while (EngineIsRunning(eng) && leftLight < 2000 && rightLight < 2000) {
			if (isStopWhenBlack){
				leftLight  = adcValues[2];
				rightLight = adcValues[0];
			}
			HAL_Delay(1);
		}

	} else {
		motorSteps = (int)((float)steps * 2.69f);
		if (motorSteps <= 0) return;

		uint32_t target    = (uint32_t)motorSteps;
		uint32_t rampSteps = accelSteps < target / 2U ? accelSteps : target / 2U;

		LineFollow_Tick(eng, minSpeed);

		Stepper_ResetCounter(&eng->leftMotor);
		Stepper_ResetCounter(&eng->rightMotor);

		while (leftLight < 2000 && rightLight < 2000) {
			if (isStopWhenBlack){
				leftLight  = adcValues[2];
				rightLight = adcValues[0];
			}
			uint32_t done = Stepper_GetCurrentSteps(&eng->leftMotor);
			if (done >= target) break;

			uint32_t left = target - done;
			uint16_t currentSpeed;
			if (done < rampSteps) {
				currentSpeed = minSpeed + (uint16_t)((uint32_t)(maxSpeed - minSpeed) * done / rampSteps);
			} else if (left < rampSteps) {
				currentSpeed = minSpeed + (uint16_t)((uint32_t)(maxSpeed - minSpeed) * left / rampSteps);
			} else {
				currentSpeed = maxSpeed;
			}

			HAL_Delay(10);
			LineFollow_Tick(eng, currentSpeed);
		}

		EngineStop(eng);
	}
}

void EngineRotateAccel(Engine_t *eng, uint16_t degree, bool IsLeft, uint16_t maxSpeed) {
	uint16_t minSpeed   = 10;
	uint16_t accelSteps = 100;
	uint16_t steps      = (uint16_t)((float)degree * 4.9f);

	if (IsLeft) {
		Stepper_MoveStepsAccel(&eng->leftMotor,  steps, STEPPER_DIR_CCW, minSpeed, maxSpeed, accelSteps);
		Stepper_MoveStepsAccel(&eng->rightMotor, steps, STEPPER_DIR_CCW,  minSpeed, maxSpeed, accelSteps);
	} else {
		Stepper_MoveStepsAccel(&eng->leftMotor,  steps, STEPPER_DIR_CW,  minSpeed, maxSpeed, accelSteps);
		Stepper_MoveStepsAccel(&eng->rightMotor, steps, STEPPER_DIR_CW, minSpeed, maxSpeed, accelSteps);
	}

	while (EngineIsRunning(eng)) {
		HAL_Delay(1);
	}
}

/* ================= SERVO METHODS ================= */
void MoveServo(Engine_t *eng, ServoSelect_t servo, uint16_t degree, uint8_t hold) {
	ServoMotor_t *s;

	switch (servo) {
		case SERVO_HAND:  s = &eng->hand;  break;
		case SERVO_ARM:   s = &eng->arm;   break;
		case SERVO_HAND2: s = &eng->hand2; break;
		case SERVO_ARM2:  s = &eng->arm2;  break;
		default: return;
	}

	if (!s->started) Servo_Start(s);
	Servo_SetAngle(s, (float)degree);
	if (!hold) {
		HAL_Delay(300);
		Servo_Stop(s);
	}
}

void DetachServo(Engine_t *eng, ServoSelect_t servo) {
	ServoMotor_t *s;

	switch (servo) {
		case SERVO_HAND:  s = &eng->hand;  break;
		case SERVO_ARM:   s = &eng->arm;   break;
		case SERVO_HAND2: s = &eng->hand2; break;
		case SERVO_ARM2:  s = &eng->arm2;  break;
		default: return;
	}
	Servo_Stop(s);
}
