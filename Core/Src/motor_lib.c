#include "motor_lib.h"

static StepperMotor_t *g_steppers[MOTOR_LIB_MAX_STEPPERS] = { 0 };
static uint8_t g_stepperCount = 0;

/* =========================================================
   Internal helpers
   ========================================================= */

static HAL_StatusTypeDef MotorLib_RegisterStepper(StepperMotor_t *motor) {
    if (g_stepperCount >= MOTOR_LIB_MAX_STEPPERS)
        return HAL_ERROR;
    g_steppers[g_stepperCount++] = motor;
    return HAL_OK;
}

static uint32_t MotorLib_UsToTicks(uint32_t timerClockHz, uint32_t pulseUs) {
    uint64_t ticks = ((uint64_t)timerClockHz * pulseUs) / 1000000ULL;
    if (ticks == 0) ticks = 1;
    return (uint32_t)ticks;
}

static void Stepper_ApplyDirection(StepperMotor_t *motor, StepperDirection_t dir) {
    if (dir == STEPPER_DIR_CCW)
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_RESET);
}

/*
 * Apply a new speed to the running timer without stopping it.
 * Called from the ISR, so only register-level HAL macros are used.
 */
static void Stepper_ApplySpeedISR(StepperMotor_t *motor, uint32_t stepsPerSec) {
    if (stepsPerSec == 0) stepsPerSec = 1;

    uint32_t arr = (motor->timerClockHz / stepsPerSec) - 1U;
    if (arr < 20U) arr = 20U;

    uint32_t pulseTicks = MotorLib_UsToTicks(motor->timerClockHz, motor->stepPulseUs);
    if (pulseTicks > ((arr + 1U) / 2U))
        pulseTicks = (arr + 1U) / 2U;
    if (pulseTicks == 0U) pulseTicks = 1U;

    __HAL_TIM_SET_AUTORELOAD(motor->htim, arr);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pulseTicks);
}

/*
 * Linear interpolation between minSpeed and maxSpeed over accelSteps.
 *   phase: current step index within the ramp (0 … accelSteps-1)
 * Returns speed in steps/s.
 */
static uint32_t Stepper_RampSpeed(uint32_t minSpeed, uint32_t maxSpeed,
                                  uint32_t accelSteps, uint32_t phase) {
    if (accelSteps == 0) return maxSpeed;
    if (phase >= accelSteps) return maxSpeed;

    /* integer linear interpolation */
    uint32_t delta = maxSpeed - minSpeed;   /* maxSpeed >= minSpeed always */
    return minSpeed + (uint32_t)((uint64_t)delta * phase / accelSteps);
}

/* =========================================================
   STEPPER — public API
   ========================================================= */

HAL_StatusTypeDef Stepper_Init(StepperMotor_t *motor, TIM_HandleTypeDef *htim,
                               uint32_t channel, GPIO_TypeDef *dirPort,
                               uint16_t dirPin, uint32_t timerClockHz,
                               uint32_t stepPulseUs) {
    if (motor == NULL || htim == NULL || dirPort == NULL) return HAL_ERROR;

    motor->htim          = htim;
    motor->channel       = channel;
    motor->dirPort       = dirPort;
    motor->dirPin        = dirPin;
    motor->timerClockHz  = timerClockHz;
    motor->stepPulseUs   = stepPulseUs;

    motor->targetSteps   = 0;
    motor->currentSteps  = 0;
    motor->isRunning     = 0;
    motor->continuousMode = 0;
    motor->speedStepsPerSec = 0;

    /* acceleration fields — zeroed = no ramp */
    motor->minSpeedStepsPerSec = 0;
    motor->maxSpeedStepsPerSec = 0;
    motor->accelSteps          = 0;
    motor->currentSpeed        = 0;

    return MotorLib_RegisterStepper(motor);
}

HAL_StatusTypeDef Stepper_SetSpeed(StepperMotor_t *motor, uint32_t stepsPerSec) {
    if (motor == NULL || motor->htim == NULL) return HAL_ERROR;
    if (stepsPerSec == 0) stepsPerSec = 1;

    motor->speedStepsPerSec = stepsPerSec;

    uint32_t arr = (motor->timerClockHz / stepsPerSec) - 1U;
    if (arr < 20U) arr = 20U;

    __HAL_TIM_SET_AUTORELOAD(motor->htim, arr);

    uint32_t pulseTicks = MotorLib_UsToTicks(motor->timerClockHz, motor->stepPulseUs);
    if (pulseTicks > ((arr + 1U) / 2U))
        pulseTicks = (arr + 1U) / 2U;
    if (pulseTicks == 0U) pulseTicks = 1U;

    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pulseTicks);
    __HAL_TIM_SET_COUNTER(motor->htim, 0);

    return HAL_OK;
}

/* ---- legacy move (no ramp) ---- */
HAL_StatusTypeDef Stepper_MoveSteps(StepperMotor_t *motor, uint32_t steps,
                                    StepperDirection_t dir, uint32_t stepsPerSec) {
    if (motor == NULL || steps == 0 || stepsPerSec == 0) return HAL_ERROR;

    Stepper_Stop(motor);
    Stepper_ApplyDirection(motor, dir);

    motor->targetSteps   = steps;
    motor->currentSteps  = 0;
    motor->continuousMode = 0;
    motor->isRunning     = 1;

    /* disable ramp */
    motor->accelSteps          = 0;
    motor->minSpeedStepsPerSec = stepsPerSec;
    motor->maxSpeedStepsPerSec = stepsPerSec;
    motor->currentSpeed        = stepsPerSec;

    if (Stepper_SetSpeed(motor, stepsPerSec) != HAL_OK) return HAL_ERROR;

    __HAL_TIM_SET_COUNTER(motor->htim, 0);
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
    HAL_TIM_Base_Start_IT(motor->htim);

    if (motor->htim->Instance == TIM1) __HAL_TIM_MOE_ENABLE(motor->htim);

    return HAL_OK;
}

/* ---- move with trapezoid profile ---- */
HAL_StatusTypeDef Stepper_MoveStepsAccel(StepperMotor_t *motor, uint32_t steps,
                                         StepperDirection_t dir,
                                         uint32_t minSpeed, uint32_t maxSpeed,
                                         uint32_t accelSteps) {
    if (motor == NULL || steps == 0 || maxSpeed == 0) return HAL_ERROR;
    if (minSpeed == 0) minSpeed = 1;
    if (minSpeed > maxSpeed) minSpeed = maxSpeed;

    /*
     * Clamp accelSteps so ramp-up + ramp-down never exceed total steps.
     * For a symmetric trapezoid: each ramp ≤ steps/2.
     */
    if (accelSteps > steps / 2U) accelSteps = steps / 2U;

    Stepper_Stop(motor);
    Stepper_ApplyDirection(motor, dir);

    motor->targetSteps         = steps;
    motor->currentSteps        = 0;
    motor->continuousMode      = 0;
    motor->isRunning           = 1;

    motor->minSpeedStepsPerSec = minSpeed;
    motor->maxSpeedStepsPerSec = maxSpeed;
    motor->accelSteps          = accelSteps;
    motor->currentSpeed        = minSpeed;   /* start slow */

    /* configure timer at starting (slow) speed */
    Stepper_SetSpeed(motor, minSpeed);
    motor->speedStepsPerSec = minSpeed;

    __HAL_TIM_SET_COUNTER(motor->htim, 0);
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
    HAL_TIM_Base_Start_IT(motor->htim);

    if (motor->htim->Instance == TIM1) __HAL_TIM_MOE_ENABLE(motor->htim);

    return HAL_OK;
}

HAL_StatusTypeDef Stepper_RunContinuous(StepperMotor_t *motor,
                                        StepperDirection_t dir,
                                        uint32_t stepsPerSec) {
    if (motor == NULL || stepsPerSec == 0) return HAL_ERROR;

    /*
     * If motor is already running in continuous mode, only update speed.
     * This avoids stopping the timer (and losing step counts) on every
     * PD tick call (~every 10 ms).
     */
    if (motor->isRunning && motor->continuousMode) {
        Stepper_ApplyDirection(motor, dir);
        Stepper_ApplySpeedISR(motor, stepsPerSec);
        motor->currentSpeed = stepsPerSec;
        return HAL_OK;
    }

    /* First call — full init, but do NOT reset currentSteps so the
     * caller (EngineMoveAccel) controls the counter via Stepper_ResetCounter. */
    Stepper_Stop(motor);
    Stepper_ApplyDirection(motor, dir);

    motor->targetSteps    = 0;
    /* currentSteps intentionally NOT reset here */
    motor->continuousMode = 1;
    motor->isRunning      = 1;
    motor->accelSteps     = 0;
    motor->currentSpeed   = stepsPerSec;

    if (Stepper_SetSpeed(motor, stepsPerSec) != HAL_OK) return HAL_ERROR;

    __HAL_TIM_SET_COUNTER(motor->htim, 0);
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
    HAL_TIM_Base_Start_IT(motor->htim);

    if (motor->htim->Instance == TIM1) __HAL_TIM_MOE_ENABLE(motor->htim);

    return HAL_OK;
}

HAL_StatusTypeDef Stepper_Stop(StepperMotor_t *motor) {
    if (motor == NULL || motor->htim == NULL) return HAL_ERROR;

    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
    HAL_TIM_Base_Stop_IT(motor->htim);

    motor->isRunning      = 0;
    motor->continuousMode = 0;

    return HAL_OK;
}

uint8_t Stepper_IsRunning(StepperMotor_t *motor) {
    if (motor == NULL) return 0;
    return motor->isRunning;
}

uint32_t Stepper_GetCurrentSteps(StepperMotor_t *motor) {
    if (motor == NULL) return 0;
    return motor->currentSteps;
}

void Stepper_ResetCounter(StepperMotor_t *motor) {
    if (motor == NULL) return;
    motor->currentSteps = 0;
}

/* =========================================================
   TIM callbacks — called from main.c HAL overrides
   ========================================================= */

/*
 * HAL_TIM_PeriodElapsedCallback  →  MotorLib_TIM_PeriodElapsedCallback
 *
 * Fires once per PWM period (= once per step pulse).
 * We count steps here AND update the timer frequency to implement
 * the trapezoid speed profile.
 */
void MotorLib_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    for (uint8_t i = 0; i < g_stepperCount; i++) {
        StepperMotor_t *motor = g_steppers[i];
        if (motor == NULL || motor->htim != htim || !motor->isRunning)
            continue;

        motor->currentSteps++;

        /* ---------- trapezoid profile ---------- */
        if (motor->accelSteps > 0 && !motor->continuousMode) {
            uint32_t s    = motor->currentSteps;
            uint32_t total = motor->targetSteps;
            uint32_t ramp  = motor->accelSteps;
            uint32_t newSpeed;

            uint32_t stepsLeft = (s < total) ? (total - s) : 0;

            if (s < ramp) {
                /* acceleration phase */
                newSpeed = Stepper_RampSpeed(motor->minSpeedStepsPerSec,
                                             motor->maxSpeedStepsPerSec,
                                             ramp, s);
            } else if (stepsLeft < ramp) {
                /* deceleration phase */
                newSpeed = Stepper_RampSpeed(motor->minSpeedStepsPerSec,
                                             motor->maxSpeedStepsPerSec,
                                             ramp, stepsLeft);
            } else {
                /* cruise */
                newSpeed = motor->maxSpeedStepsPerSec;
            }

            if (newSpeed != motor->currentSpeed) {
                motor->currentSpeed = newSpeed;
                Stepper_ApplySpeedISR(motor, newSpeed);
            }
        }
        /* --------------------------------------- */

        if (!motor->continuousMode) {
            if (motor->currentSteps >= motor->targetSteps) {
                Stepper_Stop(motor);
            }
        }
    }
}

/*
 * HAL_TIM_PWM_PulseFinishedCallback  →  MotorLib_TIM_PWM_PulseFinishedCallback
 *
 * Some STM32 configurations use this callback instead of (or in addition to)
 * PeriodElapsed. We keep it for compatibility but guard against double-counting
 * by NOT updating the speed here — that is done in PeriodElapsed above.
 */
void MotorLib_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim == NULL) return;

    for (uint8_t i = 0; i < g_stepperCount; i++) {
        StepperMotor_t *motor = g_steppers[i];
        if (motor == NULL) continue;

        if (motor->htim == htim && motor->isRunning) {
            /*
             * Only count / stop here if PeriodElapsed is NOT already doing so
             * (i.e. when accelSteps == 0 and continuousMode == 0).
             * If accelSteps > 0 the PeriodElapsed callback owns everything.
             */
            if (motor->accelSteps == 0 && !motor->continuousMode) {
                motor->currentSteps++;
                if (motor->currentSteps >= motor->targetSteps) {
                    HAL_TIM_PWM_Stop_IT(motor->htim, motor->channel);
                    motor->isRunning = 0;
                }
            }
        }
    }
}

/* =========================================================
   SERVO
   ========================================================= */
HAL_StatusTypeDef Servo_Init(ServoMotor_t *servo, TIM_HandleTypeDef *htim,
                             uint32_t channel, uint32_t timerClockHz,
                             uint32_t pwmFreqHz, uint32_t minPulseUs,
                             uint32_t maxPulseUs, float minAngle, float maxAngle) {
    if (servo == NULL || htim == NULL) return HAL_ERROR;

    servo->htim         = htim;
    servo->channel      = channel;
    servo->timerClockHz = timerClockHz;
    servo->pwmFreqHz    = pwmFreqHz;
    servo->minPulseUs   = minPulseUs;
    servo->maxPulseUs   = maxPulseUs;
    servo->minAngle     = minAngle;
    servo->maxAngle     = maxAngle;
    servo->started      = 0;

    uint32_t arr = (timerClockHz / pwmFreqHz) - 1U;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);

    uint32_t midPulse = (minPulseUs + maxPulseUs) / 2U;
    uint32_t compare  = Servo_PulseUsToCompare(servo, midPulse);

    __HAL_TIM_SET_COMPARE(htim, channel, compare);
    __HAL_TIM_SET_COUNTER(htim, 0);

    return HAL_OK;
}

HAL_StatusTypeDef Servo_Start(ServoMotor_t *servo) {
    if (servo == NULL) return HAL_ERROR;
    servo->started = 1;
    return HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

HAL_StatusTypeDef Servo_Stop(ServoMotor_t *servo) {
    if (servo == NULL) return HAL_ERROR;
    servo->started = 0;
    return HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}

uint32_t Servo_PulseUsToCompare(ServoMotor_t *servo, uint32_t pulseUs) {
    if (servo == NULL) return 0;
    if (pulseUs < servo->minPulseUs) pulseUs = servo->minPulseUs;
    if (pulseUs > servo->maxPulseUs) pulseUs = servo->maxPulseUs;
    return MotorLib_UsToTicks(servo->timerClockHz, pulseUs);
}

HAL_StatusTypeDef Servo_SetPulseUs(ServoMotor_t *servo, uint32_t pulseUs) {
    if (servo == NULL) return HAL_ERROR;
    uint32_t compare = Servo_PulseUsToCompare(servo, pulseUs);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, compare);
    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngle(ServoMotor_t *servo, float angle) {
    if (servo == NULL) return HAL_ERROR;

    if (angle < servo->minAngle) angle = servo->minAngle;
    if (angle > servo->maxAngle) angle = servo->maxAngle;

    float angleRange = servo->maxAngle - servo->minAngle;
    float pulseRange = (float)(servo->maxPulseUs - servo->minPulseUs);

    float k = 0.0f;
    if (angleRange > 0.0f)
        k = (angle - servo->minAngle) / angleRange;

    uint32_t pulseUs = (uint32_t)(servo->minPulseUs + pulseRange * k);
    return Servo_SetPulseUs(servo, pulseUs);
}
