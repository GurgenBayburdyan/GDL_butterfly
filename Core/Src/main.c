#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "engine.h"
#include "dma.h"

/* DMA destination — declared here, read as extern in engine.c */
volatile uint16_t adcValues[4];

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

Engine_t engine;
Carrier_t carrier;

void SystemClock_Config(void);

/* ===================== MOTOR ENABLE ===================== */

static void Motors_Enable(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

static void Motors_Disable(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/* ===================== BUTTON ===================== */

static void Wait_Start_Button(void) {
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) != GPIO_PIN_RESET) {
		HAL_Delay(1);
	}
	HAL_Delay(1000);
}

/* ===================== TIM CALLBACKS ===================== */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	MotorLib_TIM_PWM_PulseFinishedCallback(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	MotorLib_TIM_PeriodElapsedCallback(htim);
}


/* ===================== MAIN ===================== */

int main(void) {
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 4);

	Engine_Init(&engine);

	Motors_Disable();
//	***** Serves init ***************
	MoveServo(&engine, SERVO_ARM, 70, 1);
	MoveServo(&engine, SERVO_ARM2, 55, 1);
	MoveServo(&engine, SERVO_HAND, 160, 1);
	MoveServo(&engine, SERVO_HAND2, 95, 1);
	HAL_Delay(1000);
	DetachServo(&engine, SERVO_ARM);
	DetachServo(&engine, SERVO_ARM2);
	DetachServo(&engine, SERVO_HAND);
	DetachServo(&engine, SERVO_HAND2);
//	***** End Serves init ***************

//	while(1){
//		int a0 = adcValues[0];
//		int a1 = adcValues[1];
//		int a2 = adcValues[2];
//		int a3 = adcValues[3];
//	}

	Wait_Start_Button();
	Motors_Enable();
	HAL_Delay(100);

	//MoveServo(&engine, SERVO_ARM, 115, 0);
	//EngineMoveAccel(&engine, 1000, 70, 0, 1);

	EngineRotateAccel(&engine, 30, 0, 90);
	EngineMoveAccel(&engine, 140, 100, 0, 0);
	EngineRotateAccel(&engine, 20, 1, 90);
	EngineMoveAccel(&engine, 970, 100, 1, 0);
	EngineRotateAccel(&engine, 90, 0, 90);
	EngineMoveAccel(&engine, -50, 100, 0, 0);
	MoveServo(&engine, SERVO_ARM2, 117, 0);
	HAL_Delay(1000);
//	DetachServo(&engine, SERVO_ARM2);
	EngineRotateAccel(&engine, 65, 1, 90);
	EngineMoveAccel(&engine, -370, 150, 0, 0);
	EngineRotateAccel(&engine, 12, 0, 90);
	EngineMoveAccel(&engine, 1000, 150, 0, 0);
	EngineRotateAccel(&engine, 55,  1, 90);
	MoveServo(&engine, SERVO_ARM2, 55, 0);
	HAL_Delay(800);
	EngineMoveAccel(&engine, -220, 150, 0, 0);
	EngineRotateAccel(&engine, 45,  0, 90);
	EngineMoveAccel(&engine, 100, 100, 0, 1);
	EngineMoveAccel(&engine, 160, 100, 0, 0);
	EngineRotateAccel(&engine, 90,  0, 90);
	EngineMoveAccel(&engine, 300, 100, 1, 0);
	EngineMoveAccel(&engine, 300, 50, 1, 1);
	EngineRotateAccel(&engine, 180,  0, 90);
	EngineMoveAccel(&engine, -155, 100, 0, 0);
	EngineRotateAccel(&engine, 89,  1, 90);
	EngineMoveAccel(&engine, -265, 100, 0, 0);
	MoveServo(&engine, SERVO_ARM2, 117, 1);
	HAL_Delay(800);
	MoveServo(&engine, SERVO_ARM2, 117, 0);
	EngineMoveAccel(&engine, 600, 50, 1, 1);
	EngineRotateAccel(&engine, 35,  0, 90);
	EngineMoveAccel(&engine, -300, 100, 0, 0);




	EngineStop(&engine);
	Motors_Disable();

	while (1) {
		HAL_Delay(100);
	}
}

/* ===================== CLOCK CONFIG ===================== */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState        = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL9;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* ===================== ERROR HANDLER ===================== */

void Error_Handler(void) {
	__disable_irq();

	while (1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(200);
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif
