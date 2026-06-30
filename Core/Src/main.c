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

static void FallDown(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
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
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcValues, 4);

	Engine_Init(&engine);

	Motors_Disable();
//	***** Serves init ***************
	MoveServo(&engine, SERVO_ARM, 70, 1);
	MoveServo(&engine, SERVO_ARM2, 55, 1);
	MoveServo(&engine, SERVO_HAND, 130, 1);
	MoveServo(&engine, SERVO_HAND2, 100, 1);
	HAL_Delay(1000);
	DetachServo(&engine, SERVO_ARM);
	DetachServo(&engine, SERVO_ARM2);
	DetachServo(&engine, SERVO_HAND);
	DetachServo(&engine, SERVO_HAND2);
//	***** End Serves init ***************
//
//	while(1){
//		int a0 = adcValues[0];
//		int a1 = adcValues[1];
//		int a2 = adcValues[2];
//		int a3 = adcValues[3];
//	}

//	int colors[] = {
//			{0, 0, 0, 1},
//			{0, 1, 0, 0},
//			{0, 0, 0, 0}
//	};
//	colors[0][0];
//	int cntWhite = 0;
//	int cntYellow = 0;
//	int cntGreen = 0;
//	int cntBlue = 0;
//
//	int cntWhite1 = 6;
//	int cntYellow1 = 6;
//	int cntGreen1 = 6;
//	int cntBlue1 = 6;
//
//	for (int i = 0; i < 3; i++){
//		for (int j = 0; i < 4; j++){
//			if (colors[i][j] == 0){
//				cntWhite++;
//			}
//			if (colors[i][j] == 1){
//				cntYellow++;
//			}
//			if (colors[i][j] == 2){
//				cntGreen++;
//			}
//			if (colors[i][j] == 3){
//				cntBlue++;
//			}
//		}
//	}
//
	Wait_Start_Button();

	//FallDown();
	Motors_Enable();
	HAL_Delay(100);


	//FallDown();

		// GNAL GCI VRA
		EngineRotateAccel(&engine, 30, 0, 90);
		EngineMoveAccel(&engine, 140, 120, 0, 0);
		EngineRotateAccel(&engine, 20, 1, 90);


		EngineMoveAccel(&engine, 750, 140, 1, 0);
		EngineMoveAccel(&engine, 500, 50, 1, 1);
		EngineMoveAccel(&engine, 200, 120, 1, 0);
		EngineRotateAccel(&engine, 90, 0, 90);
		EngineMoveAccel(&engine, -70, 70, 0, 0);
		MoveServo(&engine, SERVO_ARM2, 117, 1);
		EngineMoveAccel(&engine, -120, 50, 0, 0);
		EngineMoveAccel(&engine, 125, 100, 0, 0);
		EngineFral(&engine, -50, -332, 25);
		EngineRotateAccel(&engine, 10, 1, 50);
		EngineMoveAccel(&engine, -290, 110, 0, 0);
		//EngineFral(&engine, -280, -300, 120);
		EngineFral(&engine, 240, 360, 80);
//		EngineMoveAccel(&engine, 350, 140, 0, 0);
		EngineRotateAccel(&engine, 38, 1, 50);
		EngineMoveAccel(&engine, 550, 130, 1, 0);
		EngineMoveAccel(&engine, 500, 50, 1, 1);
		EngineMoveAccel(&engine, 150, 100, 0, 0);
		EngineRotateAccel(&engine, 65, 1, 90);
		EngineMoveAccel(&engine, -270, 120, 0, 0);
		EngineMoveAccel(&engine, 30, 20, 0, 0);
		MoveServo(&engine, SERVO_ARM2, 55, 1);

		EngineMoveAccel(&engine, 270, 120, 0, 1);
		EngineMoveAccel(&engine, 165, 80, 0, 0);
		EngineRotateAccel(&engine, 152, 0, 120);
//		EngineRotateAccel(&engine, 70, 0, 90);
//		EngineMoveAccel(&engine, 150, 100, 0, 1);
//		EngineMoveAccel(&engine, 150, 100, 0, 0);
//		EngineRotateAccel(&engine, 90, 0, 90);
//
//
//
		EngineMoveAccel(&engine, 500, 100, 1, 0);
		EngineMoveAccel(&engine, 200, 50, 1, 1);
		//EngineMoveAccel(&engine, 300, 50, 0, 1);
		EngineRotateAccel(&engine, 185, 0, 120);
		EngineMoveAccel(&engine, -400, 80, 0, 0);
		EngineMoveAccel(&engine, 118, 100, 0, 0);
		EngineRotateAccel(&engine, 92, 1, 90);
		EngineMoveAccel(&engine, -235, 110, 0, 0);
		HAL_Delay(700);
		MoveServo(&engine, SERVO_ARM2, 118, 1);
		EngineMoveAccel(&engine, -200, 100, 0, 0);
		EngineMoveAccel(&engine, 150, 100, 1, 0);
		EngineFral(&engine, 150, 500, 35);
		EngineMoveAccel(&engine, 325, 130, 1, 0);
		EngineMoveAccel(&engine, 500, 50, 1, 1);
		EngineMoveAccel(&engine, 160, 120, 0, 0);
		EngineRotateAccel(&engine, 95, 1, 100);
	    EngineMoveAccel(&engine, 480, 120, 1, 0);
		EngineMoveAccel(&engine, 200, 50, 1, 1);
		EngineMoveAccel(&engine, 150, 120, 0, 0);
		EngineRotateAccel(&engine, 92, 1, 90);
		EngineMoveAccel(&engine, 40, 10, 1, 0);
		EngineMoveAccel(&engine, 200, 50, 1, 1);

		HAL_Delay(3000);
		EngineMoveAccel(&engine, -50, 30, 0, 0);
		EngineRotateAccel(&engine, 185, 1, 120);
		EngineMoveAccel(&engine, -200, 80, 0, 0);
		EngineMoveAccel(&engine, 200, 80, 0, 0);
		EngineMoveAccel(&engine, -300, 100, 0, 0);
		HAL_Delay(500);
		MoveServo(&engine, SERVO_ARM2, 55, 1);
		EngineMoveAccel(&engine, 200, 100, 0, 0);
		EngineRotateAccel(&engine, 180, 1, 120);



//		MoveServo(&engine, SERVO_ARM2, 121, 1);
//		EngineMoveAccel(&engine, -190, 120, 0, 0);
//		EngineMoveAccel(&engine,  190, 120, 0, 0);
//		EngineMoveAccel(&engine, -250, 120, 0, 0);
//		MoveServo(&engine, SERVO_ARM2, 55, 1);
//		EngineMoveAccel(&engine, 200, 100, 0, 0);
//		EngineRotateAccel(&engine, 180, 1, 140);
//		EngineMoveAccel(&engine, 100, 50, 1, 0);
//		EngineMoveAccel(&engine, 200, 50, 1, 1);
		EngineMoveAccel(&engine, -280, 100, 0, 0);
		EngineMoveAccel(&engine, 100, 100, 0, 0);
		EngineRotateAccel(&engine, 90, 0, 90);
		MoveServo(&engine, SERVO_ARM2, 90, 1);
		EngineMoveAccel(&engine, -300, 140, 0, 0);
		MoveServo(&engine, SERVO_ARM2, 119, 1);
		HAL_Delay(500);
		EngineFral(&engine, 355, 280, 150);
		EngineFral(&engine, 290, 365, 120);
		EngineMoveAccel(&engine, 150, 100, 1, 0);
		EngineMoveAccel(&engine, 350, 100, 1, 1);
		EngineMoveAccel(&engine, 200, 100, 1, 0);
//		EngineMoveAccel(&engine, 130, 100, 0, 0);
//		EngineRotateAccel(&engine, 21, 0, 90);
//		EngineMoveAccel(&engine, 320, 100, 0, 0);
//		EngineMoveAccel(&engine, 325, 100, 1, 0);
		EngineRotateAccel(&engine, 180, 1, 140);
		EngineMoveAccel(&engine, -230, 100, 0, 0);
		HAL_Delay(500);
		MoveServo(&engine, SERVO_ARM2, 55, 1);
		HAL_Delay(500);
		EngineMoveAccel(&engine, 100, 50, 1, 0);
		EngineMoveAccel(&engine, 200, 50, 1, 1);
		EngineMoveAccel(&engine, 155, 100, 0, 0);
		EngineRotateAccel(&engine, 91, 0, 90);
		EngineMoveAccel(&engine, 80, 50, 1, 0);
		EngineMoveAccel(&engine, 300, 50, 1, 1);
		EngineMoveAccel(&engine, 155, 100, 0, 0);

		EngineRotateAccel(&engine, 93, 0, 90);


	EngineMoveAccel(&engine, 80, 50, 1, 0);
	EngineMoveAccel(&engine, 200, 50, 1, 1);
	EngineMoveAccel(&engine, 10, 20, 0, 0);
	Kubik(&engine, 0, 6);
	Kubik(&engine, 1, 5);
	EngineFral(&engine, 67, 71, 15);
	Kubik(&engine, 0, 4);
	Kubik(&engine, 1, 3);
	HAL_Delay(300);
	EngineFral(&engine, 60, 55, 15);
	HAL_Delay(300);
	Kubik(&engine, 0, 2);
	Kubik(&engine, 1, 1);

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

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;

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
