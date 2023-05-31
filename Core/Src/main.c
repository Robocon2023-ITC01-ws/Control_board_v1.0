/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t data[2] = {0};
uint16_t data_16 = 0;
float speed = 0;

uint8_t load = 0;
uint8_t input_degree = 150; // 90

float speed_up;
uint8_t up_control;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PIDController MPID;
#define MKp 1.14
#define MKi 25.59
#define MKd 0.0

// need to configuration
#define CPR 64
#define Sample_time 10 // ms

#define pi 3.1415
#define Motor1 0
#define Motor2 1
volatile uint8_t nowA[2];
volatile uint8_t nowB[2];
volatile uint8_t lastA[2];
volatile uint8_t lastB[2];
volatile uint8_t dir[2];
uint16_t cnt[2];
uint16_t Enc_count[2];

uint16_t count[2]; // count pulse from encoder
uint16_t new_count[2];
uint8_t count_state[2];
uint16_t diff[2]; // difference between count and new_count in a sample time

float speedM[2];
float rdps[2];

float Motor1_speed = 0;
float Motor2_speed = 0;
float V1 = 0; // target speed of motor1
float V2 = 0; // target speed of motor2
float pwm_M1 = 0;
float pwm_M2 = 0;
float M_shooter1 = 0;
float M_shooter2 = 0;

///// reloading
uint8_t reload_feedback;
uint8_t reload_control;
int8_t reload_error;

uint8_t right_feedback;
uint8_t right_control;
int8_t right_error;

uint8_t left_feedback;
uint8_t left_control;
int8_t left_error;

uint8_t bit1;
uint8_t bit2;
uint8_t bit3;
uint8_t bit4;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[4];
uint32_t TxMailbox;
uint8_t cntt;

float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output)
{

	return (float)((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		cntt = 0;
	}

	if (RxHeader.DLC == 3 && RxHeader.StdId == 0x222)
	{
		data_16 = (RxData[0] << 8) | RxData[1];
		speed = map(data_16,0,65535,0,1500);
		load = RxData[2];
	}
	if (RxHeader.DLC == 2 && RxHeader.StdId == 0x444){
		reload_feedback = RxData[0] >> 0 & 1;
		bit1 = RxData[0] >> 1 & 1;
		bit2 = RxData[0] >> 2 & 1;
		right_feedback = (bit2 << 1 ) + bit1;
		bit3 = RxData[0] >> 3 & 1;
		bit4 = RxData[0] >> 4 & 1;
		left_feedback = (bit4 << 1 ) + bit3;

	}
	if (RxHeader.DLC == 1 && RxHeader.StdId == 0x450){
		if(reload_error == 0)
		{
			reload_control = 1;
		}
	}
	if (RxHeader.DLC == 1 && RxHeader.StdId == 0x460){
		if(RxData[0] == 1){
			input_degree = 90;
		}
		else if (RxData[0] == 0){
			input_degree = 150;
		}
	}
}

void servo_rotation(uint8_t degree){
	float y = 0.556 * (float) degree + 25;
	uint8_t pwm = round(y);
	TIM3->CCR2 = pwm;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t encoder(int i)
{
	if (nowA[i] != lastA[i])
	{
		lastA[i] = nowA[i];
		if (lastA[i] == 0)
		{
			if (nowB[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowB[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	if (nowB[i] != lastB[i])
	{
		lastB[i] = nowB[i];
		if (lastB[i] == 0)
		{
			if (nowA[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowA[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	return cnt[i];
}
float Motors_RPS(int j, float SampleTime, float N_round)
{
	new_count[Motor1] = Enc_count[0];
	new_count[Motor2] = Enc_count[1];

	count_state[Motor1] = !dir[0];
	count_state[Motor2] = !dir[1];

	if (count_state[j])
	{
		if (new_count[j] <= count[j])
		{ // Check for counter underflow
			diff[j] = count[j] - new_count[j];
		}
		else
		{
			diff[j] = (65536 - new_count[j]) + count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime) * -1.0;
	}
	else
	{
		if (new_count[j] >= count[j])
		{ // Check for counter overflow
			diff[j] = new_count[j] - count[j];
		}
		else
		{
			diff[j] = (65536 - count[j]) + new_count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
	}

	rdps[j] = -2.0f * pi * speedM[j];
	count[j] = new_count[j];

	return rdps[j];
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	PID_Init(&MPID, 2);
	MPID.T = 0.01; // T = 20ms
	MPID.limMax = 1000;
	MPID.limMin = -10;
	MPID.limMaxInt = 1000;
	MPID.limMinInt = -10;
	MPID.tau = 0; // for Kd

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);						// TIMER INTERUPT
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);				// M1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);				// M2
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);				// M1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);				// M2

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);				// servo

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);				// M1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);				// M2
	HAL_CAN_Start(&hcan);

	// Activate the notification
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GetTick() < 5000){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		  HAL_Delay(100);
	  }
	  servo_rotation(input_degree);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == En1_C1_Pin || En1_C2_Pin)
	{ // ENCODER Motor 1
		nowA[0] = HAL_GPIO_ReadPin(En1_C1_GPIO_Port, En1_C1_Pin);
		nowB[0] = HAL_GPIO_ReadPin(En1_C2_GPIO_Port, En1_C2_Pin);
		Enc_count[0] = encoder(0);
	}
	if (GPIO_Pin == En2_C1_Pin || En2_C2_Pin)
	{ // ENCODER Motor 1
		nowA[1] = HAL_GPIO_ReadPin(En2_C1_GPIO_Port, En2_C1_Pin);
		nowB[1] = HAL_GPIO_ReadPin(En2_C2_GPIO_Port, En2_C2_Pin);
		Enc_count[1] = encoder(1);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2){
		// PID // need to change for using
		M_shooter1 = PID(&MPID, speed, Motor1_speed, MKp, MKi, MKd, Motor1);
		M_shooter2 = PID(&MPID, speed, Motor2_speed, MKp, MKi, MKd, Motor2);
		// feedback speed
		Motor1_speed = (float)fabs(Motors_RPS(Motor1, Sample_time, CPR));
		Motor2_speed = (float)fabs(Motors_RPS(Motor2, Sample_time, CPR));

		// dir
			// need to config
		HAL_GPIO_WritePin(Sh_M1_dir_GPIO_Port, Sh_M1_dir_Pin, 0);
		HAL_GPIO_WritePin(Sh_M2_dir_GPIO_Port, Sh_M2_dir_Pin, 0);

		// pwm
		if (M_shooter1 > 20)	// M1
		{
			TIM1->CCR2 = M_shooter1;
		}
		else
		{
			TIM1->CCR2 = 0;
		}
		if (M_shooter2 > 20)	// M2
		{
			TIM1->CCR1 = M_shooter2;
		}
		else
		{
			TIM1->CCR1 = 0;
		}
		if(load == 1){		// load for shoot // M3
			TIM4->CCR3 = 500;
		}
		else {
			TIM4->CCR3 = 0;
		}
//
		if (pwm_M1 > 10){
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 1);
			TIM4->CCR2 = pwm_M1;
		}
		else if (pwm_M1 < -10){
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 0);
			TIM4->CCR2 = -1 * pwm_M1;
		}
		else{
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 0);
			TIM4->CCR2 = 0;
		}
		if (pwm_M2 > 10){
			HAL_GPIO_WritePin(M2_dir_GPIO_Port, M2_dir_Pin, 1);
			TIM4->CCR1 = pwm_M2;
		}
		else if (pwm_M2 < -10){
			HAL_GPIO_WritePin(M2_dir_GPIO_Port, M2_dir_Pin, 0);
			TIM4->CCR1 = -1 * pwm_M2;
		}
		else{
			HAL_GPIO_WritePin(M2_dir_GPIO_Port, M2_dir_Pin, 0);
			TIM4->CCR1 = 0;
		}




		if(reload_error > 0){	// M4

			HAL_GPIO_WritePin(M4_dir_GPIO_Port, M4_dir_Pin, 0);		// CW
			TIM4->CCR4 = 600;
		}
		else if(reload_error < 0){

			HAL_GPIO_WritePin(M4_dir_GPIO_Port, M4_dir_Pin, 1);		// CCW
			TIM4->CCR4 = 600;
		}
		else {

			HAL_GPIO_WritePin(M4_dir_GPIO_Port, M4_dir_Pin, 0);
			TIM4->CCR4 = 0;
		}

		if(right_error > 0){	// M1
			pwm_M1 = 400;
		}
		else if(right_error < 0){
			pwm_M1 = -400;
		}
		else {
			pwm_M1 = 0;
		}

		if(left_error > 0){	// M2
			pwm_M2 = -450;
		}
		else if(left_error < 0){
			pwm_M2 = 450;
		}
		else {
			pwm_M2 = 0;
		}
		if (right_error == 0 && left_error == 0){
			if(up_control == 2){
				right_control = up_control;
				left_control = up_control;
				up_control = 1;
			}

			if(up_control == 0){
				right_control = up_control;
				left_control = up_control;
			}
		}


		// reload
		reload_error = reload_control - reload_feedback;
		right_error = right_control - right_feedback;
		left_error = left_control - left_feedback;


		if(right_control == 2 && right_error == 0){
			right_control = 1;
		}
		if(left_control == 2 && left_error == 0){
			left_control = 1;
		}

		if(reload_feedback == 1 && reload_error == 0){
			reload_control = 0;
		}

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
