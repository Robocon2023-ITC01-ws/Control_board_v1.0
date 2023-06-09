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

uint8_t input_degree = 150; // 90

float speed_up;
uint8_t up_control = 1;
uint8_t shoot_state;
uint8_t shoot_finish = 1;

uint8_t start_finish = 1;
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

uint8_t right_feedback = 1;
uint8_t right_control = 1;
int8_t right_error;

uint8_t left_feedback = 1;
uint8_t left_control = 1;
int8_t left_error;

uint8_t shoot_feedback = 0;
uint8_t shoot_control = 0;
int8_t shoot_error = 0;


uint8_t bit1;
uint8_t bit2;
uint8_t bit3;
uint8_t bit4;

uint8_t pickup_state;
uint8_t ring_count;
uint8_t start;
uint8_t prepare;
uint8_t reload;
uint8_t shoot;

uint8_t right_reset;
uint8_t left_reset;

uint8_t test = 150;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[4];
uint32_t TxMailbox;
uint8_t cntt;

uint8_t break_loop;

float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output)
{

	return (float)((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void servo_rotation(uint8_t degree){
	float y = 0.556 * (float) degree + 25;
	uint8_t pwm = round(y);
	TIM3->CCR2 = pwm;
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

	if (RxHeader.DLC == 2 && RxHeader.StdId == 0x222)
	{
		data_16 = (RxData[0] << 8) | RxData[1];
		speed = map(data_16,0,65535,0,1500);
	}

	if (RxHeader.DLC == 1 && RxHeader.StdId == 0x444){
		reload_feedback = RxData[0] >> 0 & 1;
		bit1 = RxData[0] >> 1 & 1;
		bit2 = RxData[0] >> 2 & 1;
		right_feedback = (bit2 << 1 ) + bit1;
		bit3 = RxData[0] >> 3 & 1;
		bit4 = RxData[0] >> 4 & 1;
		left_feedback = (bit4 << 1 ) + bit3;

		shoot_feedback = RxData[0] >> 5 & 1;

		right_reset = RxData[0] >> 6 & 1;
		left_reset = RxData[0] >> 7 & 1;

		if (right_reset == 1 || left_reset == 1){
			right_reset = 0;
			left_reset = 0;
			// break signal

			break_loop = 1;
			servo_rotation(150);
			right_control = 1;
			left_control = 1;
		}
	}

	// working_state
	if (RxHeader.DLC == 1 && RxHeader.StdId == 0x145){
		if(RxData[0] == 0){
			if(right_error == 0 && left_error == 0 && left_feedback != 0 && right_feedback != 0) {
				prepare = 1;
			}
		}
		else if(RxData[0] == 1 && start_finish == 1){
			if(right_error == 0 && left_error == 0){
				start = 1;
			}
		}
		else if(RxData[0] == 2 && shoot_finish == 1){
			if(shoot_state == 0 ){ // && ring_count > 0 && ring_count <= 10
				shoot = 1;
			}
		}
	}
}



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// process on pick up //


void prepare_function(){
	ring_count = 0;
	servo_rotation(160);
	right_control = 0;
	left_control = 0;
	HAL_Delay(10);
	while (left_error != 0 || right_error != 0 )
	{
		/* code */
	}
}

//void reload_function(){
//
//	if(break_loop){
//		break_loop = 0;
//		right_feedback = 1;
//		left_feedback = 1;
//	}
//	else {
//		reload_control = 1;
//		HAL_Delay(50);
//		while(reload_control != 0){
//			right_control = 0;
//			left_control = 0;
//		}
//
//		while(reload_error != 0){
//			right_feedback = 0;
//			left_feedback = 0;
//			right_error = 0;
//			left_error = 0;
//		}
//		right_feedback = 0;
//		left_feedback = 0;
//		HAL_Delay(200);
//		right_control = 3;
//		left_control = 3;
//		HAL_Delay(10);
//		if(break_loop){
//			right_control = 1;
//			left_control = 1;
//			servo_rotation(160);
//		}
//		else {
//			while ((left_error != 0 || right_error != 0))
//			{
//				if (break_loop){
//					right_control = 1;
//					left_control = 1;
//					break;
//				}
//				/* code */
//			}
//			if (break_loop == 0){
//				right_control = 2;
//				left_control = 2;
//				HAL_Delay(10);
//			}
//
//			while ((left_error != 0 || right_error != 0 ))
//			{
//				/* code */
//				if (break_loop){
//					right_control = 1;
//					left_control = 1;
//					break;
//				}
//			}
//		}
//	}
//
//}
//


void reload_function(){

	if(break_loop){
		break_loop = 0;
		right_feedback = 1;
		left_feedback = 1;
	}
	else {
		reload_control = 1;
		HAL_Delay(300);
		while(reload_control != 0){
			right_control = 0;
			left_control = 0;
		}
//		right_control = 0;
//		left_control = 0;
//		HAL_Delay(10);
//		reload_control = 1;
//		while(reload_control != 0);


		while(reload_error != 0){
			right_feedback = 0;
			left_feedback = 0;
			right_error = 0;
			left_error = 0;
		}
		right_feedback = 0;
		left_feedback = 0;
		while(reload_feedback != 0){
			HAL_Delay(10);
		}
		right_control = 3;
		left_control = 3;
		HAL_Delay(10);
		if(break_loop){
			right_control = 1;
			left_control = 1;
			servo_rotation(160);
		}
		else {
			while ((left_error != 0 || right_error != 0))
			{
				if (break_loop){
					right_control = 1;
					left_control = 1;
					break;
				}
				/* code */
			}
			if (break_loop == 0){
				right_control = 2;
				left_control = 2;
				HAL_Delay(10);
			}

			while ((left_error != 0 || right_error != 0 ))
			{
				/* code */
				if (break_loop){
					right_control = 1;
					left_control = 1;
					break;
				}
			}
		}
	}

}


void start_fucntion(){
	start_finish = 0;
	servo_rotation(95);
	HAL_Delay(500);
	right_control = 2;
	left_control = 2;

	HAL_Delay(10);
	while ((left_error != 0 || right_error != 0 ) ){
		if (break_loop){
			right_control = 1;
			left_control = 1;
			break;
		}
	}

	HAL_Delay(10);
	reload_function();

	start_finish = 1;
}

void shoot_function(float speed_){
	shoot_finish = 0;
	speed = speed_;
	HAL_Delay(1000);
	shoot_state = 1;
	HAL_Delay(700);
	shoot_state = 0;
	speed = 0;

	HAL_Delay(10);
	while(shoot_error != 0);
	if (right_feedback != 1 && left_feedback != 1){
		reload_function();
	}
	shoot_finish = 1;
}
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

	servo_rotation(160);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GetTick() < 2000){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		  HAL_Delay(100);
	  }

	  if (prepare == 1){
		  prepare = 0;
		  prepare_function();
	  }
	  if(start == 1){
		  start = 0;
		  start_fucntion();
	  }
	  if(reload == 1){
		  reload = 0;
		  reload_function();
	  }
	  if(shoot == 1 && speed > 0){
		  shoot = 0;
		  shoot_function(speed);
	  }

	  if(break_loop == 1 &&(right_feedback == 1 && left_feedback == 1)){
		  break_loop = 0;
	  }

//	  servo_rotation(test);

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

//
		if (pwm_M1 > 10){
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 0);
			TIM4->CCR2 = pwm_M1;
		}
		else if (pwm_M1 < -10){
			HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, 1);
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


		// reload
		reload_error = reload_control - reload_feedback;
		shoot_error = shoot_control - shoot_feedback;

		if(shoot_state == 1){		// load for shoot // M3
			shoot_control = 1;
		}
		else {
			shoot_control = 0;
		}

		if (shoot_error > 0){
			HAL_GPIO_WritePin(M3_dir_GPIO_Port, M3_dir_Pin, 1);
			TIM4->CCR3 = 600;
		}
		else if(shoot_error < 0){
			HAL_GPIO_WritePin(M3_dir_GPIO_Port, M3_dir_Pin, 0);
			TIM4->CCR3 = 600;
		}
		else {
			HAL_GPIO_WritePin(M3_dir_GPIO_Port, M3_dir_Pin, 0);
			TIM4->CCR3 = 0;
		}

		if(reload_error > 0){	// M4

			HAL_GPIO_WritePin(M4_dir_GPIO_Port, M4_dir_Pin, 0);		// CW
			TIM4->CCR4 = 1000;
		}
		else if(reload_error < 0){

			HAL_GPIO_WritePin(M4_dir_GPIO_Port, M4_dir_Pin, 1);		// CCW
			TIM4->CCR4 = 800;
		}
		else {
			HAL_GPIO_WritePin(M4_dir_GPIO_Port, M4_dir_Pin, 0);
			TIM4->CCR4 = 0;
		}


		//////////////////////////////////////////////////

		if(right_error > 0){	// M1
			pwm_M1 = -520;
		}
		else if(right_error < 0){
			pwm_M1 = 520;
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
		
		right_error = right_control - right_feedback;
		left_error = left_control - left_feedback;

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
