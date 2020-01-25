/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "index.h"
/*from matlab*/
#include "maze_init.h"
#include "maze_solve.h"
#include "matlab_code_gen_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	search_mode 	= 0,
 	fust_run_mode 	= 1,
} maze_search_flg;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*迷路パラメータ設定*/
uint8_t maze_x_size = 5;//x方向の壁の枚数(x方向のマスの数+1)
uint8_t maze_y_size = 5;//y方向の壁の枚数(y方向のマスの数+1)
uint8_t goal_size = 1; //m ゴールのマスの数
uint8_t maze_goal[18] = {1, 0, 0, 0, 0, 0, 0, 0, 0,    	 //mゴールのx座標
		  	  	  	  	 4, 0, 0, 0, 0, 0, 0, 0, 0};	 //mゴールのy座標
uint8_t m_wall_tmp[1024];//迷路情報格納用配列
uint8_t m_search_tmp[1024];//探索情報格納用配列

uint8_t run_mode = search_mode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	  int16_t i = 0;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* m 各関数初期化　*/
  Motor_Initialize(); 			/*m モータ用タイマ設定*/
  Communication_Initialize( );	/*printf,scanf用の設定*/
  IMU_Initialize();				/*IMU初期設定*/
  Encoder_Initialize();			/*Encoderタイマ設定、位相初期化*/
  Interrupt_Initialize( );		/*interrupt処理用タイマ(TIM6)初期化*/
  //Battery_LimiterVoltage();		/*a バッテリー下限処理*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(stanby_mode == 0)//mスタンバイモードでないとき
	  {
		  Sensor_StopADC();
		  mode_count = 0;			//modeカウンタ初期化
		  while(stanby_mode == 0){	//stanby_modeに入るまでモード選択可能
		  mode_select();
//		  printf("mode_number =%d\r\n", mode_number);
		  stanby_mode = modechangejud_stanby();	//stanby_mode 移行判断
		  }
		  for(int i=0; i<2; i++){ //stanby移行時、LEDを2回点灯させ、IRセンサ起動
		  HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET);
		  HAL_Delay(700);
		  HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET);
		  HAL_Delay(300);
		  }

		  Sensor_Initialize( );	//irsensor計測開始

		  /*m右前センサをモード開始のスイッチとする。*/
		  while(1)
		  {
			  if( Sensor_GetValue(3) >= 1500)
				{
				  HAL_Delay(500);
					  break;
				}
		  }

		  mode_number_int = mode_number; //m 各モードナンバごとの割り込みフラグ
		  mode_number = 0;//m モードナンバの初期化
	  }

	  /*x ここからモードごとの処理に移行　 x*/
	  switch(mode_number_int){

	  case 0://m データ吐き出し用
//		  printf("mode0_start\r\n");
		  data_read();
	  break;

	  case 1:
//		  printf("mode1_start\r\n");
		  accel_dir_flg = 1; //m減速モード
		  fornt_wall_calibrate();
//		  HAL_Delay(5000);
//		  fornt_wall_calibrate();//壁補正
//		  turn_clk_90();
//		  fornt_wall_calibrate();
//		  turn_clk_90();
//		  HAL_Delay(1000);
		  break;

	  case 2:
//		  printf("mode2_start\r\n");
		  accel_dir_flg = 1; //m減速モード
		  move_dir_flg = 0;////m前進モード
		  real_distance_m_clr();
		  real_distance_w_clr();
		  log_init();
		  target_distance_m_set(0.45);
		  HAL_Delay(4000);
		  break;

	  case 3:
//		  printf("mode3_start\r\n");
		  accel_dir_flg = 1; //m減速モード
		  rotation_dir_flg = 0;////m反時計モード

		  real_distance_m_clr();
		  real_distance_w_clr();
		  log_init();
		  target_distance_w_set(2*PI);
		  HAL_Delay(20000);

//		  rotation_dir_flg = 1;//m時計モード
//		  target_distance_w_set(-PI/2);
//		  HAL_Delay(2000);

//		  target_distance_w_set(PI/2);
//		  HAL_Delay(500);
//		  target_distance_m_set(0.18);
//		  HAL_Delay(1000);
		  break;

	  case 4:
//		  Motor_SetDuty_Right(60);
//		  Motor_SetDuty_Left(-60);
		  HAL_Delay(500);
		  log_init();
//		  for(i=0; i<=100; i += 1){
////			  Motor_SetDuty_Right(-i);
//			  Motor_SetDuty_Left(-i);
//			  HAL_Delay(10);
//		  }
//		  HAL_Delay(200);
//		  for(i=100; i>=60; i -= 1){
//			  Motor_SetDuty_Right(i);
//			  Motor_SetDuty_Left(-i);
//			  HAL_Delay(10);
//		  }

		  Motor_SetDuty_Right(100);
		  Motor_SetDuty_Left(-100);

		  HAL_Delay(2000);
//		  printf("mode4_start\r\n");
		  break;

	  case 5:
//		  printf("mode5_start\r\n");
		  Motor_SetDuty_Right(60);
		  Motor_SetDuty_Left(60);
		  HAL_Delay(200);
		  log_init();
		  for(i=60; i<=200; i += 10){
			  Motor_SetDuty_Right(i);
			  Motor_SetDuty_Left(i);
			  HAL_Delay(10);
		  }
		  HAL_Delay(500);
		  for(i=200; i>=60; i -= 10){
			  Motor_SetDuty_Right(i);
			  Motor_SetDuty_Left(i);
			  HAL_Delay(10);
		  }
		  HAL_Delay(500);

		  break;

	  case 6:
//		  printf("mode6_start\r\n");
		  accel_dir_flg = 1; //m減速モード
		  correction_mode = 0;
		  HAL_Delay(10000);

		  break;

	  case 7:
//		  printf("mode7_start\r\n");
//		  correction_mode = 1;
		  accel_dir_flg = 1; //m減速モード
		  module_test( );
		  break;

	  /*m最短走行*/
	  case 14:

		  break;

	  /*m迷路探索*/
	  case 15:
		  /*m迷路データの初期化*/
		  maze_init(maze_y_size, maze_x_size, m_wall_tmp, m_search_tmp);

		  /*m探索モードで走行*/
		  run_mode = search_mode;
		  maze_solve(m_wall_tmp, m_search_tmp, maze_y_size, maze_x_size, maze_goal, goal_size, run_mode);

		  break;




	  }

	  Sensor_StopADC();			//m sennsor停止
	  stanby_mode = 0;			//m　モード選択処理に移行
	  mode_number_int = 0;		//m　モードごとの割り込みフラグクリア
	  Motor_SetDuty_Right(0);	//motor_r 停止
	  Motor_SetDuty_Left(0);	//motor_l 停止

	  for(int i=0; i<3; i++){ //m モード処理終了時、LEDを3回点灯
	  HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET);
	  HAL_Delay(700);
	  HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);
	  }

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
