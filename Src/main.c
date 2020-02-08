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
//#include "index.h"
#include "mode.h"
#include "ir_sensor.h"
#include "interrupt.h"
#include "imu.h"
#include "motor.h"
#include "encorder.h"
#include "target.h"
#include "movement.h"
#include "exvol.h"
#include "control.h"
#include "module_test.h"
#include "param.h"
#include "adjust.h"

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
  /*迷路変数定義*/
  static uint8_t maze_x_size = x_size;//x方向の壁の枚数(x方向のマスの数+1)
  static uint8_t maze_y_size = y_size;//y方向の壁の枚数(y方向のマスの数+1)
  static uint8_t goal_size = g_size; //ゴールのマスの数
  static uint8_t maze_goal[18] = goal_cordinate;
  static uint8_t m_wall_tmp[1024];//迷路情報格納用配列
  static uint8_t m_search_tmp[1024];//探索情報格納用配列
  static uint8_t run_mode = search_mode;

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
  Motor_Initialize(); 			/*モータ用タイマ設定*/
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
	  mode_main();//モードの選定、開始処理
	
	/*ここからモードごとの処理に移行*/
	  switch(get_mode_number()){

	    case 0://データ吐き出し用
  	  data_read();
	    break;

	  case 1:
       log_init();
	     set_mode_ctrl(trace);
       set_accel_mode(deceleration);
       set_target_length(0.09);
       HAL_Delay(3000);
		  break;

	  case 2:
      set_mode_ctrl(trace);
    	set_rotation_mode(counter_clockwise);
      set_accel_mode(deceleration);
      set_target_angle(PI/2);
      HAL_Delay(3000);
		  break;

	  case 3:

      fornt_wall_calibrate();
		  break;

	  case 4:
      HAL_Delay(5000);
		  break;

	  case 5:
      set_mode_ctrl(trace);
      set_accel_mode(deceleration);
      turn_conclk_180();
		  break;

	  case 6:
		  break;

	  case 7:
      module_test();
		  break;

	  /*最短走行*/
	  case 14:

		  break;

	  /*迷路探索*/
	  case 15:
		  /*m迷路データの初期化*/
		  maze_init(maze_y_size, maze_x_size, m_wall_tmp, m_search_tmp);

		  /*探索モードで走行*/
		  run_mode = search_mode;
		  maze_solve(m_wall_tmp, m_search_tmp, maze_y_size, maze_x_size, maze_goal, goal_size, run_mode);

		  break;
	  }
    clr_mode_state();
    Sensor_StopADC();

	  set_duty_r(0);	//motor_r 停止
	  set_duty_l(0);	//motor_l 停止

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
