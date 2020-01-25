/*
 * mode.c
 *
 *  Created on: 2019/08/22
 *      Author: 岡田 泰裕
 */
#include "index.h"

uint8_t mode_number = 0; //mode番号
uint8_t stanby_mode = 0; //1=stanby 0:not_stanby
uint8_t mode_count = 0;  //modeカウンタ

void mode_select (void)
{


	static uint8_t mode_number_old = 0;

	while(speed_r != 0); //m いったんタイヤ停止するまで待ち

	speed_r_max = 0;//m右タイヤ速度max初期化
	speed_r_min = 0;//m右タイヤ速度	min初期化
	speed_l_max = 0;//m左タイヤ速度max初期化
	speed_l_min = 0;//m左タイヤ速度	min初期化

	HAL_Delay(200);	//m タイヤ最大値更新時間

//	 printf("speed_r_max =%5.3f,speed_r_min =%5.3f \r\n",speed_r_max, speed_r_min);
	if (speed_r_max > mode_count_up_th) //m右タイヤ速度＞正の閾値の時の処理
	{
		mode_count	+= 1;
	}

	if (speed_r_min < mode_count_down_th) //m右タイヤ速度＜負の閾値の時の処理
	{
		mode_count -= 1;
	}

	mode_count &= 0b00001111;	//8bit->4bit
	mode_number = mode_count;

	if (mode_number != mode_number_old){
		switch(mode_number){
		/*mode_numberに応じてLED点灯処理*/
			case 0:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET);//m 無点灯
			break;

			case 1:
				HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED2_Pin, GPIO_PIN_SET);
			break;

			case 2:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED3_Pin, GPIO_PIN_SET);
			break;

			case 3:
				HAL_GPIO_WritePin(GPIOA,LED4_Pin|LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin, GPIO_PIN_SET);
			break;

			case 4:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED4_Pin, GPIO_PIN_SET);
			break;

			case 5:
				HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED4_Pin, GPIO_PIN_SET);
			break;

			case 6:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED4_Pin, GPIO_PIN_SET);
			break;

			case 7:
				HAL_GPIO_WritePin(GPIOA,LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_SET);
			break;

			case 8:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED5_Pin, GPIO_PIN_SET);
			break;

			case 9:
				HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED5_Pin, GPIO_PIN_SET);
			break;

			case 10:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED5_Pin, GPIO_PIN_SET);
			break;

			case 11:
				HAL_GPIO_WritePin(GPIOA,LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED5_Pin, GPIO_PIN_SET);
			break;

			case 12:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED4_Pin|LED5_Pin, GPIO_PIN_SET);
			break;

			case 13:
				HAL_GPIO_WritePin(GPIOA,LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET);
			break;

			case 14:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET);
			break;

			case 15:
				HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET);
			break;
			}
		}

	mode_number_old = mode_number;

}

uint8_t modechangejud_stanby(void) //stanbyモード移行判定関数
{
	if(mode_stanby_th < speed_l_max )
	{
//		printf ("ready \r\n");
		return 1;
	}
	else {
		return 0;
	}
}

