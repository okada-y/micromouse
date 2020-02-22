/*
 * mode.c
 *
 *  Created on: 2019/08/22
 *      Author: 岡田 泰裕
 */
#include "main.h"
#include "param.h"
#include "mouse_state.h"
#include "mode.h"
#include "ir_sensor.h"
#include "imu.h"

typedef enum{
	before, //モード選択中
	after	//モード実行準備完了
}MODE_DECIDE;

static uint8_t mode_count = 0;	  //mode選択用カウンタ
static uint8_t mode_number = 0;   //mode番号
static MODE_DECIDE ready_mode = before;
static MODE_STATE mode_state = preparation;

//機能 	:メインモード処理
//引数 	:なし
//返り値:なし
void mode_main(void)
{
	//初期処理
	ready_mode = before; 		//モード決定状態を決定前に
	mode_state = preparation;	//モード開始状態を開始前に
	mode_count = 0;				//modeカウンタ初期化
	
	Sensor_StopADC();	//IRセンサ停止
	
	while(ready_mode == before) //モード選択モードのとき
	{	
		mode_select();
		ready_mode = mode_decide_jud();
	}
	
	//モード開始可能状態に遷移したとき、2回LEDを点滅させる。
	for(int i=0; i<2; i++)
	{ 
		HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET);
		HAL_Delay(700);
		HAL_GPIO_WritePin(GPIOA,LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
	}

	//IRセンサを稼働させる。
	Sensor_Initialize( );	

	//モード開始を待機
	mode_start();
}


//機能 	:モード実行状態を返す
//引数 	:なし
//返り値	:モード実行状態
uint8_t get_mode_state(void)
{
	return mode_state;
}

//機能 	:モード実行状態をクリアする
//引数 	:なし
//返り値	:なし
void clr_mode_state(void)
{
	mode_state = preparation;
}

//機能 	:モード番号を返す
//引数 	:なし
//返り値:モード番号
uint8_t get_mode_number(void)
{
	return mode_number;
}

//機能 	:モード番号をセットする
//引数 	:新規のモード番号
//返り値	:なし
void set_mode_number(uint8_t mn)
{
	mode_number = mn;
}

//機能	:右タイヤの速度を入力に、モード番号を変更、LEDに出力
//引数	:なし
//返り値	:なし
void mode_select (void)
{
	static uint8_t mode_number_old = 0;

	while(get_tire_r_speed() != 0); //いったんタイヤ停止するまで待ち

	init_tire_speed(); //タイヤの最大、最低速度初期化

	HAL_Delay(200);	//タイヤ最大値更新時間

	if (get_tire_r_speed_max() > mode_count_up_th) //m右タイヤ速度＞正の閾値の時の処理
	{
		mode_count	+= 1;
	}

	if (get_tire_r_speed_min() < mode_count_down_th) //m右タイヤ速度＜負の閾値の時の処理
	{
		mode_count -= 1;
	}

	mode_count &= 0b00001111;	//8bit->4bit

	if (mode_count != mode_number_old){
		switch(mode_count){
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

	mode_number_old = mode_count;
	set_mode_number(mode_count);//モード番号にセット
}

//機能	:左タイヤ速度を入力とし、モードを決定する。
//引数	:なし
//返り値	:モード決定状態
uint8_t mode_decide_jud(void)
{
	uint8_t temp = 0;
	
	if(mode_stanby_th < get_tire_l_speed_max() )
	{
		temp = after;
	}
	else
	{
		temp = before;
	}
	return temp;
}

//機能	:IRセンサによるモード動作開始SW
//引数	:なし
//返り値	:なし
void mode_start(void)
{
	while(1)/*m右前センサをモード開始のスイッチとする。*/
	{
		if( Sensor_GetValue(3) >= 1500)
		{
		    IMU_ResetReference(); //IMUのリファレンス取得
			mode_state = process;//モードを実行中に遷移
			break;
		}
	}
}