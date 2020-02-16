/*
 * motor.c
 *
 *  Created on: Jul 14, 2019
 *      Author: 岡田 泰裕
 */

#include "motor.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "param.h"

#define PCLK			(HAL_RCC_GetPCLK2Freq())	//マイコンの動作周波数[Hz]
#define PWMFREQ			(100000)					//モータの動作周波数[Hz]
#define MOT_DUTY_MIN	(30)						//モータの最低Duty
#define MOT_DUTY_MAX	(150)						//モータの最大Duty

//モータの向き設定
#define MOT_SET_COMPARE_L_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, x)
#define MOT_SET_COMPARE_L_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, x)
#define MOT_SET_COMPARE_R_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, x)
#define MOT_SET_COMPARE_R_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, x)

static int16_t duty_r;
static int16_t duty_l;

/* ---------------------------------------------------------------
	モータ用のタイマーを開始する関数
--------------------------------------------------------------- */
void Motor_Initialize( void )
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/* ---------------------------------------------------------------
	モータのの回転を止める関数
--------------------------------------------------------------- */
void Motor_StopPWM( void )
{
	MOT_SET_COMPARE_L_FORWARD( 0xffff );
	MOT_SET_COMPARE_L_REVERSE( 0xffff );
	MOT_SET_COMPARE_R_FORWARD( 0xffff );
	MOT_SET_COMPARE_R_REVERSE( 0xffff );
}

/* ---------------------------------------------------------------
	左モータを指定のDuty（0～1000）で回転させる関数
--------------------------------------------------------------- */
void set_duty_l( int16_t duty_l_tmp )
{
	uint32_t	pulse_l;

	duty_l = duty_l_tmp;
	if( ABS(duty_l) > MOT_DUTY_MAX ) {
		pulse_l = (uint32_t)(PCLK / PWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_l) < MOT_DUTY_MIN ) {
		pulse_l = (uint32_t)(PCLK / PWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_l = (uint32_t)(PCLK / PWMFREQ * ABS(duty_l) / 1000) - 1;
	}

	if( duty_l > 0 ) {
		MOT_SET_COMPARE_L_FORWARD( pulse_l );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	} else if( duty_l < 0 ) {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( pulse_l );
	} else {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	}
}

/* ---------------------------------------------------------------
	右モータを指定のDuty（0～1000）で回転させる関数
--------------------------------------------------------------- */
void set_duty_r( int16_t duty_r_tmp )
{
	uint32_t	pulse_r;
	duty_r = duty_r_tmp;
	if( ABS(duty_r) > MOT_DUTY_MAX ) {
		pulse_r = (uint32_t)(PCLK / PWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_r) < MOT_DUTY_MIN ) {
		pulse_r = (uint32_t)(PCLK / PWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_r = (uint32_t)(PCLK / PWMFREQ * ABS(duty_r) / 1000) - 1;
	}

	if( duty_r > 0 ) {
		MOT_SET_COMPARE_R_FORWARD( pulse_r );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	} else if( duty_r < 0 ) {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( pulse_r );
	} else {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	}
}

//機能	: 右モータの変調率を取得する
//引数	: なし
//返り値	: 右モータの変調率
int16_t get_duty_r ( void )
{
	return duty_r;
}

//機能	: 左モータの変調率を取得する
//引数	: なし
//返り値	: 左モータの変調率
int16_t get_duty_l ( void )
{
	return duty_l;
}