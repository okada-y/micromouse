/*
 * interrupt.c
 *
 *  Created on: Aug 4, 2019
 *      Author: 岡田 泰裕
 */


#include "index.h"

#define TIMER_COUNT		(__HAL_TIM_GET_COUNTER(&htim6))
#define TIMER_LOAD		(__HAL_TIM_GET_AUTORELOAD(&htim6))
#define TIMER_PSC		((&htim6)->Instance->PSC)

 static uint32_t		interrupt_count_now;
//static uint16_t		interrupt_count_now;
static uint32_t		interrupt_duty;
static uint32_t		interrupt_duty_max = 0;
static float		boot_time = 0.f;


/* ---------------------------------------------------------------
	1ms周期で割り込み処理関数
--------------------------------------------------------------- */
void Interrupt_Main( void )
{

	Get_speed();
	speed_m_average();

	switch(mode_number_int){
	case 0 ://m 初期起動時にとおるので、つかわないこと。(ログ出力用）
		break;

	case 1 :
		data_get();
		calibrate_tim(); 			//m前壁補正用カウンタ
		real_distance_m_calc();		//m現在速度を積分して距離算出
		real_distance_w_calc();		//m現在角速度を積分して角度算出
		target_speed_m_calc();		//m現在距離と目標距離に応じて加速度を切り替え
		target_speed_w_calc();		//m現在角度と目標角度に応じて角加速度を切り替え
		target_speed_inc();			//m加速度、角加速度を目標速度に加算
		Operation_amount_calc();
		break;
	case 2 :
		data_get();
		real_distance_m_calc();		//m現在速度を積分して距離算出
		real_distance_w_calc();		//m現在角速度を積分して角度算出
		target_speed_m_calc();		//m現在距離と目標距離に応じて加速度を切り替え
		target_speed_w_calc();		//m現在角度と目標角度に応じて角加速度を切り替え
		target_speed_inc();			//m加速度、角加速度を目標速度に加算
		Operation_amount_calc();	//m目標速度と現在速度の偏差に応じて各モータの電圧演算
		break;
	case 3 :
		data_get();
		real_distance_m_calc();		//m現在速度を積分して距離算出
		real_distance_w_calc();		//m現在角速度を積分して角度算出
		target_speed_m_calc();		//m現在距離と目標距離に応じて加速度を切り替え
		target_speed_w_calc();		//m現在角度と目標角度に応じて角加速度を切り替え
		target_speed_inc();			//m加速度、角加速度を目標速度に加算
		Operation_amount_calc();	//m目標速度と現在速度の偏差に応じて各モータの電圧演算
		break;
	case 4 :
		data_get();
		break;
	case 5 :
		data_get();
		break;
	case 6 :
		data_get();
		real_distance_m_calc();		//m現在速度を積分して距離算出
		real_distance_w_calc();		//m現在角速度を積分して角度算出
		target_speed_m_calc();		//m現在距離と目標距離に応じて加速度を切り替え
		target_speed_w_calc();		//m現在角度と目標角度に応じて角加速度を切り替え
		target_speed_inc();			//m加速度、角加速度を目標速度に加算
		Operation_amount_calc();	//m目標速度と現在速度の偏差に応じて各モータの電圧演算
		break;
	case 7 :
		data_get();
		calibrate_tim(); //m前壁補正用カウンタ
		real_distance_m_calc();		//m現在速度を積分して距離算出
		real_distance_w_calc();		//m現在角速度を積分して角度算出
		target_speed_m_calc();		//m現在距離と目標距離に応じて加速度を切り替え
		target_speed_w_calc();		//m現在角度と目標角度に応じて角加速度を切り替え
		target_speed_inc();			//m加速度、角加速度を目標速度に加算
		Operation_amount_calc();
		break;
	case 15 :
		data_get();
		calibrate_tim(); 			//m前壁補正用カウンタ
		real_distance_m_calc();		//m現在速度を積分して距離算出
		real_distance_w_calc();		//m現在角速度を積分して角度算出
		target_speed_m_calc();		//m現在距離と目標距離に応じて加速度を切り替え
		target_speed_w_calc();		//m現在角度と目標角度に応じて角加速度を切り替え
		target_speed_inc();			//m加速度、角加速度を目標速度に加算
		Operation_amount_calc();	//m目標速度と現在速度の偏差に応じて各モータの電圧演算
		break;
	}
}

/* ---------------------------------------------------------------
	メイン割り込みの初期設定関数
--------------------------------------------------------------- */
void Interrupt_Initialize( void )
{
	HAL_TIM_Base_Start_IT( &htim6 );
}

/* ---------------------------------------------------------------
	割り込み前処理関数
--------------------------------------------------------------- */
void Interrupt_PreProcess( void )
{
//	static uint32_t		interrupt_count_old = 0;
//	static uint16_t		interrupt_count_old = 0;

	static uint64_t		boot_time_count = 0;

	interrupt_count_now = TIMER_COUNT;
//	boot_time_count += (uint16_t)(interrupt_count_now - interrupt_count_old);
//	interrupt_count_old = interrupt_count_now;
//	boot_time = (float)boot_time_count / (float)(HAL_RCC_GetPCLK1Freq() *2 / ( TIMER_PSC + 1) );

	boot_time_count += 1;
	boot_time = ((float)boot_time_count /(float)1000);
}

/* ---------------------------------------------------------------
	割り込み後処理関数
--------------------------------------------------------------- */
void Interrupt_PostProcess( void )
{
	interrupt_duty = (TIMER_COUNT - interrupt_count_now) * 1000 / TIMER_LOAD;
	interrupt_duty_max = MAX( interrupt_duty_max, interrupt_duty );
}

/* ---------------------------------------------------------------
	割り込み周期に占める呼び出し位置までの処理時間の割合を取得する関数
--------------------------------------------------------------- */
uint16_t Interrupt_GetDuty( void )
{
	return interrupt_duty;
}

/* ---------------------------------------------------------------
	上記割合の最大値を取得する関数
--------------------------------------------------------------- */
uint16_t Interrupt_GetDuty_Max( void )
{
	return interrupt_duty_max;
}

/* ---------------------------------------------------------------
	マイコン起動時からの経過時間を取得する関数
--------------------------------------------------------------- */
float Interrupt_GetBootTime( void )
{
	return boot_time;
}
