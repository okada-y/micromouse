/*
 * encorder.c
 *
 *  Created on: Jul 14, 2019
 *      Author: 岡田 泰裕
 */



#include "index.h"

#define ENC_CNT_L		(TIM3 -> CNT)
#define ENC_CNT_R		(TIM2 -> CNT)
#define ENC_CNT_SR_L	(TIM3 -> SR)	//TIM3のstatus register
#define ENC_CNT_SR_R	(TIM2 -> SR) 	//TIM2のstatus register


#define ENC_ZERO		(32767 - 1)		/* encoderの初期値 */
#define ENC_RESOLUTION 	(1024 - 1)		/* encoderの分解能 */



static uint16_t enc_cnt_r_new = 0;
static uint16_t enc_cnt_l_new = 0;
static uint16_t enc_cnt_r_old = 0;
static uint16_t enc_cnt_l_old = 0;
float speed_r = 0;//m 右タイヤ速度[ m/s]
float speed_r_max = 0;// m最大右タイヤ速度（デバッグ用
float speed_r_min = 0;// m最小右タイヤ速度（デバッグ用
float speed_l = 0;//m 左タイヤ速度[ m/s]
float speed_l_max = 0;// m最大左タイヤ速度（デバッグ用
float speed_l_min = 0;// m最小左タイヤ速度（デバッグ用
float speed_m = 0;//m 並進方向速度[ m/s]
float g_ave_speed_m = 0;//m 並進方向速度を移動平均処理[m/s]
float g_ave_accel_m = 0;//m 並進方向加速度を移動平均処理[m/s2]
float speed_rad = 0;//m エンコーダから算出される角速度[rad/s]

float delta_angle_motor_r = 0;	//m 右モータの回転数[rad/s]
float delta_angle_motor_l = 0;	//m 左モータの回転数[rad/s]

typedef struct {	//a 移動平均用構造体の定義
	float	speed_m;			// m　測定開始からの時間(m/s)
	float   accel_m;			// m 目標速度(m/s)
} ave_struct;

static ave_struct ave_store[m_ave_num];  // データ格納用の構造体





/* ---------------------------------------------------------------
	位相係数モードのタイマーを開始し、位相カウントを初期化する関数
--------------------------------------------------------------- */
void Encoder_Initialize( void )
{
	HAL_TIM_Encoder_Start( &htim2, TIM_CHANNEL_ALL );
	HAL_TIM_Encoder_Start( &htim3, TIM_CHANNEL_ALL );
	Encoder_ResetCount_Left();
	Encoder_ResetCount_Right();
	enc_cnt_r_old	=	ENC_CNT_R;
	enc_cnt_l_old	=	ENC_CNT_L;
}

/* ---------------------------------------------------------------
	左タイヤの位相係数カウントを初期化する関数
--------------------------------------------------------------- */
void Encoder_ResetCount_Left( void )
{
	ENC_CNT_L = ENC_ZERO;
}

/* ---------------------------------------------------------------
	右タイヤの位相係数カウントを初期化する関数
--------------------------------------------------------------- */
void Encoder_ResetCount_Right( void )
{
	ENC_CNT_R = ENC_ZERO;
}

/* ---------------------------------------------------------------
	左タイヤの角度を取得する関数[rad]
--------------------------------------------------------------- */
float Encoder_GetAngle_Left( void )
{
	return(2 * PI * (float)( (int32_t)ENC_CNT_L - (int32_t)ENC_ZERO ) / (float)ENC_RESOLUTION);
}

/* ---------------------------------------------------------------
	右タイヤの角度を取得する関数[rad]
--------------------------------------------------------------- */
float Encoder_GetAngle_Right( void )
{
	return(2 * PI * (float)( (int32_t)ENC_ZERO - (int32_t)ENC_CNT_R ) / (float)ENC_RESOLUTION);
}


/* ---------------------------------------------------------------
 	 速度を算出する関数(1msタスク)[m/s]
 -------------------------------------------------------------- */
void Get_speed(void)
{
	uint16_t	uss_delta_cnt_r;	//m　右タイヤのカウント[/ms](計算用)
	uint16_t	uss_delta_cnt_l;	//m　左タイヤのカウント[/ms](計算用)
	int32_t		sl_delta_cnt_r;	//m　右タイヤのカウント [/ms]
	int32_t		sl_delta_cnt_l;	//m　左タイヤのカウント [/ms]
	float 		delta_angle_tire_r;	//m　右タイヤの角速度	[rad/s]
	float		delta_angle_tire_l;	//m 左タイヤの角速度	[rad/s]

	enc_cnt_r_new = ENC_CNT_R;
	enc_cnt_l_new = ENC_CNT_L;

	if((ENC_CNT_SR_R & 0x0001) == 0x0000)//overflowがないとき(UIF=0)
	{
		sl_delta_cnt_r = (int32_t)enc_cnt_r_new - (int32_t)enc_cnt_r_old;	//m　右タイヤのカウント値を取得(/1ms)
	}
	else//overflow　or underflowがあるとき(UIF=1)
	{
		if(enc_cnt_r_new >= enc_cnt_r_old)//m 逆回転時
		{
			uss_delta_cnt_r = enc_cnt_r_old	- enc_cnt_r_new;
			sl_delta_cnt_r = -1 * (int32_t)uss_delta_cnt_r;
		}
		else	//m 正回転時
		{
			uss_delta_cnt_r = enc_cnt_r_new	- enc_cnt_r_old;
			sl_delta_cnt_r = 1 * (int32_t)uss_delta_cnt_r;
		}

		ENC_CNT_SR_R = ENC_CNT_SR_R & ~(0x0001); //UIFクリア

	}


	if((ENC_CNT_SR_L & 0x0001) == 0x0000)//overflowがないとき(UIF=0)
	{
		sl_delta_cnt_l = (int32_t)enc_cnt_l_new - (int32_t)enc_cnt_l_old;	//m　右タイヤのカウント値を取得(/1ms)
	}
	else//overflow　or underflowがあるとき(UIF=1)
	{
		if(enc_cnt_l_new >= enc_cnt_l_old)//m 逆回転時
		{
			uss_delta_cnt_l = enc_cnt_l_old	- enc_cnt_l_new;
			sl_delta_cnt_l = -1 * (int32_t)uss_delta_cnt_l;
		}
		else	//m 正回転時
		{
			uss_delta_cnt_l = enc_cnt_l_new	- enc_cnt_l_old;
			sl_delta_cnt_l = 1 * (int32_t)uss_delta_cnt_l;
		}

		ENC_CNT_SR_L = ENC_CNT_SR_L & ~(0x0001); //UIFクリア

	}



	/*m 各タイヤの角速度算出 */
	delta_angle_tire_r = -1 * 1000 * 2 * PI * (float)sl_delta_cnt_r / (float)ENC_RESOLUTION;//m 右タイヤ角速度 [rad/s]
	delta_angle_tire_l =  1 * 1000 * 2 * PI * (float)sl_delta_cnt_l / (float)ENC_RESOLUTION;//m 左タイヤ角速度 [rad/s]

	/*m 各モータの角速度算出*/
	delta_angle_motor_r = gear_rate * delta_angle_tire_r;
	delta_angle_motor_l = gear_rate * delta_angle_tire_l;

	/*m 各タイヤの角速度から各タイヤの速度算出　*/
	speed_r = Tire_diameter * 0.5 * delta_angle_tire_r;	//m 右タイヤ速度[ m/s]
	speed_l = Tire_diameter * 0.5 * delta_angle_tire_l;	//m 左タイヤ速度[ m/s]



	if(speed_r > speed_r_max)
	{
		speed_r_max = speed_r;
	}
	else if(speed_r < speed_r_min)
	{
		speed_r_min = speed_r;
	}

	if(speed_l > speed_l_max)
	{
		speed_l_max = speed_l;
	}
	else if(speed_l < speed_l_min)
	{
		speed_l_min = speed_l;
	}

	speed_m = (speed_r + speed_l)/2;					//m 本体速度（右と左の平均値）  [m/s]
	speed_rad = (-speed_l + speed_r) / chassis_width;	//m エンコーダ値から算出される角速度[rad/s]


	enc_cnt_r_old	=	enc_cnt_r_new;	//m 右タイヤのカウント値を更新
	enc_cnt_l_old	=	enc_cnt_l_new;	//m 左タイヤのカウント値を更新

}

/* ---------------------------------------------------------------
	並進方向の速度に移動平均をかけ、遅れを補完する関数(1msタスク)
--------------------------------------------------------------- */
void speed_m_average( void )
{
	static uint16_t l_ave_counter = 0;
	uint16_t i = 0;
	uint16_t j = 0;

	float l_ave_speed_m = 0;	//m 並進方向速度平均
	float l_ave_accel_m = 0;	//m 並進方向加速度

	float l_sum_speed_m = 0; 	//m 並進方向速度合計
	float l_sum_accel_m = 0;	//m 並進方向加速度合計

	i = l_ave_counter % m_ave_num;

	ave_store[i].speed_m = speed_m;
	ave_store[i].accel_m = IMU_GetAccel_X();

	for(j=0; j < m_ave_num; j++)
	{
		l_sum_speed_m += ave_store[j].speed_m;
		l_sum_accel_m += ave_store[j].accel_m;
	}

	l_ave_speed_m = l_sum_speed_m / m_ave_num;
	l_ave_accel_m = l_sum_accel_m / m_ave_num;

	/*加速度センサのぶれを補正、閾値範囲内を０とする*/
	if(ABS(l_ave_accel_m) < 0.5)
	{
		l_ave_accel_m = 0;
	}

	/*m 遅れ補正*/
	l_ave_speed_m += 0.5 * ( l_ave_accel_m * (m_ave_num * 0.001)); //m 平均速度＋0.5(　加速度平均[m/s2]　・　平均時間[s])


	g_ave_speed_m = l_ave_speed_m;
	g_ave_accel_m = l_ave_accel_m;


	l_ave_counter += 1 ;
}

