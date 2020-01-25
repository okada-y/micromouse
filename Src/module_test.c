/*
 * module_test.c
 *
 *  Created on: Aug 13, 2019
 *      Author: 岡田 泰裕
 */


#include "index.h"


uint16_t log_counter = 0 ; //m ログ取得開始からの時間監視用カウンタ[ms]

typedef struct {	//a データ格納用構造体の定義（最大２０個とすること（４秒の時））
	float	time;				// m測定開始からの時間[m/s]
	float	target_d_m;			// m目標移動距離[m]
	float 	ideal_d_m;			// m理想の並進方向移動距離[m]
	float	real_d_m;			// m並進方向移動距離[m]
	float   target_speed_m;		// m目標速度[m/s]
	float	speed_m;			// m並進方向の速度[m/s]
	float	speed_m_ave;		// m並進方向の速度平均[m/s]
	float   accel_m;			// m並進方向の加速度(m/s)
	float	accel_m_ave;		// m並進方向の加速度平均(m/s)
	float	target_d_w;			// m目標角度
	float	ideal_d_w;			// m理想角度
	float	real_d_w;			// m回転角度(rad)
	float   target_speed_w;		// m目標角速度(rad/s)
	float	speed_w;			// m実際の角速度(rad/s)
	float   duty_r;				// m右モータ操作量（duty)
	float   duty_l;				// m左モータ操作量(duty)
	float   front_sensor_r;		// m右前壁センサ値
	float   front_sensor_l;		// m左前壁センサ値
	float   side_sensor_r;		// m右横壁センサ値
	float   side_sensor_l;		// m左横壁センサ値
	float   front_r;			// m壁との距離（右前センサ）(m)
	float   front_l;			// m壁との距離（左前センサ）(m)）
	float 	V_battery;			// mバッテリー電圧
} log_struct;

static log_struct log_store[log_count_lim / log_count_step];  // データ格納用の構造体


/* ---------------------------------------------------------------
	ログ取得カウンタ初期化関数
--------------------------------------------------------------- */
void log_init (void)
{
	log_counter = 0; //m ログカウンタの初期化
}

/* ---------------------------------------------------------------
	ログ取得用関数（1msタスク）
--------------------------------------------------------------- */
void data_get (void)
{

	uint16_t i = 0;
	i = log_counter / log_count_step;

	if(log_counter < log_count_lim)
	{
		if( (log_counter % log_count_step) == 0)
		{
			log_store[i].time = (float)log_counter;
			log_store[i].target_d_m = (float)target_distance_m;
			log_store[i].ideal_d_m = (float)ideal_distance_m;
			log_store[i].real_d_m = (float)real_distance_m;
			log_store[i].target_speed_m =  (float)target_speed_m;
			log_store[i].speed_m = (float)speed_m;
			log_store[i].speed_m_ave = (float)g_ave_speed_m;
			log_store[i].accel_m = (float)IMU_GetAccel_X();
			log_store[i].accel_m_ave = (float)g_ave_accel_m;
			log_store[i].target_d_w = (float)target_distance_w;
			log_store[i].ideal_d_w = (float)ideal_distance_w;
			log_store[i].real_d_w = (float)real_distance_w;
			log_store[i].target_speed_w = (float)target_speed_w;
			log_store[i].speed_w = (float)IMU_GetGyro_Z();
			log_store[i].duty_r = (float)g_duty_r;
			log_store[i].duty_l = (float)g_duty_l;
			log_store[i].front_sensor_r = (float)Sensor_GetValue(3);
			log_store[i].front_sensor_l = (float)Sensor_GetValue(0);
			log_store[i].side_sensor_r = (float)Sensor_GetValue(2);
			log_store[i].side_sensor_l = (float)Sensor_GetValue(1);
			log_store[i].front_r = (float)SensorValue2length(3);
			log_store[i].front_l = (float)SensorValue2length(0);
			log_store[i].V_battery =(float) Battery_GetVoltage();
		}
		log_counter += 1; //logカウンタ更新
	}
}

/* ---------------------------------------------------------------
	ログ吐き出し用関数
--------------------------------------------------------------- */
void data_read(void)
{

	uint16_t i = 0;
	uint16_t j = 0;

	j =  log_count_lim / log_count_step - 1 ;

	//printfで一行づつ書き出していく。
	//m 一行目はパラメータ名

//	printf ("TIME[ms],target_distance[m],ideal_distance[m],current_distance[m],TARGET_SPEED_m[m/s],SPEED_m[m/s],SPEED_m_ave[m/s],accel_m[m/s2],accel_m_ave[m/s2],current_angle[rad],DUTY_R[%%],Duty_L[%%],V_battery[V]\r\n");	//mパラメータ名を記述
	printf ("TIME[ms],target_distance[m],ideal_distance[m],current_distance[m],TARGET_SPEED_m[m/s],SPEED_m[m/s],SPEED_m_ave[m/s],"
			"accel_m[m/s2],accel_m_ave[m/s2],target_angle[rad],ideal_angle[rad],current_angle[rad],target_speed_w[rad/s],"
			"speed_w[rad/s],DUTY_R[%%],Duty_L[%%],front_r,front_l,side_r,side_l,front_r[m],front_l[m],V_battery[V]\r\n");	//mパラメータ名を記述

	for(i = 0; i <= j ; i++)
	{
		printf("%f,",log_store[i].time);
		printf("%f,",log_store[i].target_d_m);
		printf("%f,",log_store[i].ideal_d_m);
		printf("%f,",log_store[i].real_d_m);
		printf("%f,",log_store[i].target_speed_m);
		printf("%f,",log_store[i].speed_m);
		printf("%f,",log_store[i].speed_m_ave);
		printf("%f,",log_store[i].accel_m);
		printf("%f,",log_store[i].accel_m_ave);
		printf("%f,",log_store[i].target_d_w);
		printf("%f,",log_store[i].ideal_d_w);
		printf("%f,",log_store[i].real_d_w);
		printf("%f,",log_store[i].target_speed_w);
		printf("%f,",log_store[i].speed_w);
		printf("%f,",log_store[i].duty_r);
		printf("%f,",log_store[i].duty_l);
		printf("%f,",log_store[i].front_sensor_r);
		printf("%f,",log_store[i].front_sensor_l);
		printf("%f,",log_store[i].side_sensor_r);
		printf("%f,",log_store[i].side_sensor_l);
		printf("%f,",log_store[i].front_r);
		printf("%f,",log_store[i].front_l);
		printf("%f",log_store[i].V_battery);			//m 最後はカンマなし
		printf("\r\n"); //m 改行

	}

}



/* ---------------------------------------------------------------
	各機能の動作確認用関数
--------------------------------------------------------------- */
void module_test( void )
{
	uint16_t	line 	  = 0;
	int16_t		duty_l	  = 0;
	int16_t		duty_r	  = 0;

	// aエンコーダのカウントをリセット
	Encoder_ResetCount_Left();
	Encoder_ResetCount_Right();

	while( 1 ) {
//		__disable_irq();
		// a割り込み処理率を表示
		printf("<Boot Time> %8.3f[s]\r\n", Interrupt_GetBootTime()); line++;
		printf("<Interrupt> %3.1f[%%] (MAX : %3.1f[%%])\r\n",
				(float)Interrupt_GetDuty()/10.f, (float)Interrupt_GetDuty_Max()/10.f); line++;

		// aモータを指定のDutyを表示
		printf("<PWM Duty> L: %4.1f[%%],  R: %4.1f[%%]\r\n",
				(float)duty_l/10.f, (float)duty_r/10.f); line++;

		// aエンコーダの角度表示
		printf("<Encoder> L: %5.1f[deg],  R: %5.1f[deg]\r\n",
				RAD2DEG(Encoder_GetAngle_Left()), RAD2DEG(Encoder_GetAngle_Right())); line++;

		// aマウスのスピード表示 from encoder
		printf("<mouse_speed> speed_m : %5.3f[m/s]  spwwd_w : %5.3f[m/s] \r\n ",
				speed_m,IMU_GetGyro_Z());line++;

		// aバッテリー電圧の表示
		printf("<Battery> %3.2f[V]\r\n", Battery_GetVoltage()); line++;

		// a壁センサのAD値表示
		printf("<IR Sensor> Front_R: %4d, Side_R: %4d, Side_L: %4d, Front_L: %4d\r\n",
				Sensor_GetValue(3), Sensor_GetValue(2), Sensor_GetValue(1), Sensor_GetValue(0)); line++;
		Sensor_DebugPrintf();line++;

		//壁センサの距離変換結果確認
		printf("<IR Sensor_length> Front_R: %8.8f, Front_L: %8.8f\r\n",
				SensorValue2length(3),SensorValue2length(0)); line++;
		// IMU（加速度計とジャイロ）の計測値表示
		printf("<IMU> Accel_X: %5.3f[m/s^2], Gyro_Z: %6.3f[rad/s]\r\n",
				IMU_GetAccel_X(), IMU_GetGyro_Z()); line++;
		// a壁センサ用タイマ
		// printf("<IR Sensor_tim> tim: %5.8f\r\n",ABS(front_sensor_ref - SensorValue2length(3)) );line++;
		// 壁センサ
		printf("front_sensor_ref - real, r: %5.8f  l: %5.8f \r\n",ABS(front_sensor_r_ref - SensorValue2length(3)),ABS(front_sensor_l_ref - SensorValue2length(0)) );line++;

		Motor_SetDuty_Left(duty_l);
		Motor_SetDuty_Right(duty_r);

		fflush(stdout);
		HAL_Delay(500);
		// a画面のクリア
		printf("%c[0J", 0x1b);
		printf("%c[%dA", 0x1b, line);
	}
	Motor_StopPWM();
}

