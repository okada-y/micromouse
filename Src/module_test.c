/*
 * module_test.c
 *
 *  Created on: Aug 13, 2019
 *      Author: 岡田 泰裕
 */


#include "index.h"
#include "module_test.h"
#include "ir_sensor.h"
#include "battery.h"
#include "imu.h"
#include "param.h"
#include "mouse_state.h"
#include "target.h"
#include "exvol.h"
#include "stm32f4xx_hal.h"
#include "encorder.h"
#include "interrupt.h"
#include "adjust.h"	


static uint16_t log_counter = 0 ; //ログ取得開始からの時間監視用カウンタ[ms]

typedef struct {	//データ格納用構造体の定義（最大２０個とすること（４秒の時））
	
	
	#ifdef DATA_DEFAULT
	float	time;				//測定開始からの時間[m/s]
	float	target_d_m;			//目標移動距離[m]
	float 	ideal_d_m;			//理想の並進方向移動距離[m]
	float	real_d_m;			//並進方向移動距離[m]
	float   target_speed_m;		//目標速度[m/s]
	float	speed_m;			//並進方向の速度[m/s]
	float	speed_m_ave;		//並進方向の速度平均[m/s]
	float   accel_m;			//並進方向の加速度(m/s)
	float	accel_m_ave;		//並進方向の加速度平均(m/s)
	float	target_d_w;			//目標角度
	float	ideal_d_w;			//理想角度
	float	real_d_w;			//回転角度(rad)
	float   target_speed_w;		//目標角速度(rad/s)
	float	speed_w;			//実際の角速度(rad/s)
	float   duty_r;				//右モータ操作量（duty)
	float   duty_l;				//左モータ操作量(duty)
	float   front_sensor_r;		//右前壁センサ値
	float   front_sensor_l;		//左前壁センサ値
	float   side_sensor_r;		//右横壁センサ値
	float   side_sensor_l;		//左横壁センサ値
	float   front_r;			//壁との距離（右前センサ）(m)
	float   front_l;			//壁との距離（左前センサ）(m)）
	float   side_r;				//壁との距離（右前センサ）(m)
	float   side_l;				//壁との距離（左前センサ）(m)）
	float   side_vol;			//横壁制御での電圧出力
	float   side_mode;			//横壁制御のモード
	float 	V_battery;			//バッテリー電圧
	#endif

	#ifdef DATA_SIDE
	float	time;				//測定開始からの時間[m/s]
	float	real_d_m;			//並進方向移動距離[m]
	float	speed_m_ave;		//並進方向の速度平均[m/s]
	float   duty_r;				//右モータ操作量（duty)
	float   duty_l;				//左モータ操作量(duty)
	float   front_sensor_r;		//右前壁センサ値
	float   front_sensor_l;		//左前壁センサ値
	float   side_sensor_r;		//右横壁センサ値
	float   side_sensor_l;		//左横壁センサ値
	float   front_r;			//壁との距離（右前センサ）(m)
	float   front_l;			//壁との距離（左前センサ）(m)
	float   side_r;				//壁との距離（右前センサ）(m)
	float   side_l;				//壁との距離（左前センサ）(m)
	float	side_r_target;		//右壁目標距離
	float	side_l_target;		//左壁目標距離
	float	side_r_th;			//右壁制御モード閾値
	float	side_l_th;			//左壁制御モード閾値
	float   side_vol;			//横壁制御での電圧出力
	float   side_mode;			//横壁制御のモード
	float 	V_battery;			//バッテリー電圧


	#endif
} log_struct;

static log_struct log_store[log_count_lim / log_count_step];  // データ格納用の構造体


/* ---------------------------------------------------------------
	ログ取得カウンタ初期化関数
--------------------------------------------------------------- */
void log_init (void)
{
	log_counter = 0; //ログカウンタの初期化
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
			//測定モード１
			#ifdef DATA_DEFAULT
			log_store[i].time = (float)log_counter;
			log_store[i].target_d_m = (float)get_target_length();
			log_store[i].ideal_d_m = (float)get_ideal_length();
			log_store[i].real_d_m = (float)get_move_length();
			log_store[i].target_speed_m =  (float)get_target_move_speed();
			log_store[i].speed_m = (float)get_move_speed();
			log_store[i].speed_m_ave = (float)get_move_speed_ave();
			log_store[i].accel_m = (float)IMU_GetAccel_X();
			log_store[i].accel_m_ave = (float)get_move_accel_ave();
			log_store[i].target_d_w = (float)get_target_angle();
			log_store[i].ideal_d_w = (float)get_ideal_angle();
			log_store[i].real_d_w = (float)get_rotation_angle();
			log_store[i].target_speed_w = (float)get_target_rotation_speed();
			log_store[i].speed_w = (float)get_rotation_speed();
			log_store[i].duty_r = (float)get_target_duty_r();
			log_store[i].duty_l = (float)get_target_duty_l();
			log_store[i].front_sensor_r = (float)Sensor_GetValue(3);
			log_store[i].front_sensor_l = (float)Sensor_GetValue(0);
			log_store[i].side_sensor_r = (float)Sensor_GetValue(2);
			log_store[i].side_sensor_l = (float)Sensor_GetValue(1);
			log_store[i].front_r = (float)SensorValue2length(3);
			log_store[i].front_l = (float)SensorValue2length(0);
			log_store[i].side_r = (float)SensorValue2length(2);
			log_store[i].side_l = (float)SensorValue2length(1);
			log_store[i].side_vol = (float)get_target_vol_diff_sidewall();
			log_store[i].side_mode = (float)get_side_wall_ctrl_mode();
			log_store[i].V_battery =(float) Battery_GetVoltage();
			#endif

			//測定モード２
			#ifdef DATA_SIDE
			log_store[i].time = (float)log_counter;
			log_store[i].real_d_m = (float)get_move_length();
			log_store[i].speed_m_ave = (float)get_move_speed_ave();
			log_store[i].duty_r = (float)get_target_duty_r();
			log_store[i].duty_l = (float)get_target_duty_l();
			log_store[i].front_sensor_r = (float)Sensor_GetValue(3);
			log_store[i].front_sensor_l = (float)Sensor_GetValue(0);
			log_store[i].side_sensor_r = (float)Sensor_GetValue(2);
			log_store[i].side_sensor_l = (float)Sensor_GetValue(1);
			log_store[i].front_r = (float)SensorValue2length(3);
			log_store[i].front_l = (float)SensorValue2length(0);
			log_store[i].side_r = (float)SensorValue2length(2);
			log_store[i].side_l = (float)SensorValue2length(1);
			log_store[i].side_r_target = (float)get_target_sensor_sr();
			log_store[i].side_l_target = (float)get_target_sensor_sl();
			log_store[i].side_r_th = (float)get_side_sensor_r_th();
			log_store[i].side_l_th = (float)get_side_sensor_l_th();
			log_store[i].side_vol = (float)get_target_vol_diff_sidewall();
			log_store[i].side_mode = (float)get_side_wall_ctrl_mode();
			log_store[i].V_battery =(float) Battery_GetVoltage();
			#endif
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
	//一行目はパラメータ名
	#ifdef DATA_DEFAULT
	printf ("TIME[ms],target_distance[m],ideal_distance[m],current_distance[m],TARGET_SPEED_m[m/s],SPEED_m[m/s],SPEED_m_ave[m/s],"
			"accel_m[m/s2],accel_m_ave[m/s2],target_angle[rad],ideal_angle[rad],current_angle[rad],target_speed_w[rad/s],"
			"speed_w[rad/s],DUTY_R[%%],Duty_L[%%],front_r,front_l,side_r,side_l,front_r[m],front_l[m],side_r[m],side_l[m],side_vol,side_mode,V_battery[V]\r\n");	//パラメータ名を記述
	#endif

	#ifdef DATA_SIDE
	printf ("TIME[ms],current_distance[m],SPEED_m_ave[m/s],"
			"DUTY_R[%%],Duty_L[%%],front_r,front_l,side_r,side_l,"
			"front_r[m],front_l[m],side_r[m],side_l[m],side_r_target[m],side_l_target[m],"
			"side_r_th,side_l_th,side_vol[V],side_mode,V_battery[V]\r\n");	//パラメータ名を記述
	#endif

	for(i = 0; i <= j ; i++)
	{
		#ifdef DATA_DEFAULT
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
		printf("%f,",log_store[i].side_r);
		printf("%f,",log_store[i].side_l);
		printf("%f,",log_store[i].side_vol);
		printf("%f,",log_store[i].side_mode);
		printf("%f",log_store[i].V_battery);			//最後はカンマなし
		printf("\r\n"); //m 改行
		#endif

		#ifdef DATA_SIDE
		printf("%f,",log_store[i].time);
		printf("%f,",log_store[i].real_d_m);
		printf("%f,",log_store[i].speed_m_ave);
		printf("%f,",log_store[i].duty_r);
		printf("%f,",log_store[i].duty_l);
		printf("%f,",log_store[i].front_sensor_r);
		printf("%f,",log_store[i].front_sensor_l);
		printf("%f,",log_store[i].side_sensor_r);
		printf("%f,",log_store[i].side_sensor_l);
		printf("%f,",log_store[i].front_r);
		printf("%f,",log_store[i].front_l);
		printf("%f,",log_store[i].side_r);
		printf("%f,",log_store[i].side_l);
		printf("%f,",log_store[i].side_r_target);
		printf("%f,",log_store[i].side_l_target);
		printf("%f,",log_store[i].side_r_th);
		printf("%f,",log_store[i].side_l_th);
		printf("%f,",log_store[i].side_vol);
		printf("%f,",log_store[i].side_mode);
		printf("%f",log_store[i].V_battery);			//最後はカンマなし
		printf("\r\n"); // 改行

		#endif

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
				get_move_speed(),IMU_GetGyro_Z());line++;

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
		printf("front_sensor, sl: %5.8f  sr: %5.8f \r\n",SensorValue2length(1),SensorValue2length(2));line++;

		// 横壁制御
		printf("side_wall_ctrl_mode %d \r\n",get_side_wall_ctrl_mode());line++;
		printf("r_target sl: %5.8f  sr: %5.8f \r\n",get_target_sensor_sl(),get_target_sensor_sr());line++;
		
		fflush(stdout);
		HAL_Delay(500);
		// a画面のクリア
		printf("%c[0J", 0x1b);
		printf("%c[%dA", 0x1b, line);
	}
}

