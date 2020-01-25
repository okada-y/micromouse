/*
 * target.c
 *
 *  Created on: Aug 7, 2019
 *      Author: 岡田 泰裕
 */

#include "index.h"

float target_speed_m = 0; 		//m 並進方向の目標速度[m/s]
float target_speed_m_fin = 0;	//m 区間終了時の目標速度[ m/s]
float target_speed_w = 0;		//m 目標角速度[rad/s]
float g_accelation_m = 0;		//m 加速度バッファ[m/s^2]
float g_accelation_w = 0;		//m 角加速度バッファ[m/s^2]
float target_distance_m = 0; 	//m　目標距離[m]
float target_distance_w = 0; 	//m　目標角度[rad]
float ideal_distance_m = 0; 	//m 理想の並進方向移動距離[m]
float ideal_distance_w = 0; 	//m 理想の実角度[rad]

float operation_amount_r = 0; 		//m 右タイヤの操作量[ duty % ]
float operation_amount_l = 0;		//m 左タイヤの操作量[ duty % ]

float real_distance_m = 0;			//m 並進方向移動距離[ m ]
float real_distance_w = 0;			//m 回転方向移動距離[ rad]

uint8_t move_dir_flg = 0;			//m　移動方向フラグ　0:前進 1:後進
uint8_t rotation_dir_flg = 0;		//m　回転方向フラグ　0:時計周り 1:反時計周り
uint8_t accel_dir_flg = 0;		    //m　加速方向フラグ　0:加速 1:減速
uint8_t run_first_flg = 0;			//m 走行開始フラグ 0:走行開始時　1:それ以外
uint8_t correction_mode = 0;		//m 補正モードフラグ 0:目標速度　1:前壁センサ値

uint16_t fornt_wall_calibrate_tim = 0; //m 前壁補正用カウンタ


///////////////////////
/* a 各モードフラグ定義	 */
///////////////////////

typedef enum {
	forward_mode 	= 0,
	backward_mode 	= 1,
} direction_mode;

typedef enum {
	counter_clockwise 	= 0,
	clockwise 	= 1,
} rotation_mode;

typedef enum {
	acceleration 	= 0, //m加速
	deceleration 	= 1, //m減速
} accel_mode;

typedef enum {
	start 	= 0, //m走行開始
	other 	= 1, //mそれ以外
} serch_start;



////////////////////////////////
/* a パラメータ更新関数(1msタスク)	 */
////////////////////////////////

/* memo:並進方向移動距離更新(1msタスク)
 * param
 */
void real_distance_m_calc (void)
{
	real_distance_m += g_ave_speed_m * 0.001; // [m/s] * [s]
}

/* memo:回転方向移動角度取得(1msタスク)
 * param
 */
void real_distance_w_calc (void)
{
	real_distance_w += IMU_GetGyro_Z() * 0.001; // [rad/s] * [s]
}


/* memo:目標速度更新処理(1msタスク)
 * 現在目標速度に1ms*加速度を加算し、加速、回転モードごとにリミット処理
 * param:
 *  * */
void target_speed_inc(void)
{

	/*m加速処理*/
	target_speed_m += g_accelation_m * 0.001; //1ms分の加速度加算
	target_speed_w += g_accelation_w * 0.001; //1ms分の角加速度加算

	/*limit処理*/
	switch(move_dir_flg)
	{
		case forward_mode:

			if (target_speed_m < 0)	//m 下限スピード処理
					{
						target_speed_m = 0;
					}
					else if (target_speed_m > speed_m_max)//上限スピード処理
					{
						target_speed_m = speed_m_max;
					}
			break;

		case backward_mode:
			if (target_speed_m > 0)	//m 上限スピード処理
				{
					target_speed_m = 0;
				}
				else if (target_speed_m < -1 * speed_m_max)//下限スピード処理
				{
					target_speed_m = -1 * speed_m_max;
				}
			break;
	}

	switch(rotation_dir_flg)
		{
			case counter_clockwise:

				if (target_speed_w < 0)	//m 下限スピード処理
						{
							target_speed_w = 0;
						}
						else if (target_speed_w > speed_w_max)//上限スピード処理
						{
							target_speed_w = speed_w_max;
						}
				break;

			case clockwise:
				if (target_speed_w > 0)	//m 上限スピード処理
					{
						target_speed_w = 0;
					}
					else if (target_speed_w < -1 * speed_w_max)//下限スピード処理
					{
						target_speed_w = -1 * speed_w_max;
					}
				break;
		}


	/*m移動距離、角度算出*/
	ideal_distance_m += target_speed_m * 0.001;
	ideal_distance_w += target_speed_w * 0.001;


}

/* memo:加速度更新(1msタスク)
 * 現在移動距離と目標移動距離に応じて、加速度をスイッチ
 * param:
 *  * */
void target_speed_m_calc(void)
{
	float l_target_distance_m = 0;		//m 目標距離(ローカル) [m]
	float l_accelation_m = 0; 			//m 目標加速度（ローカル) [m/s^2]
	float l_speed_m = 0;				//m 目標速度(ローカル)[m/s]
	float l_speed_m_fin= 0;				//m 目標終端速度(ローカル)[m/s]
	float l_distance_m = 0;				//m 現在移動距離[m]
	float l_deccel_distance_m = 0;		//m 減速開始距離[m]

	/*m グローバル変数参照*/
	l_speed_m = target_speed_m;									//m 目標並進速度で制御
	l_speed_m_fin = target_speed_m_fin;							//m 目標距離到達時の目標速度
	l_distance_m = 	ideal_distance_m;							//m 理想移動距離で制御
	l_target_distance_m = target_distance_m;					//m 目標並進距離


	switch(accel_dir_flg)
	{
		case acceleration: //m 加速モード時、常に加速(リミットに引っ掛かり続ける)
			switch(move_dir_flg)
			{
			case forward_mode:
				l_accelation_m = 1 * accelation_m;
				break;
			case backward_mode:
				l_accelation_m = -1 * accelation_m;
				break;
			}
			break;

		case deceleration:
			switch(move_dir_flg)
			{
			case forward_mode:
				/*m 減速に必要な距離算出*/
				l_deccel_distance_m = ((l_speed_m*l_speed_m)-(l_speed_m_fin*l_speed_m_fin)) / (2*accelation_m);
				if(l_distance_m < (l_target_distance_m - l_deccel_distance_m)){			//m減速開始前
					l_accelation_m =  1 * accelation_m; //mリミットにかかるまで加速
				}
				else{																	//m減速開始後
					l_accelation_m = -1 * accelation_m;
				}
				break;
			case backward_mode:
				/*m 加速に必要な距離算出*/
				l_deccel_distance_m = ((l_speed_m*l_speed_m)-(l_speed_m_fin*l_speed_m_fin)) / (2*accelation_m);
				if(l_distance_m > (l_target_distance_m + l_deccel_distance_m)){			//m加速開始前
					l_accelation_m = -1 * accelation_m; //mリミットにかかるまで減速
				}
				else{																	//m加速開始後
					l_accelation_m = 1 * accelation_m;
				}
				break;
			}
			break;
	}

	/*m グローバル変数に出力*/

	g_accelation_m = l_accelation_m;

}


/* memo:角加速度更新(1msタスク)
 * 現在角度と目標角度に応じて、角加速度をスイッチ
 * param:
 *  * */
void target_speed_w_calc(void)
{
	float l_target_distance_w = 0;		//m目標角度(ローカル) [rad]
	float l_accelation_w = 0; 			//m目標角加速度（ローカル) [rad/s^2]
	float l_speed_w = 0;				//m現在角速度(ローカル)[rad/s]
	float l_distance_w = 0;				//m現在角度[rad]
	float l_deccel_distance_w = 0;		//m減速開始角度[rad]

	/*mグローバル変数参照*/
	l_speed_w = target_speed_w;									//m目標並進速度で制御
	l_distance_w = 	ideal_distance_w;							//m理想移動距離で制御
	l_target_distance_w = target_distance_w;					//m目標角度

	switch(rotation_dir_flg)
	{
	case counter_clockwise:
		/*m減速に必要な角度算出*/
		l_deccel_distance_w = 0.5 * l_speed_w * l_speed_w / accelation_w;

		if(l_distance_w < (l_target_distance_w - l_deccel_distance_w)){			//m減速開始前
			l_accelation_w =  1 * accelation_w;
		}
		else{																	//m減速開始後
			l_accelation_w = -1 * accelation_w;
		}
		break;
	case clockwise:
		/*m加速に必要な角度算出*/
		l_deccel_distance_w = 0.5 * l_speed_w * l_speed_w / accelation_w;

		if(l_distance_w > (l_target_distance_w + l_deccel_distance_w)){			//m加速開始前
			l_accelation_w =  -1 * accelation_w;
		}
		else{																	//m加速開始後
			l_accelation_w = 1 * accelation_w;
		}
		break;

		break;
	}

	/*mグローバル変数に出力*/

	g_accelation_w = l_accelation_w;

}


/* memo:印加電圧更新(1msタスク)
 *　目標加速度、現在速度でFF制御
 *　目標速度との偏差でFB制御
 * param:
 *  * */

void Operation_amount_calc(void)
{
//	static float target_acceleration_m = 0;		//m 目標並進方向加速度[ m/s^2]
//	static float target_acceleration_w = 0;		//m 目標並進方向加速度[ rad/s^2]

//	static int8_t offset_flg_r = 0;				//右モータ電圧オフセット電圧フラグ
//	static int8_t offset_flg_l = 0;				//右モータ電圧オフセット電圧フラグ

	static float target_vol_r = 0;	 			//m 右モータの目標電圧[ V]
	static float target_vol_l = 0; 				//m 左モータの目標電圧[ V]

//	static float ff_target_vol_r = 0;			//ff制御の右モータ電圧[V]
//	static float ff_target_vol_l = 0;			//ff制御の左モータ電圧[V]

	static float fb_target_vol_r = 0;			//fb制御の右モータ電圧[V]
	static float fb_target_vol_l = 0;			//fb制御の右モータ電圧[V]

	static float speed_m_err = 0; 				//m 並進方向の目標速度偏差
	static float speed_w_err = 0;				//m 目標角速度偏差
	static float speed_w_err_D = 0;				//m 角速度偏差微分
	static float speed_w_err_prev = 0;			//m 前回角速度偏差
	static float speed_w_err_D_prev = 0;		//m 前回角速度偏差微分
	static float speed_m_err_I = 0; 			//m 速度和の偏差の積分
	static float speed_w_err_I = 0;				//m 速度差の偏差の積分



	static float speed_m_err_PID = 0; 			//m　並進方向速度差のPID量
	static float speed_w_err_PID = 0;			//m 回転方向速度差のPID量

	/*m 壁補正用*/
    double l_front_sensor_r = 0; 		//m 右前センサ値のバッファ
    double l_front_sensor_l = 0; 		//m 左前センサ値のバッファ
    double l_front_sensor_r_err = 0;   //m 右前センサの偏差
    double l_front_sensor_l_err = 0;	//m 左前センサの偏差
    double l_front_sensor_m_err = 0;   //m センサの偏差の和
    double l_front_sensor_w_err = 0;	//m センサの偏差の差
    static double l_front_sensor_m_err_I = 0;   	//m 偏差和積分
    static double l_front_sensor_w_err_I = 0;   	//m 偏差差積分
    static double l_front_sensor_m_err_prev = 0;   	//m 前回偏差和
    static double l_front_sensor_w_err_prev = 0;   	//m 前回偏差差
    static double l_front_sensor_m_D_prev = 0;   	//m 前回偏差和微分
    static double l_front_sensor_w_D_prev = 0;   	//m 前回偏差差微分

    double l_front_sensor_m_err_P = 0;
    double l_front_sensor_w_err_P = 0;
    double l_front_sensor_m_err_D = 0;
    double l_front_sensor_w_err_D = 0;

    double l_front_sensor_m_PID = 0;
    double l_front_sensor_w_PID = 0;


	/*m 目標速度による制御*/
	if (correction_mode == 0){

		/*m 偏差取得*/
		speed_m_err = target_speed_m - g_ave_speed_m;		//m 目標スピードー実速度(エンコーダから)[m/s]
		speed_w_err = target_speed_w - IMU_GetGyro_Z();		//m 目標角速度ー実速度（IMUから）[rad/s]

		/*m 偏差積分*/
		speed_m_err_I = speed_m_err_I + speed_m_KI*0.001*speed_m_err;
		speed_w_err_I = speed_w_err_I + speed_w_KI*0.001*speed_w_err;
		//speed_w_err_I += speed_w_err;
		/*m 偏差微分*/
		speed_w_err_D  = (speed_w_err_D_prev + speed_w_KD*speed_w_fil*(speed_w_err-speed_w_err_prev))
		                 /(1+speed_w_fil*0.001);
		//2自由度制御(PI2)
		//p(b*r-y) + I*Ts*z/(z-1)(r-y)
		//=P(r-y)+P(b-1)*r + I*Ts*z/(z-1)(r-y)
		//=P*e+P(b-1)r + I*Ts*z/(z-1)(r-y)
		speed_m_err_PID = speed_m_KP * speed_m_err + speed_m_err_I;
		//speed_w_err_PID = speed_w_KP * (speed_w_b*target_speed_w - IMU_GetGyro_Z()) + speed_w_err_I;
		speed_w_err_PID = speed_w_KP * speed_w_err + speed_w_err_I;
				//+ speed_w_err_D;


//		/*m PI制御*/
//		speed_m_err_PID = (speed_m_KP * speed_m_err) + (speed_m_KI * speed_m_err_I);   //PIDとはいってるけどPIだけ
//		speed_w_err_PID = (speed_w_KP * speed_w_err) + (speed_w_KI * speed_w_err_I);	 //PIDとはいえ（略
//
//		/*0929_Moore-Penrose 疑似逆行列より求められたパラメータを使用する。*/
//		ff_target_vol_r = ((ff_m_a_gain * target_acceleration_m ) + (ff_m_v_gain * target_speed_m) + ff_m_f_gain * 1)//m並進方向
//							+((ff_w_a_gain * target_acceleration_w ) + (ff_w_v_gain * target_speed_w) + ff_w_f_gain * 1); //m回転方向
//		ff_target_vol_l = ((ff_m_a_gain * target_acceleration_m ) + (ff_m_v_gain * target_speed_m) + ff_m_f_gain * 1)//m並進方向
//							-((ff_w_a_gain * target_acceleration_w ) + (ff_w_v_gain * target_speed_w) + ff_w_f_gain * 1);//m回転方向

		/*FB項演算*/
		fb_target_vol_r = speed_m_err_PID + speed_w_err_PID;
		fb_target_vol_l = speed_m_err_PID - speed_w_err_PID;

		/*m 印加電圧算出*/
		target_vol_r = (fb_gain * fb_target_vol_r);
		target_vol_l = (fb_gain * fb_target_vol_l);

		/*m 前回偏差更新*/
		speed_w_err_prev = speed_w_err;			//m 前回角速度偏差
		speed_w_err_D_prev = speed_w_err_D;			//m 前回角速度偏差微分

	}

	/*m 前壁補正*/
	if(correction_mode == 1){
		l_front_sensor_l = SensorValue2length(0);
		l_front_sensor_r = SensorValue2length(3);

		/*m 偏差取得*/
		l_front_sensor_l_err = l_front_sensor_l - front_sensor_l_ref ;		//m 目標距離　－　センサ距離
		l_front_sensor_r_err = l_front_sensor_r - front_sensor_r_ref ;		//m 目標距離　－　センサ距離

		/* m 偏差変換*/
		l_front_sensor_m_err = l_front_sensor_r_err + l_front_sensor_l_err;
		l_front_sensor_w_err = (l_front_sensor_r_err - l_front_sensor_l_err)/chassis_width; //角度に変換(atanを0近傍で線形化)

		/*m 偏差積分(I項)*/
		l_front_sensor_m_err_I  += front_sensor_m_KI * 0.001 * l_front_sensor_m_err;
		l_front_sensor_w_err_I  += front_sensor_w_KI * 0.001 * l_front_sensor_w_err;

		/*P項*/
		l_front_sensor_m_err_P = front_sensor_m_KP * l_front_sensor_m_err;
		l_front_sensor_w_err_P = front_sensor_w_KP * l_front_sensor_w_err;

		/*D項*/
		l_front_sensor_m_err_D = (l_front_sensor_m_D_prev+front_sensor_m_KD*front_sensor_m_fil*(l_front_sensor_m_err - l_front_sensor_m_err_prev))
									/(1+front_sensor_m_fil*0.001);
		l_front_sensor_w_err_D = (l_front_sensor_w_D_prev+front_sensor_w_KD*front_sensor_w_fil*(l_front_sensor_w_err - l_front_sensor_w_err_prev))
									/(1+front_sensor_w_fil*0.001);

		/*PID*/
		l_front_sensor_m_PID = l_front_sensor_m_err_P + l_front_sensor_m_err_I + l_front_sensor_m_err_D;
	    l_front_sensor_w_PID = l_front_sensor_w_err_P + l_front_sensor_w_err_I + l_front_sensor_w_err_D;

	    /*m 印加電圧算出*/
		target_vol_r = (l_front_sensor_m_PID + l_front_sensor_w_PID)/2;
		target_vol_l = (l_front_sensor_m_PID - l_front_sensor_w_PID)/2;

		/*m パラメータ更新*/
	    l_front_sensor_m_err_prev = l_front_sensor_m_err;   		//m 前回偏差和
	    l_front_sensor_w_err_prev = l_front_sensor_w_err;   		//m 前回偏差差
	    l_front_sensor_m_D_prev = l_front_sensor_m_err_D;		   	//m 前回偏差和微分
	    l_front_sensor_w_D_prev = l_front_sensor_w_err_D;		   	//m 前回偏差差微分

	}

//	/*m 目標電圧に応じて補正*/
//	/*m オフセット電圧加算処理*/
//	/*m 右モータ*/
//	if(target_vol_r > 0)
//	{
//		target_vol_r += offset_voltage;
//		offset_flg_r = 1;
//	}
//	else if(target_vol_r < 0)
//	{
//		target_vol_r -= offset_voltage;
//		offset_flg_r = -1;
//	}
//	else /*target_vol_r = 0のとき、前回オフセット方向を引き継ぎ*/
//	{
//		target_vol_r += offset_flg_r * offset_voltage;
//	}
//
//	/*m左モータ*/
//	if(target_vol_l > 0)
//	{
//		target_vol_l += offset_voltage;
//		offset_flg_l = 1;
//	}
//	else if(target_vol_l < 0)
//	{
//		target_vol_l -= offset_voltage;
//		offset_flg_l = -1;
//	}
//	else /*target_speed_l = 0のとき、前回オフセット方向を引き継ぎ*/
//	{
//		target_vol_l += offset_flg_l * offset_voltage;
//	}


	/* m バッテリー電圧とモータに印加する電圧から、duty[*0.1%]を算出	*/
	operation_amount_r = target_vol_r / (Battery_GetVoltage()) * 1000;
	operation_amount_l = target_vol_l / (Battery_GetVoltage()) * 1000;


	if( operation_amount_r > 0){
		operation_amount_r = operation_amount_r + 40;  //40->30
	}
	else{
		operation_amount_r = operation_amount_r - 45; //45 ->35
	}

	if( operation_amount_l > 0){
		operation_amount_l = operation_amount_l + 40;  //40->30
	}
	else{
		operation_amount_l = operation_amount_l - 45; //45 ->35
	}

	/*mモータに電圧を印加 */
	Motor_SetDuty_Left((int16_t)operation_amount_l);
	Motor_SetDuty_Right((int16_t)operation_amount_r);


}




////////////////////////////////
/* a パラメータ設定関数			 */
////////////////////////////////

/* memo:並進方向移動距離クリア
 * param
 */
void real_distance_m_clr (void)
{
	real_distance_m = 0; // [m/s] * [s]
}

/* memo:回転方向移動角度クリア
 * param
 */
void real_distance_w_clr (void)
{
	real_distance_w= 0; // [rad/s] * [s]
}

/* memo:並進方向目標距離更新
 * param:
 */
void target_distance_m_set(float distance_m)
{
	target_distance_m += distance_m;
}

/* memo:目標角度更新
 * param:
 */
void target_distance_w_set(float distance_w)
{
	target_distance_w += distance_w;
}

/* memo:並進方向目標距離クリア
 * param:
 *  * */
void target_distance_m_clr(void)
{
	target_distance_m = 0;
}


/* memo:目標角度クリア
 * param:
 *  * */
void target_distance_w_clr(void)
{
	target_distance_w = 0;
}

/* memo:理想並進方向目標距離クリア
 * param:
 *  * */
void ideal_distance_m_clr(void)
{
	ideal_distance_m = 0;
}


/* memo:理想現在角度クリア
 * param:
 *  * */
void ideal_distance_w_clr(void)
{
	ideal_distance_w = 0;
}


////////////////////////////////
/* a フラグ設定関数				 */
////////////////////////////////

/* memo:前壁補正フラグクリア
 * param:
 *  * */
void front_wall_calib_flg_clr(void)
{
	  front_calib_flg = 0; //m　前壁補正フラグの初期化
	  right_calib_flg = 0; //m　前壁補正フラグの初期化
	  left_calib_flg = 0; //m　前壁補正フラグの初期化
}




////////////////////////////////////////
/* a マウス動作設定関数			 		*/
/* a 距離、角度に応じた速度の加減速を設定する。 */
////////////////////////////////////////

/* memo:スタート時加速
 * param:
 *  * */
void start_acceleration (void)
{
	/*m移動方向、加速モード設定*/
	move_dir_flg = forward_mode;
	accel_dir_flg = acceleration;
    target_distance_m_set(0.060286-0.045);

    /*m 半区画進むまで待機*/
    while (1)
    {
    	if(ABS(target_distance_m - ideal_distance_m) < 0.0002)//0.1mm手前まできたらブレイク
    	{
    		break;
    	}
    }
}

/* memo:半区画加速
 * param:
 *  * */
void half_acceleration (void)
{
	/*m移動方向、加速モード設定*/
	move_dir_flg = forward_mode;
	accel_dir_flg = acceleration;
    target_distance_m_set(0.045);

    /*m 半区画進むまで待機*/
    while (1)
    {
    	if(ABS(target_distance_m - ideal_distance_m) < 0.0002)//0.1mm手前まできたらブレイク
    	{
    		break;
    	}
    }
}


/* memo:半区画減速
 * param:
 *  * */
void half_deceleration (void)
{
	/*m移動方向、加速モード設定*/
	move_dir_flg = forward_mode;
	accel_dir_flg = deceleration;
    target_distance_m_set(0.045);

    /*m 半区画進むまで待機*/
    while (1)
    {
    	if(ABS(target_distance_m - ideal_distance_m) < 0.0002)//0.1mm手前まできたらブレイク
    	{
    		break;
    	}
    }
}

/* memo:一区画定速
 * param:
 *  * */
void constant_speed (void)
{
	/*m移動方向、加速モード設定*/
	move_dir_flg = forward_mode;
	accel_dir_flg = acceleration;
    target_distance_m_set(0.09);

    /*m 一区画進むまで待機*/
    while (1)
    {
    	if(ABS(target_distance_m - ideal_distance_m) < 0.0002)//0.1mm手前まできたらブレイク
    	{
    		break;
    	}
    }
}

/* memo:90度時計回りに回転
 * param:
 *  * */
void turn_clk_90 (void)
{
	/*m回転方向設定*/
	rotation_dir_flg = clockwise;
    target_distance_w_set(-PI/2);

    /*m90度回転するまで待機*/
    while (1)
    {
    	if(ABS(target_distance_w - ideal_distance_w) < 0.005)//0.3deg手前まできたらブレイク
    	{
    		break;
    	}
    }

}

/* memo:90度反時計回りに回転
 * param:
 *  * */
void turn_conclk_90 (void)
{
	/*m回転方向設定*/
	rotation_dir_flg = counter_clockwise;
    target_distance_w_set(PI/2);

    /*m90度回転するまで待機*/
    while (1)
    {
    	if(ABS(target_distance_w - ideal_distance_w) < 0.005)//0.3deg手前まできたらブレイク
    	{
    		break;
    	}
    }

}

/* memo:180度反時計回りに回転
 * param:
 *  * */
void turn_conclk_180 (void)
{
	/*m回転方向設定*/
	rotation_dir_flg = counter_clockwise;
    target_distance_w_set(PI);

    /*m180度回転するまで待機*/
    while (1)
    {
    	if(ABS(target_distance_w - ideal_distance_w) < 0.005)//0.3deg手前まできたらブレイク
    	{
    		break;
    	}
    }

}



////////////////////////////////////////
/* a 軌跡生成関数						*/
/* a 速度、角速度の加減速により軌跡を生成する。 */
////////////////////////////////////////

/* memo:一区画前進
 * param:
 *  * */
void move_front (void)
{
	if(run_first_flg == start){
		half_acceleration();//半区画加速
	}
	if(run_first_flg == other){
		constant_speed();//定速で一マス前進
	}
	run_first_flg = other;
	front_wall_calib_flg_clr();
}

/* memo:右折
 * param:
 *  * */
void move_right (void)
{
	if(run_first_flg == start){
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_clk_90();		//m時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	if(run_first_flg == other){
		half_deceleration();//m半区画減速で中央に停止
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_clk_90();		//m時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	run_first_flg = other;
	front_wall_calib_flg_clr();
}

/* memo:左折
 * param:
 *  * */
void move_left (void)
{
	if(run_first_flg == start){
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_conclk_90();	//m反時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	if(run_first_flg == other){
		half_deceleration();//m半区画減速で中央に停止
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_conclk_90();	//m反時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	run_first_flg = other;
	front_wall_calib_flg_clr();
}

/* memo:バック
 * param:
 *  * */
void move_back (void)
{
	if(run_first_flg == start){

		if(front_calib_flg == 1){
			fornt_wall_calibrate();
			if(right_calib_flg == 1){
				turn_clk_90();
				fornt_wall_calibrate();
				turn_clk_90();
			}
			else if(left_calib_flg == 1){
					turn_conclk_90();
					fornt_wall_calibrate();
					turn_conclk_90();
			}
		}
		else{
			turn_conclk_180();	//m反時計回りに180度回転
		}
		half_acceleration();//m半区画加速
	}
	if(run_first_flg == other){
		half_deceleration();//m半区画減速で中央に停止
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
			if(right_calib_flg == 1){
				turn_clk_90();
				fornt_wall_calibrate();
				turn_clk_90();
			}
			else if(left_calib_flg == 1){
					turn_conclk_90();
					fornt_wall_calibrate();
					turn_conclk_90();
			}
		}
		else{
			turn_conclk_180();	//m反時計回りに180度回転
		}
		half_acceleration();//m半区画加速
	}
	run_first_flg = other;
	front_wall_calib_flg_clr();
}


////////////////////////////////////////
/* a マウス位置補正関数					*/
/* a 壁を使った位置の補正用関数				*/
////////////////////////////////////////

/* memo:前壁補正
 * param:
 *  * */
void fornt_wall_calibrate (void)
{
	double temp_r;
	double temp_l;
	double temp;


	  correction_mode = 1; //m 前壁補正モードに切り替え
	  fornt_wall_calibrate_tim = 0; //m前壁補正タイマを初期化

	  while(1)
	  {
		  temp_r = ABS(front_sensor_r_ref - SensorValue2length(3));
		  temp_l = ABS(front_sensor_l_ref - SensorValue2length(0));
		  temp = MAX(temp_r,temp_l);
		  //mセンサ値が基準より差を持つとき、タイマをリセット
		  if(temp > front_sensor_th){
			  fornt_wall_calibrate_tim = 0;
		  }
		  //m キャリブレーション時間を超えるとき、ブレイク
		  if(fornt_wall_calibrate_tim >= calib_tim ){
			  break;
		  }
		  printf("temp: %8.5f,time:%8.5d \r\n",temp,fornt_wall_calibrate_tim);
	  }

	  //m 補正終了時、移動距離、角度を初期化
	  real_distance_m_clr();
	  real_distance_w_clr();
	  target_distance_m_clr();
	  target_distance_w_clr();
	  ideal_distance_m_clr();
	  ideal_distance_w_clr();

	  //m 補正モードを目標速度に変更
	  correction_mode = 0;
}

/* memo:前壁補正用タイマ
 * param:
 *  * */
void calibrate_tim (void){

	fornt_wall_calibrate_tim += 1;

	if(fornt_wall_calibrate_tim > calib_tim){
		fornt_wall_calibrate_tim = calib_tim;
	}
}



