

#include <stdio.h>
#include "adjust.h"
#include "param.h"
#include "exvol.h"
#include "mouse_state.h"
#include "target.h"
#include "ir_sensor.h"


static double front_sensor_move_err_I = 0;   	//偏差和のI項
static double front_sensor_rotate_err_I = 0;   	//偏差差のI項
static double front_sensor_move_err_prev = 0;   //前回偏差和
static double front_sensor_rotate_err_prev = 0; //前回偏差差
static double front_sensor_move_D_prev = 0; 	//前回偏差和微分
static double front_sensor_rotate_D_prev = 0;   //前回偏差差微分

static float target_vol_r_frontwall = 0;		//前壁制御による右モータ印加電圧[V]
static float target_vol_l_frontwall = 0;		//前壁制御による左モータ印加電圧[V]

static uint16_t fornt_wall_calibrate_tim = 0;  	//前壁補正用カウンタ

//機能	: adjust.cの1msタスクまとめ
//引数	: なし
//返り値	: なし
void adjust_1ms (void)
{
	calc_motor_vol_front_wall();	//前壁制御における印加電圧計算
	calibrate_tim();				//前壁制御用タイマ
}


//機能	: 前壁制御におけるモータ印加電圧を計算する。
//引数	: なし
//返り値	: なし
//備考	:1msタスク
void calc_motor_vol_front_wall ( void )
{
	double front_sensor_r = 0; 				//右前センサ値
    double front_sensor_l = 0; 				//左前センサ値
    double front_sensor_r_err = 0;   		//右前センサの偏差
    double front_sensor_l_err = 0;			//左前センサの偏差
    double front_sensor_move_err = 0;   	//センサの偏差の和
    double front_sensor_rotate_err = 0;		//センサの偏差の差

    double front_sensor_move_err_P = 0;   	//偏差和のP項
    double front_sensor_rotate_err_P = 0;   //偏差差のP項
    double front_sensor_move_err_D = 0;   	//偏差和のD項
    double front_sensor_rotate_err_D = 0;   //偏差差のD項
    double front_sensor_move_PID = 0;  		//
    double front_sensor_rotate_PID = 0;   		//


	//センサ値取得
	front_sensor_l = SensorValue2length(0);
	front_sensor_r = SensorValue2length(3);

	//偏差取得
	front_sensor_l_err = front_sensor_l - front_sensor_l_ref ;
	front_sensor_r_err = front_sensor_r - front_sensor_r_ref ;

	/* 偏差変換*/
	front_sensor_move_err = front_sensor_r_err + front_sensor_l_err;
	front_sensor_rotate_err = (front_sensor_r_err - front_sensor_l_err)/chassis_width; //角度に変換(atanを0近傍で線形化)

	/*偏差積分(I項)*/
	front_sensor_move_err_I  += front_sensor_move_KI * 0.001 * front_sensor_move_err;
	front_sensor_rotate_err_I  += front_sensor_rotate_KI * 0.001 * front_sensor_rotate_err;
	/*P項*/
	front_sensor_move_err_P = front_sensor_move_KP * front_sensor_move_err;
	front_sensor_rotate_err_P = front_sensor_rotate_KP * front_sensor_rotate_err;
	/*D項*/
	front_sensor_move_err_D = (front_sensor_move_D_prev + front_sensor_move_KD * front_sensor_move_fil * (front_sensor_move_err - front_sensor_move_err_prev))
								/(1+front_sensor_move_fil*0.001);
	front_sensor_rotate_err_D = (front_sensor_rotate_D_prev + front_sensor_rotate_KD * front_sensor_rotate_fil * (front_sensor_rotate_err - front_sensor_rotate_err_prev))
								/(1+front_sensor_rotate_fil*0.001);
	/*PID*/
	front_sensor_move_PID = front_sensor_move_err_P + front_sensor_move_err_I + front_sensor_move_err_D;
    front_sensor_rotate_PID = front_sensor_rotate_err_P + front_sensor_rotate_err_I + front_sensor_rotate_err_D;

    /*印加電圧算出*/
	target_vol_r_frontwall = (front_sensor_move_PID + front_sensor_rotate_PID)/2;
	target_vol_l_frontwall = (front_sensor_move_PID - front_sensor_rotate_PID)/2;

	/*パラメータ更新*/
    front_sensor_move_err_prev = front_sensor_move_err;   	
	front_sensor_rotate_err_prev = front_sensor_rotate_err; 
    front_sensor_move_D_prev = front_sensor_move_err_D;		
    front_sensor_rotate_D_prev = front_sensor_rotate_err_D;

}

//機能	: 前壁補正による右モータ印加電圧を取得する
//引数	: なし
//返り値	: 前壁補正による右モータ印加電圧
float get_target_vol_r_frontwall ( void )
{
	return target_vol_r_frontwall;
}

//機能	: 前壁補正による左モータ印加電圧を取得する
//引数	: なし
//返り値	: 前壁補正による右モータ印加電圧
float get_target_vol_l_frontwall ( void )
{
	return target_vol_l_frontwall;
}

//機能	: 前壁制御の操作履歴を消す
//引数	: なし
//返り値	: なし
void clr_frontwall_operate_history ( void )
{
	front_sensor_move_err_I = 0;   		
	front_sensor_rotate_err_I = 0; 	  	
	front_sensor_move_err_prev = 0;	   	
	front_sensor_rotate_err_prev = 0; 	
	front_sensor_move_D_prev = 0; 		
	front_sensor_rotate_D_prev = 0;	   	
}


////////////////////////////////////////
/* マウス位置補正関数		　			*/
/* 壁を使った位置の補正用関数			*/
////////////////////////////////////////

/* memo:前壁補正
 * param:
 *  * */
void fornt_wall_calibrate (void)
{
	double temp_r;
	double temp_l;
	double temp;

	set_mode_ctrl(front_wall); 		//制御モードを前壁補正モードに
	fornt_wall_calibrate_tim = 0; 	//前壁制御用タイマを初期化

	while(1)
	{
		//偏差取得
		temp_r = ABS(front_sensor_r_ref - SensorValue2length(3));
		temp_l = ABS(front_sensor_l_ref - SensorValue2length(0));
		//偏差の最大値取得
		temp = MAX(temp_r,temp_l);
		//センサ値が基準より差を持つとき、タイマをリセット
		if(temp > front_sensor_th)
		{
		 fornt_wall_calibrate_tim = 0;
		}
		//キャリブレーション時間を超えるとき、ブレイク
		if(fornt_wall_calibrate_tim >= calib_tim )
		{
		 break;
		}
	}
	//補正終了時、移動距離、角度に理想値を代入
	set_move_length(get_ideal_length());
	set_rotation_angle(get_ideal_angle());
	
	//制御モードを軌跡制御に変更
	set_mode_ctrl(trace);
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



