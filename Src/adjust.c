//irセンサによる補正記述予定

#include <stdio.h>
#include "param.h"
#include "ir_sensor.h"


static double front_sensor_move_err_I = 0;   	//偏差和のI項
static double front_sensor_rotate_err_I = 0;   	//偏差差のI項
static double front_sensor_move_err_prev = 0;   //前回偏差和
static double front_sensor_rotate_err_prev = 0; //前回偏差差
static double front_sensor_move_D_prev = 0; 	//前回偏差和微分
static double front_sensor_rotate_D_prev = 0;   //前回偏差差微分

static float target_vol_r_frontwall = 0;		//前壁制御による右モータ印加電圧[V]
static float target_vol_r_frontwall = 0;		//前壁制御による左モータ印加電圧[V]

static uint16_t fornt_wall_calibrate_tim = 0;  	//前壁補正用カウンタ

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

    /*m 印加電圧算出*/
	target_vol_r_frontwall = (front_sensor_move_PID + front_sensor_rotate_PID)/2;
	target_vol_r_frontwall = (front_sensor_move_PID - front_sensor_rotate_PID)/2;

	/*パラメータ更新*/
    front_sensor_move_err_prev = front_sensor_move_err;   	
	front_sensor_rotate_err_prev = front_sensor_rotate_err; 
    front_sensor_move_D_prev = front_sensor_move_err_D;		
    front_sensor_rotate_D_prev = front_sensor_rotate_err_D;

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



