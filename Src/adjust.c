

#include <stdio.h>
#include <math.h>
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

static float target_vol_sum_frontwall = 0;		//前壁制御によるモータ印加電圧の和[V]
static float target_vol_diff_frontwall = 0;		//前壁制御によるモータ印加電圧の差[V]

static uint16_t fornt_wall_calibrate_tim = 0;  	//前壁補正用カウンタ

static double target_sensor_sl = 0;				//左壁距離目標値[m]
static double target_sensor_sr = 0;				//右壁距離目標値[m]

static float target_vol_diff_sidewall = 0;			//横壁制御におけるモータ印加電圧の差[V]

static side_wall_ctrl side_wall_ctrl_mode = none;	//横壁補正モード（左、右、両方、なし)
static int16_t side_sensor_l_th = 0;				//左壁制御の閾値
static int16_t side_sensor_r_th = 0;				//右壁制御の閾値

//機能	: adjust.cの1msタスクまとめ
//引数	: なし
//返り値	: なし
void adjust_1ms (void)
{
	set_target_side_sensor();		//初期位置のセンサ値を横壁制御目標値にセット
	calc_side_wall_ctrl_mode();		//横壁補正のモードを決定
	adjust_theta_side_wall();		//横壁補正のモードに応じ、軌跡制御における角度を調整する（未実装）
	calc_motor_vol_side_wall();		//横壁補正における印加電圧計算
	calc_motor_vol_front_wall();	//前壁制御における印加電圧計算
	calibrate_tim();				//前壁制御用タイマ
}

//機能	: 壁トレースの目標距離をセットする
//引数	: なし 
//返り値	: なし
//備考	:初期位置の壁距離を目標位置とする。
void set_target_side_sensor(void)
{
	static uint8_t target_set_flg = 0;
	if(target_set_flg == 0) 
	{
		//target_sensor_sl = SensorValue2length(1);
		//target_sensor_sr = SensorValue2length(2);
		//デバッグ中	
		target_sensor_sl = 0.024331;
		target_sensor_sr = 0.023078;	

		target_set_flg = 1;
	}
}

//機能	: 壁トレースの右壁目標距離を取得する
//引数	: なし
//返り値	: 右壁距離目標値
float get_target_sensor_sr ( void )
{
	return (float)target_sensor_sr;
}

//機能	: 壁トレースの左壁目標距離を取得する
//引数	: なし
//返り値	: 左壁距離目標値
float get_target_sensor_sl ( void )
{
	return (float)target_sensor_sl;
}

//機能	: 壁トレースの左壁モード閾値を取得する
//引数	: なし
//返り値	: 左壁モード閾値
uint16_t get_side_sensor_l_th ( void )
{
	return side_sensor_l_th;
}


//機能	: 壁トレースの右壁モード閾値を取得する
//引数	: なし
//返り値	: 右壁モード閾値
uint16_t get_side_sensor_r_th ( void )
{
	return side_sensor_r_th;
}

//機能	: 横壁制御におけるモードを取得する
//引数	: なし
//返り値	: 横壁制御モード
uint8_t get_side_wall_ctrl_mode ( void )
{
	return side_wall_ctrl_mode;
}

//機能	: 横壁制御におけるモードを決定する（右、左、両壁、なし）
//引数	: なし
//返り値	: なし
//備考 	:1msタスク
void calc_side_wall_ctrl_mode ( void )
{
	int16_t side_sensor_r = 0; 				//右前センサ値
    int16_t side_sensor_l = 0; 				//左前センサ値
	static int16_t side_sensor_r_old = 0;	//前回右前センサ値
    static int16_t side_sensor_l_old = 0; 	//前回左前センサ値
	int16_t side_sensor_r_diff = 0;			//右前センサ値変化量
    int16_t side_sensor_l_diff = 0; 		//左前センサ値変化量
	int16_t side_sensor_r_diff_sum = 0;		//右前センサ値変化量合計
    int16_t side_sensor_l_diff_sum = 0; 	//左前センサ値変化量合計
	float side_sensor_r_diff_ave = 0;		//右前センサ値平均
    float side_sensor_l_diff_ave = 0; 		//左前センサ値平均

    typedef struct {
	int16_t	right;			
	int16_t left;			
    } ave_temp;

	static ave_temp ave_store[ir_diff_ave_num];  // データ格納用の構造体
	static uint16_t ave_counter = 0;
	uint16_t i = 0;
	uint16_t j = 0;

	//センサ値取得
	side_sensor_l = Sensor_GetValue(1);
	side_sensor_r = Sensor_GetValue(2);

	//センサ変化量取得
	side_sensor_l_diff = ABS(side_sensor_l - side_sensor_l_old);
	side_sensor_r_diff = ABS(side_sensor_r - side_sensor_r_old);

	i = ave_counter % ir_diff_ave_num;
	ave_store[i].right = side_sensor_r_diff;
	ave_store[i].left = side_sensor_l_diff;

	//移動平均処理
	for(j=0; j < ir_diff_ave_num; j++)
	{
		side_sensor_r_diff_sum += ave_store[j].right;
		side_sensor_l_diff_sum += ave_store[j].left;
	}

	side_sensor_r_diff_ave = (float)side_sensor_r_diff_sum / ir_diff_ave_num;
	side_sensor_l_diff_ave = (float)side_sensor_l_diff_sum / ir_diff_ave_num;
	
	ave_counter += 1 ;


	//変化量に応じて、壁制御の閾値を変化
	if(side_sensor_l_diff_ave > side_sensor_diff_th){
		side_sensor_l_th = side_sensor_th + side_sensor_th_add;
	}else{
		side_sensor_l_th = side_sensor_th;
	}

	if(side_sensor_r_diff_ave > side_sensor_diff_th){
		side_sensor_r_th = side_sensor_th + side_sensor_th_add;
	}else{
		side_sensor_r_th = side_sensor_th;
	}

	//閾値に応じて、横壁制御モードを設定
	if( (side_sensor_l_th < side_sensor_l) && (side_sensor_r_th < side_sensor_r))
	{
		side_wall_ctrl_mode = both_side;
	}
	else if((side_sensor_l_th < side_sensor_l))
	{
		side_wall_ctrl_mode = left;
	}
	else if((side_sensor_r_th < side_sensor_r))
	{
		side_wall_ctrl_mode = right;
	}
	else{
		side_wall_ctrl_mode = none;
	}

	//センサ値更新
	side_sensor_l_old = side_sensor_l;
	side_sensor_r_old = side_sensor_r;

}

//機能	: 各横壁制御モードにおける偏差を算出、取得する
//引数	: なし
//返り値	: 中心位置からの偏差
//備考 	:1msタスク
double get_side_wall_err(void)
{
	double side_wall_err_tmp = 0;
	
	switch(side_wall_ctrl_mode){
		case left:
			side_wall_err_tmp = -2 * (target_sensor_sl - SensorValue2length(1));
			break;
		
		case right:
			side_wall_err_tmp = 2 * (target_sensor_sr - SensorValue2length(2));
			break;
		
		case both_side:
			side_wall_err_tmp = (target_sensor_sr - SensorValue2length(2)) - (target_sensor_sl - SensorValue2length(1));
			break;
		
		case none:
			side_wall_err_tmp = 0;
			break;						
	}

	return side_wall_err_tmp;
}

//機能	: 各横壁制御モードにおける角度補正処理を入れる
//引数	: なし
//返り値	: なし
//備考 	:1msタスク
void adjust_theta_side_wall(void)
{
	switch(side_wall_ctrl_mode){
		case left:
			break;
		case right:
			break;
		case both_side:
			break;
		case none:
			break;						
	}
	//軌跡制御のI項にフィルタをかける。各モードに応じた感じで。
}

//機能	: 偏差から横壁制御のモータ印加電圧を計算する。
//引数	: なし
//返り値	: なし
//備考 	:1msタスク
void calc_motor_vol_side_wall(void)
{
	double side_wall_err = 0;
	double side_wall_err_P = 0;
	double side_wall_err_D = 0;
	double side_wall_err_PD = 0;
	static double side_wall_err_prev = 0;
	static double side_wall_err_D_prev = 0;

	//偏差取得
	side_wall_err = get_side_wall_err();

	//P項
	side_wall_err_P = side_wall_P * side_wall_err;
	
	//D項
	//side_wall_err_D = (side_wall_err_D_prev + side_wall_D * side_wall_fil * (side_wall_err - side_wall_err_prev))
	//							/(1+side_wall_fil*0.001);

	//D項
	//y方向の速度
	//速度*目標角度と実角度のエラー
	side_wall_err_D = -1 * side_wall_D * get_move_speed() * (get_rotation_angle() - get_ideal_angle());

	//合算
	side_wall_err_PD = side_wall_err_P + side_wall_err_D;

	/*印加電圧算出*/
	target_vol_diff_sidewall = (float)side_wall_err_PD;

	/*パラメータ更新*/
	side_wall_err_prev = side_wall_err;
	side_wall_err_D_prev = side_wall_err_D;

}

//機能	: 横壁補正によるモータ印加電圧の差を取得する
//引数	: なし
//返り値	: 横壁補正による右モータ印加電圧
float get_target_vol_diff_sidewall ( void )
{
	return target_vol_diff_sidewall;
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
	target_vol_sum_frontwall = front_sensor_move_PID;
	target_vol_diff_frontwall = front_sensor_rotate_PID;

	/*パラメータ更新*/
    front_sensor_move_err_prev = front_sensor_move_err;   	
	front_sensor_rotate_err_prev = front_sensor_rotate_err; 
    front_sensor_move_D_prev = front_sensor_move_err_D;		
    front_sensor_rotate_D_prev = front_sensor_rotate_err_D;

}

//機能	: 前壁補正によるモータ印加電圧の和を取得する
//引数	: なし
//返り値	: 前壁補正による右モータ印加電圧
float get_target_vol_sum_frontwall ( void )
{
	return target_vol_sum_frontwall;
}

//機能	: 前壁補正によるモータ印加電圧の差を取得する
//引数	: なし
//返り値	: 前壁補正による右モータ印加電圧
float get_target_vol_diff_frontwall ( void )
{
	return target_vol_diff_frontwall;
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



