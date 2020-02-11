//軌道制御、軌道補正から得た印加電圧から、
//モータに出力する印加電圧を決定する。

#include <stdio.h>
#include "main.h"
#include "exvol.h"
#include "motor.h"
#include "control.h"
#include "adjust.h"
#include "battery.h"
#include "param.h"


static ctrl_mode_num ctrl_mode = trace;
static float target_vol_sum = 0;
static float target_vol_diff = 0;
static float target_vol_r = 0;
static float target_vol_l = 0;
static int16_t target_duty_r = 0;
static int16_t target_duty_l = 0;

//機能	: モータの1msタスクまとめ
//引数	: なし
//返り値	: なし
//備考	: 1msタスク
void motor_1ms ( void )
{
	set_motor_vol();		//印加電圧の和、差を制御モードに従い決定
	clr_operate_history();	//各制御の操作履歴クリア
	calc_motor_vol();		//印加電圧の和、差から各モータの印加電圧を決定する。
	calc_vol2duty();		//印加電圧を変調率に変換
	motor_duty_adjust();	//低変調率を避ける調整
	set_motor_duty();		//モータに電圧を印加
}


//機能 	:制御モードを設定
//引数 	:なし
//返り値:なし
void set_mode_ctrl(ctrl_mode_num mode_num)
{
	ctrl_mode = mode_num;
}

//機能 	:制御モード番号に従い、電圧を印可
//引数 	:なし
//返り値:なし
void set_motor_vol(void)
{
	switch (ctrl_mode)
	{
	case trace:
		set_motor_vol_trace();
		break;
	
	case front_wall:
		set_motor_vol_front_wall();
		break;
	
	case side_wall:
		set_motor_vol_side_wall();
		break;
	
	default:
		clr_motor_vol();

	}
}

//機能 	:軌跡制御時の印可電圧出力
//引数 	:なし
//返り値:なし
void set_motor_vol_trace(void)
{
	target_vol_sum = get_target_vol_sum_ctrl();
	target_vol_diff = get_target_vol_diff_ctrl();
}

//機能 	:前壁制御時の印可電圧出力
//引数 	:なし
//返り値:なし
void set_motor_vol_front_wall(void)
{
	target_vol_sum = get_target_vol_sum_frontwall();
	target_vol_diff = get_target_vol_diff_frontwall();
}

//機能 	:横壁制御時の印可電圧出力(差のみ)
//引数 	:なし
//返り値:なし
void set_motor_vol_side_wall(void)
{
	switch(get_side_wall_ctrl_mode()){
		case right:
			target_vol_sum = get_target_vol_sum_ctrl();	
			target_vol_diff = get_target_vol_diff_sidewall();
			break;
		
		case left:
			target_vol_sum = get_target_vol_sum_ctrl();	
			target_vol_diff = get_target_vol_diff_sidewall();
			break;

		case both_side:
			target_vol_sum = get_target_vol_sum_ctrl();
			target_vol_diff = rate_side_wall * get_target_vol_diff_sidewall()
								+(1 - rate_side_wall) * get_target_vol_diff_ctrl();
			break;

		case none: //両壁がないときは軌跡制御に
			target_vol_sum = get_target_vol_sum_ctrl();
			target_vol_diff = get_target_vol_diff_ctrl();
			break;
	}
}

//機能 	:モータ印可電圧初期化
//引数 	:なし
//返り値:なし
void clr_motor_vol(void)
{
	target_vol_r = 0;
	target_vol_l = 0;
}

//機能 	:各制御モードでの操作量履歴クリア
//引数 	:なし
//返り値:なし
void clr_operate_history(void)
{
	switch(ctrl_mode)
	{
		case trace:
			clr_frontwall_operate_history(); //前壁制御の操作履歴をクリア
			break;

		case front_wall:
			clr_trace_operate_history();			//軌跡制御の操作履歴をクリア
			break;
		
		case side_wall:
			switch(get_side_wall_ctrl_mode()){
				case none:
					break;
				default:
					adjust_trace_theta();
			}
			clr_frontwall_operate_history(); //前壁制御の操作履歴をクリア
			break;
	}
}

//機能	: 印加電圧の和、差から各モータに印加する電圧を決定する。
//引数	: なし
//返り値	: なし
void calc_motor_vol ( void )
{
	target_vol_r = (target_vol_sum + target_vol_diff)/2;
	target_vol_l = (target_vol_sum - target_vol_diff)/2;	
}

//機能 	:印可電圧を変調率に変換
//引数 	:なし
//返り値:なし
void calc_vol2duty ( void )
{

	/* バッテリー電圧とモータに印加する電圧から、duty[*0.1%]を算出	*/
	target_duty_r = target_vol_r / (Battery_GetVoltage()) * 1000;
	target_duty_l = target_vol_l / (Battery_GetVoltage()) * 1000;
}

//機能 	:非線形部は使わないように、変調率を調整
//引数 	:なし
//返り値:なし
//
void motor_duty_adjust (void)
{
	if( target_duty_r > 0){
		target_duty_r = target_duty_r + 40;  //40->30
	}
	else{
		target_duty_r = target_duty_r - 45; //45 ->35
	}

	if( target_duty_l > 0){
		target_duty_l = target_duty_l + 40;  //40->30
	}
	else{
		target_duty_l = target_duty_l - 45; //45 ->35
	}
}

//機能 	:モータに印可電圧を出力
//引数 	:なし
//返り値:なし
void set_motor_duty (void)
{
	set_duty_l(target_duty_l);
	set_duty_r(target_duty_r);
}

//機能	: 右モータの変調率を取得する
//引数	: なし
//返り値	: なし
int16_t get_target_duty_r ( void )
{
	return target_duty_r;
}

//機能	: 左モータの変調率を取得する
//引数	: なし
//返り値	: なし
int16_t get_target_duty_l ( void )
{
	return target_duty_l;
}