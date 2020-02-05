//軌道制御、軌道補正から得た印加電圧から、
//モータに出力する印加電圧を決定する。

#include <stdio.h>
#include "main.h"
#include "exvol.h"
#include "motor.h"
#include "control.h"
#include "adjust.h"

typedef enum {
	trace = 0,
	front_wall = 1,
	side_wall = 2,
} ctrl_mode_num;

static ctrl_mode = trace;

//機能 	:メインモード処理
//引数 	:なし
//返り値:なし
void set_mode_ctrl(ctrl_mode_num mode_num)
{
	ctrl_mode = mode_num;
}

//機能 	:制御モード番号に従い、電圧を印可
//引数 	:なし
//返り値:なし
void set_motor_vol(void))
{
	switch (ctrl_mode)
	{
	case trace:
		set_motor_vol_trace()
		break;
	
	case front_wall:
		set_motor_vol_front_wall()
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

}

//機能 	:前壁制御時の印可電圧出力
//引数 	:なし
//返り値:なし
void set_motor_vol_front_wall(void)
{

}

//機能 	:横壁制御時の印可電圧出力
//引数 	:なし
//返り値:なし
void set_motor_vol_side_wall(void)
{

}

//機能 	:モータ印可電圧初期化
//引数 	:なし
//返り値:なし
void clr_motor_vol(void)
{
	
}

//機能 	:各制御モードでの操作量履歴クリア
//引数 	:制御モード番号
//返り値:なし
//target.c adjust.cにおいて操作履歴クリア関数を作り、呼び出すこと
void clr_operate_history(ctrl_mode_num mode_num)
{

}

//機能 	:非線形部は使わないように、印可電圧を調整
//引数 	:なし
//返り値:なし
//
void motor_vol_adjust (void)
{
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

//機能 	:モータに印可電圧を出力
//引数 	:なし
//返り値:なし
//ここでセットデューティー