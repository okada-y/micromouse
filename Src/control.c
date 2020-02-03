#include "param.h"
#include "target.h"
#include "control.h"
#include "mouse_state.h"

static float target_vol_r_cont = 0;		//右タイヤの操作量[ duty % ]
static float target_vol_l_cont = 0;		//左タイヤの操作量[ duty % ]

//機能	: 軌道制御による右モータ印加電圧を取得する
//引数	: なし
//返り値	: 軌道制御による右モータ印加電圧
float get_target_vol_r_cont ( void )
{
    return target_vol_r_cont;
}

//機能	: 軌道制御による左モータ印加電圧を取得する
//引数	: なし
//返り値	: 軌道制御による左モータ印加電圧
float get_target_vol_l_cont ( void )
{
    return target_vol_l_cont;
}

//機能	: 軌道制御により、左右のモータ印加電圧を計算する
//引数	: なし
//返り値	: なし
//備考  : 1msタスク
void calc_motor_vol_cont(void)
{
	static float move_speed_err = 0; 			//移動速度偏差
    static float move_speed_err_I = 0; 			//移動速度偏差積分
	static float rotate_speed_err = 0;			//角速度偏差
	static float rotate_speed_err_I = 0;	    //角速度偏差積分

	static float move_speed_err_PI = 0; 		//移動速度偏差によるPIコントローラ出力
	static float rotate_speed_err_PI = 0;		//角速度偏差によるPIコントローラ出力

    /*偏差取得*/
    move_speed_err = get_target_move_speed() - get_move_speed_ave();
    rotate_speed_err = get_target_rotation_speed() - get_rotation_speed();

    /*偏差積分*/
    move_speed_err_I = move_speed_err_I + move_speed_I*0.001*move_speed_err;
    rotate_speed_err_I = rotate_speed_err_I + rotate_speed_I*0.001*rotate_speed_err;

    /*PIコントローラ出力計算*/
    move_speed_err_PI = move_speed_P * move_speed_err + move_speed_err_I;
    rotate_speed_err_PI = rotate_speed_P * rotate_speed_err + rotate_speed_err_I;

    /*モータ印加電圧計算*/
    target_vol_r_cont = move_speed_err_PI + rotate_speed_err_PI;
    target_vol_l_cont = move_speed_err_PI - rotate_speed_err_PI;
}

