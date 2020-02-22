#include "param.h"
#include "target.h"
#include "control.h"
#include "mouse_state.h"


static float move_speed_err_I = 0; 			//移動速度偏差積分
static float rotate_speed_err_I = 0;	    //角速度偏差積分

static float target_vol_sum_ctrl = 0;		//右タイヤの操作量[ duty % ]
static float target_vol_diff_ctrl = 0;		//左タイヤの操作量[ duty % ]

//機能	: 軌道制御によるモータ印加電圧の和を取得する
//引数	: なし
//返り値	: 軌道制御による右モータ印加電圧
float get_target_vol_sum_ctrl ( void )
{
    return target_vol_sum_ctrl;
}

//機能	: 軌道制御によるモータ印加電圧の差を取得する
//引数	: なし
//返り値	: 軌道制御による左モータ印加電圧
float get_target_vol_diff_ctrl ( void )
{
    return target_vol_diff_ctrl;
}

//機能	: 軌道制御により、左右のモータ印加電圧を計算する
//引数	: なし
//返り値	: なし
//備考  : 1msタスク
void calc_motor_vol_ctrl(void)
{
	float move_speed_err = 0; 			        //移動速度偏差
	float rotate_speed_err = 0;	        		//角速度偏差

	float move_speed_err_PI = 0; 		//移動速度偏差によるPIコントローラ出力
	float rotate_speed_err_PI = 0;		//角速度偏差によるPIコントローラ出力

    static float post_target_rotation_speed = 0; //前回の回転速度目標値
    float target_rotation_accel = 0;        //目標回転角加速度
    float rotate_FF = 0;   //FFコントローラによる電圧差出力

    /*FB制御*/
    /*偏差取得*/
    move_speed_err = get_target_move_speed() - get_move_speed_ave();
    rotate_speed_err = get_target_rotation_speed() - get_rotation_speed();

    /*偏差積分*/
    move_speed_err_I = move_speed_err_I + move_speed_I*0.001*move_speed_err;
    rotate_speed_err_I = rotate_speed_err_I + rotate_speed_I*0.001*rotate_speed_err;

    /*PIコントローラ出力計算*/
    move_speed_err_PI = move_speed_P * move_speed_err + move_speed_err_I;
    rotate_speed_err_PI = rotate_speed_P * rotate_speed_err + rotate_speed_err_I;


    

    /*FF制御*/
    target_rotation_accel = (get_target_rotation_speed() - post_target_rotation_speed) * Sampling_cycle; //目標回転角加速度更新
    rotate_FF = ff_gain_a_w * target_rotation_accel + ff_gain_v_w * get_target_rotation_speed();

    

    /*モータ印加電圧計算*/
    target_vol_sum_ctrl = move_speed_err_PI;
    target_vol_diff_ctrl = ff_rate_w * rotate_FF  +  (1.0 - ff_rate_w) * rotate_speed_err_PI;

    /*パラメータ更新*/
    post_target_rotation_speed = get_target_rotation_speed();

}

//機能	: 軌跡制御の操作履歴クリア
//引数	: なし
//返り値	: なし
void clr_trace_operate_history ( void )
{
    move_speed_err_I = 0; 		//移動速度偏差積分
    rotate_speed_err_I = 0;	    //角速度偏差積分
}

//機能	: 軌跡制御の角度履歴フィルタ
//引数	: なし
//返り値	: なし
void adjust_trace_theta ( void )
{
    rotate_speed_err_I = 0.99 * rotate_speed_err_I;	    //角速度偏差積分
}