//動作方法を記述予定(move_back)など



#include <stdio.h>
#include "param.h"
#include "target.h"
#include "movement.h"
#include "adjust.h"
#include "exvol.h"
#include "mouse_state.h"
#include "main.h"

static run_start run_first_flg = 0;			// 走行開始フラグ 0:走行開始時　1:それ以外
static wall_flg	front_wall_flg = nowall;	//　前壁の有無フラグ
static wall_flg	right_wall_flg = nowall;	//　右壁の有無フラグ
static wall_flg	left_wall_flg = nowall;		//　左壁の有無フラグ


//機能	: 移動完了判断
//引数	: なし
//返り値	: 判断結果(0:未完,1:完了)
uint8_t move_comp_jud ( void )
{
    return (uint8_t)((get_target_length() - get_move_length()) < 0);
}

//機能	: 移動完了判断（停止時）
//引数	: なし
//返り値	: 判断結果(0:未完,1:完了)
uint8_t move_comp_jud_stop ( void )
{
    return (uint8_t)((get_target_length() - get_move_length()) 
							< get_target_move_speed() * get_target_move_speed()/(2 * move_accel));
}


//機能	: 回転完了判断
//引数	: なし
//返り値	: 判断結果(0:未完,1:完了)
uint8_t rotate_comp_jud ( void )
{
    return (uint8_t)(ABS(get_target_angle() - get_ideal_angle()) < rotate_comp_th);
}

//機能	: スタート時加速
//引数	: なし
//返り値	: なし
void start_acceleration (void)
{
	/*移動方向、加速モード設定*/
	set_direction_mode(forward_mode);
	set_accel_mode(acceleration);
    set_target_length(0.060286-0.045);

    /*半区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
	 	{
    		break;
    	}
    }
}

//機能	: 半分区画加速
//引数	: なし
//返り値	: なし
void half_acceleration (void)
{
	/*移動方向、加速モード設定*/
	set_direction_mode(forward_mode);
	set_accel_mode(acceleration);
    set_target_length(0.045);

    /*半区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
    	{
    		break;
    	}
    }
}


//機能	: 反区画減速
//引数	: なし
//返り値	: なし
void half_deceleration (void)
{
	/*移動方向、加速モード設定*/
	set_direction_mode(forward_mode);
	set_accel_mode(deceleration);
    set_target_length(0.045);
	set_speed_under_lim_flg(slow);
    /*半区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud_stop())
    	{
			set_speed_under_lim_flg(zero);
			HAL_Delay(100);
    		break;
    	}
    }
}

//機能	: 一区画定速
//引数	: なし
//返り値	: なし
void constant_speed (void)
{
	/*移動方向、加速モード設定*/
	set_direction_mode(forward_mode);
    set_accel_mode(acceleration);
    set_target_length(0.09);

    /*一区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
    	{
    		break;
    	}
    }
}

//機能	: 時計回りに90度回転
//引数	: なし
//返り値	: なし
void turn_clk_90 (void)
{
	/*回転方向設定*/
	set_rotation_mode(clockwise);
    set_target_angle(-PI/2);

    /*90度回転するまで待機*/
    while (1)
    {
    	if(rotate_comp_jud())
    	{
    		break;
    	}
    }

}

//機能	: 反時計回りに90度回転
//引数	: なし
//返り値	: なし
void turn_conclk_90 (void)
{
	/*回転方向設定*/
	set_rotation_mode(counter_clockwise);
    set_target_angle(PI/2);

    /*90度回転するまで待機*/
    while (1)
    {
    	if(rotate_comp_jud())
    	{
    		break;
    	}
    }

}

//機能	: 反時計回りに180度回転
//引数	: なし
//返り値	: なし
void turn_conclk_180 (void)
{
	/*回転方向設定*/
	set_rotation_mode(counter_clockwise);
    set_target_angle(PI);

    /*180度回転するまで待機*/
    while (1)
    {
    	if(rotate_comp_jud())//0.3deg手前まできたらブレイク
    	{
    		break;
    	}
    }

}

//機能	: 走行開始フラグのクリア
//引数	: なし
//返り値	: なし
void clr_run_first_flg (void)
{
	run_first_flg = start;
}

//機能	: 走行開始フラグセット
//引数	: なし
//返り値	: なし
void set_run_first_flg (void)
{
	run_first_flg = already;
}


//機能	: 壁の有無フラグのクリア
//引数	: なし
//返り値	: なし
void clr_wall_flg (void)
{
	front_wall_flg = nowall;
	right_wall_flg = nowall;
	left_wall_flg = nowall;
}

//機能	: 前壁フラグセット
//引数	: なし
//返り値	: なし
void set_front_wall_flg ( void )
{
	front_wall_flg = wall;
}

//機能	: 右壁フラグセット
//引数	: なし
//返り値	: なし
void set_rigth_wall_flg ( void )
{
	right_wall_flg = wall;
}

//機能	: 左壁フラグセット
//引数	: なし
//返り値	: なし
void set_left_wall_flg ( void )
{
	left_wall_flg = wall;
}


////////////////////////////////////////
/* 軌跡生成関数						*/
////////////////////////////////////////


//機能	: 前進
//引数	: なし
//返り値	: なし
void move_front (void)
{
	if(run_first_flg == start)
	{
		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
	}
	if(run_first_flg == already)
	{
		set_mode_ctrl(side_wall);
		constant_speed();//定速で一マス前進
		set_mode_ctrl(trace);
	}
	run_first_flg = already;
	clr_wall_flg();
}

//機能	: 右折
//引数	: なし
//返り値	: なし
void move_right (void)
{
	if(run_first_flg == start){
		if(front_wall_flg == wall){
			fornt_wall_calibrate();
		}
		turn_clk_90();		//時計回りに90度回転
		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
	}
	if(run_first_flg == already){
		half_deceleration();//半区画減速で中央に停止
		if(front_wall_flg == wall){
			fornt_wall_calibrate();
		}
		turn_clk_90();		//時計回りに90度回転
		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
	}
	run_first_flg = already;
	clr_wall_flg();
}

//機能	: 左折
//引数	: なし
//返り値	: なし
void move_left (void)
{
	if(run_first_flg == start){
		if(front_wall_flg == wall){
			fornt_wall_calibrate();
		}
		turn_conclk_90();	//m反時計回りに90度回転
		set_mode_ctrl(side_wall);
		half_acceleration();//m半区画加速
		set_mode_ctrl(trace);
	}
	if(run_first_flg == already){
		half_deceleration();//m半区画減速で中央に停止
		if(front_wall_flg == wall){
			fornt_wall_calibrate();
		}
		turn_conclk_90();	//m反時計回りに90度回転
		set_mode_ctrl(side_wall);
		half_acceleration();//m半区画加速
		set_mode_ctrl(trace);
	}
	run_first_flg = already;
	clr_wall_flg();
}

//機能	: 後進
//引数	: なし
//返り値	: なし
void move_back (void)
{
	if(run_first_flg == start){

		if(front_wall_flg == wall){
			fornt_wall_calibrate();
			if(right_wall_flg == wall){
				turn_clk_90();
				fornt_wall_calibrate();
				turn_clk_90();
			}
			else if(left_wall_flg == wall){
					turn_conclk_90();
					fornt_wall_calibrate();
					turn_conclk_90();
			}
			else {
				turn_conclk_180();	//反時計回りに180度回転
			}
		}
		else{
			turn_conclk_180();	//反時計回りに180度回転
		}
		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
	}
	if(run_first_flg == already){
		set_mode_ctrl(side_wall);
		half_deceleration();//m半区画減速で中央に停止
		set_mode_ctrl(trace);
		if(front_wall_flg == wall){
			fornt_wall_calibrate();
			if(right_wall_flg == wall){
				turn_clk_90();
				fornt_wall_calibrate();
				turn_clk_90();
			}
			else if(left_wall_flg == wall){
				turn_conclk_90();
				fornt_wall_calibrate();
				turn_conclk_90();
			}
			else {
				turn_conclk_180();	//反時計回りに180度回転
			}
		}
		else{
			turn_conclk_180();	//m反時計回りに180度回転
		}
		set_mode_ctrl(side_wall);
		half_acceleration();//半区画加速
		set_mode_ctrl(trace);
	}
	run_first_flg = already;
	clr_wall_flg();
}

