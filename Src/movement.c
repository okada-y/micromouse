//動作方法を記述予定(move_back)など



#include <stdio.h>
#include "param.h"
#include "target.h"
#include "movement.h"

static run_start run_first_flg = 0;		// 走行開始フラグ 0:走行開始時　1:それ以外

//機能	: 移動完了判断
//引数	: なし
//返り値	: 判断結果(0:未完,1:完了)
uint8_t move_comp_jud ( void )
{
    return (uint8_t)(ABS(get_target_length() - get_ideal_length()) < move_comp_th);
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

    /*半区画進むまで待機*/
    while (1)
    {
    	if(move_comp_jud())
    	{
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
	set_target_angle(counter_clockwise);
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
		half_acceleration();//半区画加速
	}
	if(run_first_flg == already)
	{
		constant_speed();//定速で一マス前進
	}
	run_first_flg = already;
	front_wall_calib_flg_clr();
}

//機能	: 右折
//引数	: なし
//返り値	: なし
void move_right (void)
{
	if(run_first_flg == start){
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_clk_90();		//m時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	if(run_first_flg == already){
		half_deceleration();//m半区画減速で中央に停止
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_clk_90();		//m時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	run_first_flg = already;
	front_wall_calib_flg_clr();
}

//機能	: 左折
//引数	: なし
//返り値	: なし
void move_left (void)
{
	if(run_first_flg == start){
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_conclk_90();	//m反時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	if(run_first_flg == already){
		half_deceleration();//m半区画減速で中央に停止
		if(front_calib_flg == 1){
			fornt_wall_calibrate();
		}
		turn_conclk_90();	//m反時計回りに90度回転
		half_acceleration();//m半区画加速
	}
	run_first_flg = already;
	front_wall_calib_flg_clr();
}

//機能	: 後進
//引数	: なし
//返り値	: なし
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
	if(run_first_flg == already){
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
	run_first_flg = already;
	front_wall_calib_flg_clr();
}