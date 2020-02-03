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
    return (uint8_t)((get_target_length() - get_ideal_length()) < comp_th);
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
    	if(ABS(get_target_length() - get_ideal_length()) < 0.0002)//0.2mm手前まできたらブレイク
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
    	if(ABS(get_target_length() - get_ideal_length()) < 0.0002)//0.2mm手前まできたらブレイク
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
	/*m移動方向、加速モード設定*/
	set_direction_mode(forward_mode);
	set_accel_mode(deceleration);
    set_target_length(0.045);

    /*m 半区画進むまで待機*/
    while (1)
    {
    	if(ABS(get_target_length() - get_ideal_length()) < 0.0002)//0.2mm手前まできたらブレイク
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
	/*m移動方向、加速モード設定*/
	set_direction_mode(forward_mode);
    set_accel_mode(acceleration);
    set_target_length(0.09);

    /*m 一区画進むまで待機*/
    while (1)
    {
    	if(ABS(get_target_length() - get_ideal_length()) < 0.0002)//0.1mm手前まできたらブレイク
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