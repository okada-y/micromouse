/*
 * target.c
 *
 *  Created on: Aug 7, 2019
 *      Author: 岡田 泰裕
 */

#include "param.h"
#include "ir_sensor.h"
#include "target.h"


static float target_move_speed = 0;			//目標移動速度[m/s]
static float target_move_speed_fin = 0;		//目標終端速度[m/s]
static float target_rotation_speed = 0;		//目標角速度[rad/s]
static float target_move_accel = 0;			//目標移動加速度[m/s2]
static float target_rotation_accel = 0;		//目標角加速度[rad/s2]
static float target_length = 0;				//目標移動距離[m]
static float target_angle = 0;				//目標回転角度[rad]
static float ideal_length = 0;				//理想移動距離[m]
static float ideal_angle = 0;				//理想回転角度[rad]

static direction_mode move_dir_flg = forward_mode;			//移動方向フラグ　0:前進 1:後進
static rotation_mode rotation_dir_flg = counter_clockwise;	//回転方向フラグ　0:時計周り 1:反時計周り
static accel_mode accel_dir_flg = acceleration;		   		//加減速フラグ　　0:加速 1:減速

//機能	: traget.cの1msタスクのまとめ
//引数	: なし
//返り値	: なし
void target_1ms ( void )
{
	calc_target_move_accel();		//移動加速度更新
	calc_rotate_accel();			//回転角速度更新
	calc_target_move_speed();		//目標移動速度更新
	calc_target_rotation_speed();	//目標角速度更新
}

//機能	: 移動方向モードをセットする
//引数	: 移動方向モード(forward,back)
//返り値	: なし
void set_direction_mode ( direction_mode dmode )
{
	move_dir_flg = dmode;
}

//機能	: 回転方向モードをセットする
//引数	: 回転方向モード(counter_clocwise,clockwise)
//返り値	: なし
void set_rotation_mode ( rotation_mode rmode )
{
	rotation_dir_flg = rmode;
}

//機能	: 加速モードをセットする
//引数	: 加速モード(acceleration,deceleration)
//返り値	: なし
void set_accel_mode ( accel_mode amode )
{
	accel_dir_flg = amode;
}


//機能	: 目標移動速度[m/s]取得
//引数	: なし
//返り値	: 目標移動速度[m/s]
float get_target_move_speed ( void )
{
	return target_move_speed;
}

//機能	: 目標角速度[m/s]取得
//引数	: なし
//返り値	: 目標角速度[m/s]
float get_target_rotation_speed ( void )
{
	return target_rotation_speed;
}

//機能	: 目標移動距離取得
//引数	: なし
//返り値	: 目標移動距離
float get_target_length ( void )
{
	return target_length;
}

//機能	: 理想移動距離取得
//引数	: なし
//返り値	: 理想移動距離
float get_ideal_length ( void )
{
	return ideal_length;
}

//機能	: 目標角度取得
//引数	: なし
//返り値	: 目標角度
float get_target_angle ( void )
{
	return target_angle;
}

//機能	: 理想角度取得
//引数	: なし
//返り値	: 理想角度
float get_ideal_angle ( void )
{
	return ideal_angle;
}



//機能	: 目標移動距離更新
//引数	: 加算する移動距離[m]
//返り値	: なし
void set_target_length ( float set_length )
{
	target_length += set_length;
}

//機能	: 目標角度更新
//引数	: 加算する角度[rad]
//返り値	: なし
void set_target_angle ( float set_angle )
{
	target_angle += set_angle;
}


//機能	: 移動加速度の更新
//引数	: なし
//返り値	: なし
//備考	: 現在移動距離と目標移動距離に応じて、加速度をスイッチ
//		: 1msタスク
void calc_target_move_accel(void)
{

	float tm_deccel_length = 0;

	switch(accel_dir_flg)
	{
		case acceleration: //加速モード時、常に加速(リミットに引っ掛かり続ける)
			switch(move_dir_flg)
			{
			case forward_mode:
				target_move_accel = 1 * move_accel;
				break;
			case backward_mode:
				target_move_accel = -1 * move_accel;
				break;
			}
			break;

		case deceleration:
			switch(move_dir_flg)
			{
			case forward_mode:
				/* 減速に必要な距離算出*/
				tm_deccel_length = ((target_move_speed*target_move_speed)-(target_move_speed_fin*target_move_speed_fin)) / (2*move_accel);
				
				//減速開始前
				if(ideal_length < (target_length - tm_deccel_length))
				{
					target_move_accel =  1 * move_accel; //リミットにかかるまで加速
				}
				else//減速開始後
				{																	
					target_move_accel = -1 * move_accel;
				}
				break;

			case backward_mode:
				/* 加速に必要な距離算出*/
				tm_deccel_length = ((target_move_speed*target_move_speed)-(target_move_speed_fin*target_move_speed_fin)) / (2*move_accel);
				
				//加速開始前
				if(ideal_length > (target_length + tm_deccel_length))
				{			
					target_move_accel = -1 * move_accel; //リミットにかかるまで減速
				}
				else//加速開始後
				{												
					target_move_accel = 1 * move_accel;
				}
				break;
			}
			break;
	}

}

//機能	: 角加速度の更新
//引数	: なし
//返り値	: なし
//備考	: 現在角度と目標角度に応じて、加速度をスイッチ
//		: 1msタスク
void calc_rotate_accel(void)
{

	float tm_deccel_angle = 0;

	switch(rotation_dir_flg)
	{
	case counter_clockwise:
		/*減速に必要な角度算出*/
		tm_deccel_angle = 0.5 * target_angle * target_angle / rotat_accel;
		
		//減速開始前
		if(ideal_angle < (target_angle - tm_deccel_angle))
		{			
			target_rotation_accel =  1 * rotat_accel;
		}
		else//減速開始後
		{																	
			target_rotation_accel = -1 * rotat_accel;
		}
		break;

	case clockwise:
		/*加速に必要な角度算出*/
		tm_deccel_angle = 0.5 * target_angle * target_angle / rotat_accel;
		
		//加速開始前
		if(ideal_angle > (target_angle + target_angle))
		{
			target_rotation_accel =  -1 * rotat_accel;
		}
		else//加速開始後
		{
			target_rotation_accel = 1 * rotat_accel;
		}
		break;
	}

}


//機能	: 目標速度更新
//引数	: なし
//返り値	: なし
//備考	:1msタスク
void calc_target_move_speed(void)
{

	/*m加速処理*/
	target_move_speed += target_rotation_accel * 0.001; //1ms分の加速加算

	/*limit処理*/
	switch(move_dir_flg)
	{
		case forward_mode:

			if (target_move_speed < 0)	//下限スピード処理
					{
						target_move_speed = 0;
					}
					else if (target_move_speed > move_speed_max)//上限スピード処理
					{
						target_move_speed = move_speed_max;
					}
			break;

		case backward_mode:
			if (target_move_speed > 0)	//上限スピード処理
				{
					target_move_speed = 0;
				}
				else if (target_move_speed < -1 * move_speed_max)//下限スピード処理
				{
					target_move_speed = -1 * move_speed_max;
				}
			break;
	}

	//理想移動距離算出
	ideal_length += target_move_speed * 0.001;

}

//機能	: 目標角速度を更新する
//引数	: なし:
//返り値	: なし
//備考	:1msタスク
void calc_target_rotation_speed ( void )
{
	target_rotation_speed += target_rotation_accel * 0.001; //1ms分の角加速度加算

	switch(rotation_dir_flg)
		{
			case counter_clockwise:

				if (target_rotation_speed < 0)	//m 下限スピード処理
						{
							target_rotation_speed = 0;
						}
						else if (target_rotation_speed > rotat_speed_max)//上限スピード処理
						{
							target_rotation_speed = rotat_speed_max;
						}
				break;

			case clockwise:
				if (target_rotation_speed > 0)	//m 上限スピード処理
					{
						target_rotation_speed = 0;
					}
					else if (target_rotation_speed < -1 * rotat_speed_max)//下限スピード処理
					{
						target_rotation_speed = -1 * rotat_speed_max;
					}
				break;
		}

	ideal_angle += target_rotation_speed * 0.001;

}


