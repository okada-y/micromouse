#include "param.h"
#include "target.h"
#include "control.h"



static float target_vol_r_cont = 0;		//右タイヤの操作量[ duty % ]
static float target_vol_l_cont = 0;		//左タイヤの操作量[ duty % ]


void Operation_amount_calc(void)
{
//	static float target_acceleration_m = 0;		//m 目標並進方向加速度[ m/s^2]
//	static float target_acceleration_w = 0;		//m 目標並進方向加速度[ rad/s^2]

//	static int8_t offset_flg_r = 0;				//右モータ電圧オフセット電圧フラグ
//	static int8_t offset_flg_l = 0;				//右モータ電圧オフセット電圧フラグ

	static float target_vol_r = 0;	 			//m 右モータの目標電圧[ V]
	static float target_vol_l = 0; 				//m 左モータの目標電圧[ V]

//	static float ff_target_vol_r = 0;			//ff制御の右モータ電圧[V]
//	static float ff_target_vol_l = 0;			//ff制御の左モータ電圧[V]

	static float fb_target_vol_r = 0;			//fb制御の右モータ電圧[V]
	static float fb_target_vol_l = 0;			//fb制御の右モータ電圧[V]

	static float speed_m_err = 0; 				//m 並進方向の目標速度偏差
	static float speed_w_err = 0;				//m 目標角速度偏差
	static float speed_w_err_D = 0;				//m 角速度偏差微分
	static float speed_w_err_prev = 0;			//m 前回角速度偏差
	static float speed_w_err_D_prev = 0;		//m 前回角速度偏差微分
	static float speed_m_err_I = 0; 			//m 速度和の偏差の積分
	static float speed_w_err_I = 0;				//m 速度差の偏差の積分



	static float speed_m_err_PID = 0; 			//m　並進方向速度差のPID量
	static float speed_w_err_PID = 0;			//m 回転方向速度差のPID量

	/*m 壁補正用*/
    double l_front_sensor_r = 0; 		//m 右前センサ値のバッファ
    double l_front_sensor_l = 0; 		//m 左前センサ値のバッファ
    double l_front_sensor_r_err = 0;   //m 右前センサの偏差
    double l_front_sensor_l_err = 0;	//m 左前センサの偏差
    double l_front_sensor_m_err = 0;   //m センサの偏差の和
    double l_front_sensor_w_err = 0;	//m センサの偏差の差
    static double l_front_sensor_m_err_I = 0;   	//m 偏差和積分
    static double l_front_sensor_w_err_I = 0;   	//m 偏差差積分
    static double l_front_sensor_m_err_prev = 0;   	//m 前回偏差和
    static double l_front_sensor_w_err_prev = 0;   	//m 前回偏差差
    static double l_front_sensor_m_D_prev = 0;   	//m 前回偏差和微分
    static double l_front_sensor_w_D_prev = 0;   	//m 前回偏差差微分

    double l_front_sensor_m_err_P = 0;
    double l_front_sensor_w_err_P = 0;
    double l_front_sensor_m_err_D = 0;
    double l_front_sensor_w_err_D = 0;

    double l_front_sensor_m_PID = 0;
    double l_front_sensor_w_PID = 0;



    /*m 偏差取得*/
    speed_m_err = target_speed_m - g_ave_speed_m;		//m 目標スピードー実速度(エンコーダから)[m/s]
    speed_w_err = target_speed_w - IMU_GetGyro_Z();		//m 目標角速度ー実速度（IMUから）[rad/s]

    /*偏差積分*/
    speed_m_err_I = speed_m_err_I + speed_m_KI*0.001*speed_m_err;
    speed_w_err_I = speed_w_err_I + speed_w_KI*0.001*speed_w_err;

    speed_m_err_PID = speed_m_KP * speed_m_err + speed_m_err_I;
    speed_w_err_PID = speed_w_KP * speed_w_err + speed_w_err_I;

    /*FB項演算*/
    fb_target_vol_r = speed_m_err_PID + speed_w_err_PID;
    fb_target_vol_l = speed_m_err_PID - speed_w_err_PID;

    /*m 印加電圧算出*/
    target_vol_r = (fb_gain * fb_target_vol_r);
    target_vol_l = (fb_gain * fb_target_vol_l);

    /*m 前回偏差更新*/
    speed_w_err_prev = speed_w_err;			//m 前回角速度偏差
    speed_w_err_D_prev = speed_w_err_D;			//m 前回角速度偏差微分

	

}

