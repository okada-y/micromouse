#ifndef PARAM_H_
#define PARAM_H_

//定数
#define G					(9.80665f)					//重量加速度[m/s^2]
#define PI					(3.1415926f)				//円周率
#define SQRT2				(1.41421356237f)			//ルート2
#define SQRT3				(1.73205080757f)			//ルート3
#define SQRT5				(2.2360679775f)				//ルート5
#define SQRT7				(2.64575131106f)			//ルート7

//マクロ関数
#define DEG2RAD(x)			(((x)/180.0f)*PI)			                //度数法からラジアンに変換
#define RAD2DEG(x)			(180.0f*((x)/PI))			                //ラジアンから度数法に変換
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))	//aとbの入れ替え
#define ABS(x) 				((x) < 0 ? -(x) : (x))		                // 絶対値
#define SIGN(x)				((x) < 0 ? -1 : 1)			                // 符号
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		                // 2つのうち大きい方を返します
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		                // 2つのうち小さい方を返します
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))


/////////////////////////////////////
//           パラメータ             //
/////////////////////////////////////

/*module_test*/
#define log_count_lim		(20000)	//データ取得する期間[ms]
#define log_count_step		(20)	    //データを取得する時間間隔[ms]

#define DATA_DEFAULT              //測定モード１
//#define DATA_SIDE                 //測定モード２
//#define DATA_MAZE                 //測定モード３

//maze
#define x_size              (10)     //x軸方向の壁(縦壁)の枚数+1
#define y_size              (10)     //y軸方向の壁(横壁)の枚数+1
#define g_size              (1)     //ゴールのマスの数
#define goal_cordinate      {1, 7, 8, 8, 3, 3, 4, 4, 4,\
                             9, 11, 10, 11, 2, 3, 1, 2, 3}
                              //一行目、ゴールのx座標
                              //二行目、ゴールのy座標
//ir
#define front_th            (50) //前壁有無判定の閾値
#define right_th            (100) //右壁有無判定の閾値
#define left_th             (100) //左壁有無判定の閾値                             

//mode
#define mode_count_up_th 	(0.10f)     //モードカウントアップの右タイヤ速度閾値
#define mode_count_down_th 	(-0.10f)    //モードカウントダウンの右タイヤ速度閾値
#define mode_stanby_th 		(0.10f)     //モードカウントアップの左タイヤ速度閾値

//mouse_state
#define ave_num 			(10)	    //速度の移動平均フィルタの長さ
#define Tire_diameter		(0.01266698f)//タイヤの直径    

//target
#define move_accel          (1.5f)      //移動加速度[m/ss]
#define rotat_accel         (3*PI)      //角加速度[rad/ss]
#define move_speed_max      (0.3f)      //最大移動速度[m/s]
#define rotat_speed_max     (3*PI)      //最大角速度[rad/s]
#define move_speed_slow     (0.03)      //スロー走行時の速度[m/s]

//control
#define move_speed_P		(20.674f)	//移動速度制御のPゲイン
#define move_speed_I		(100.4724f)	//移動速度制御のIゲイン
#define rotate_speed_P		(0.34802f)	//角速度制御のPゲイン
#define rotate_speed_I		(9.1729f)	//角速度制御のIゲイン

//movement
#define move_comp_th        (0.001f)   //移動完了の閾値
#define rotate_comp_th      (0.001f)    //回転完了の閾値

//adjust
#define front_sensor_r_ref      (0.0107f)	        //前壁補正時の右前距離目標値(1cm)
#define front_sensor_l_ref      (0.0088)	        //前壁補正時の左前距離目標値(1cm)
#define chassis_width           (0.036787f)         //シャシー幅
#define front_sensor_move_KP	(50.412292054546f)	//前壁距離のPゲイン
#define front_sensor_move_KI	(26.2444668681538f)	//前壁距離のIゲイン
#define front_sensor_move_KD	(3.28431998107001f)	//前壁距離のDゲイン
#define front_sensor_move_fil	(23.9567963129851f)	//前壁距離フィルタ係数
#define front_sensor_rotate_KP	(6.40482660641506f)	//前壁角度のPゲイン
#define front_sensor_rotate_KI	(2.52037335129739f)	//前壁角度のIゲイン
#define front_sensor_rotate_KD	(0.206728439324594f)//前壁補正のDゲイン
#define front_sensor_rotate_fil	(44.5570260812328f)	//前壁角度フィルタ係数
#define front_sensor_th     	(0.0002)			//前壁補正時の補正閾値(0.3mm)
#define calib_tim               (200)				//前壁補正時間(100ms)
#define calib_tim_lim           (500)              //前壁補正最大時間

#define side_sensor_th          (0.035)               //横壁センサ値の閾値[m]
#define side_sensor_th_add      (0.02)               //横壁センサ値の閾値変化量[m]
#define side_sensor_diff_th     (0.00007)           //横壁センサ変化量の閾値
#define side_wall_P             (200)               //横壁制御　Pゲイン
#define side_wall_D             (1)                //横壁制御　Dゲイン
#define side_wall_fil           (294.083904215522)  //横壁制御　フィルタ係数

#define ir_diff_ave_num         (10)                //IRセンサの変動値の移動平均期間

//imu
#define REFFERENCE_NUM		(1000)		//何回の平均をもってジャイロのリファレンス電圧とするか
#define GYRO_Z_SIGN			(-1.f)		//ジャイロの出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define GYRO_Z_SENSITIVITY	(16.77f)	
#define ACCEL_X_SIGN		(1.f)		//加速度計の出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define ACCEL_X_SENSITIVITY	(4096.f)

//encorder
#define pinion_gear_num		(9.0f)								//ピニオンギヤ数
#define wheel_gear_num		(37.0f)								//ホイールギア数
#define gear_rate			(wheel_gear_num/pinion_gear_num)	//変速比（ギア比）

//exvol
#define rate_side_wall      (0.3)                 //壁制御時の壁制御による印加電圧の割合

//motor
#define MOT_DUTY_MIN	(30)						//モータの最低Duty
#define MOT_DUTY_MAX	(300)						//モータの最大Duty



#endif /* PARAM_H_*/