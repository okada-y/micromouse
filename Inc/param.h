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

//mode
#define mode_count_up_th 	(0.10f)     //モードカウントアップの右タイヤ速度閾値
#define mode_count_down_th 	(-0.10f)    //モードカウントダウンの右タイヤ速度閾値
#define mode_stanby_th 		(0.10f)     //モードカウントアップの左タイヤ速度閾値

//mouse_state
#define ave_num 			(10)	    //速度の移動平均フィルタの長さ

//target
#define move_accel          (1.5f)      //移動加速度[m/ss]
#define rotat_accel         (2*PI)      //角加速度[rad/ss]
#define move_speed_max      (0.3f)      //最大移動速度[m/s]
#define rotat_speed_max     (2*PI)      //最大角速度[rad/s]

//control
#define move_speed_P		(2.674f)	//移動速度制御のPゲイン
#define move_speed_I		(39.4724f)	//移動速度制御のIゲイン
#define rotate_speed_P		(0.34802f)	//角速度制御のPゲイン
#define rotate_speed_I		(9.1729f)	//角速度制御のIゲイン

//movement
#define comp_th             (0.0002f)   




#endif /* PARAM_H_*/