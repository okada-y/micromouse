/*
 * index.h
 *
 *  Created on: Jul 14, 2019
 *      Author: Yasuhiro Okada
 */

#ifndef INDEX_H_
#define INDEX_H_


#include <stdio.h>
#include <math.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/*extern 宣言*/
extern uint8_t mode_number;	//mode番号
extern uint8_t mode_number_int; //m モードごとの割り込みフラグ
extern uint8_t mode_count;  //modeカウンタ

extern uint8_t stanby_mode; 		//1=stanby 0:not_stanby
extern float speed_r;				//	右タイヤ速度[m/s]
extern float speed_r_max;			// m最大右タイヤ速度（デバッグ用
extern float speed_r_min;			// m最小右タイヤ速度（デバッグ用
extern float operation_amount_r; 	//m 右タイヤ操作量
extern float speed_l;				//	左タイヤ速度[m/s]
extern float speed_l_max;			// m最大左タイヤ速度（デバッグ用
extern float speed_l_min;			// m最小左タイヤ速度（デバッグ用
extern float operation_amount_l; 	//m 左タイヤ操作量
extern float speed_m;				//m	本体並進方向速度[m/s]
extern float g_ave_speed_m; 		//m 本体並進方向速度の移動平均[m/s]
extern float g_ave_accel_m; 		//m 本体並進方向速度の移動平均[m/s]
extern float speed_rad;				//m エンコーダから算出される角速度[rad/s]
extern float target_speed_m;  		//m 並進方向の目標速度[m/s]
extern float target_speed_w;		//m 目標角速度[rad/s]
extern float target_distance_m; 	//m 目標距離[m]
extern float target_distance_w; 	//m 目標角度[rad]
extern float ideal_distance_m; 		//m 理想の現在移動距離[m]
extern float ideal_distance_w; 		//m 理想の現在角度[rad]
extern float real_distance_m;		//m 現在移動距離[m]
extern float real_distance_w;		//m 現在角度[rad]

extern float delta_angle_motor_r;	//m 右モータの回転数[rpm]
extern float delta_angle_motor_l;	//m 左モータの回転数[rpm]

extern int16_t g_duty_r;			//m 右モータのduty[*0.1%]
extern int16_t g_duty_l;			//m 左モータのduty[*0.1%]

extern uint8_t move_dir_flg;			//m　移動方向フラグ　0:前進 1:後進
extern uint8_t rotation_dir_flg;		//m　回転方向フラグ　0:時計周り 1:反時計周り
extern uint8_t accel_dir_flg;		    //m　加速方向フラグ　0:加速 1:減速
extern uint8_t run_first_flg;			//m 走行開始フラグ 0:走行開始時　1:それ以外
extern uint8_t correction_mode;			//m 補正モードフラグ　0：目標速度　1:前壁センサ値目標値

extern short wall_sensor_front;		//m 前壁センサ値
extern short wall_sensor_front_th;	//m 前壁センサ閾値
extern short wall_sensor_right;		//m　右壁センサ値
extern short wall_sensor_right_th;	//m　右壁センサ閾値
extern short wall_sensor_left;		//m　左壁センサ値
extern short wall_sensor_left_th;	//m　左壁センサ閾値

extern uint16_t fornt_wall_calibrate_tim; //m 前壁補正用カウンタ

extern uint8_t front_calib_flg; //m 前壁補正用フラグ
extern uint8_t right_calib_flg; //m 前壁補正用フラグ
extern uint8_t left_calib_flg; 	//m 前壁補正用フラグ

/* m便利な定数群 */
#define G					(9.80665f)					// m重量加速度[m/s^2]
#define PI					(3.1415926f)				// m円周率
#define SQRT2				(1.41421356237f)			// mルート2
#define SQRT3				(1.73205080757f)			// mルート3
#define SQRT5				(2.2360679775f)				// mルート5
#define SQRT7				(2.64575131106f)			// mルート7

/* m便利なマクロ関数群 */
#define DEG2RAD(x)			(((x)/180.0f)*PI)			// m度数法からラジアンに変換
#define RAD2DEG(x)			(180.0f*((x)/PI))			// mラジアンから度数法に変換
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))	//aとbの入れ替え
#define ABS(x) 				((x) < 0 ? -(x) : (x))		// m絶対値
#define SIGN(x)				((x) < 0 ? -1 : 1)			// m符号
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		// 2つのうち大きい方を返します
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		// 2つのうち小さい方を返します
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))


/*m速度算出まわりの定数*/
#define m_ave_num			(10)	//m 並進方向速度、加速度の移動平均個数

/*mモード選択の閾値*/
#define mode_count_up_th 	(0.10f)//m モードカウントアップの右タイヤ速度閾値
#define mode_count_down_th 	(-0.10f)//m モードカウントダウンの右タイヤ速度閾値
#define mode_stanby_th 		(0.10f)//m モードカウントアップの左タイヤ速度閾値

/*m データ取得周りの定数*/
#define log_count_lim		(5000)	//m　データ取得する期間[ms]
#define log_count_step		(5)		//m データを取得する時間間隔[ms]

/*mハード周りの定数*/
#define mouse_weight		(0.017f)							//mouse 本体重さ[kg]
#define mouse_inertia		(0.000002754f)						//mouse 本体慣性モーメント[kg・m^2] (m マウスを円板として近似=(0.000002754f)

#define Tire_diameter		(0.0128365f)						//tire直径[ m]
#define chassis_width		(0.036787f)							//m シャシー幅[ m]
#define pinion_gear_num		(9.0f)								//m ピニオンギヤ数
#define wheel_gear_num		(37.0f)								//mホイールギア数
#define gear_rate			(wheel_gear_num/pinion_gear_num)	//m変速比（ギア比）
//
//#define counter_v_cons		(0.00059205638f)					//m 逆起電圧定数[ V・s/rad]
//#define torque_cons			(0.00059205638f)					//m トルク定数[ N・m/A]
//
//#define motor_ohm			(4.5f)						//m モータ抵抗[Ω]

/*m制御周りの定数*/
#define speed_m_KP			(2.674f)				//m 並進方向のPゲイン
#define speed_m_KI			(39.4724f)				//m 並進方向のIゲイン
#define speed_w_KP			(0.34802f)				//m 回転方向のPゲイン
#define speed_w_KI			(9.1729f)				//m 回転方向のIゲイン
#define speed_w_KD			(0.0f)					//m 回転方向のDゲイン
#define speed_w_fil			(207.632819982745f)		//m 回転方向のフィルタ係数

//#define speed_w_b			(0.020141f)				//m　回転方向のFF重みづけ
//
//#define speed_w_KP			(0.23448f)			//m 回転方向のPゲイン
//#define speed_w_KI			(3.6716f)			//m 回転方向のIゲイン
//#define speed_w_b			(0.020141140849473)		//m　回転方向のFF重みづけ


//#define ff_m_a_gain			(0.1188f)					//m 並進方向の加速度項のゲイン
//#define ff_m_v_gain			(0.4771f)					//m 並進方向の速度項のゲイン
//#define ff_m_f_gain			(0.0000f)					//m 並進方向の摩擦項のゲイン(0.1447f)(オフセット電圧にて対処)
//
//#define ff_w_a_gain			(0.0018f)					//m 回転方向の加速度項のゲイン
//#define ff_w_v_gain			(0.0144f)					//m 回転方向の速度項のゲイン
//#define ff_w_f_gain			(0.0000f)					//m 回転方向の摩擦項のゲイン(0.1366f)(オフセット電圧にて対処)



#define ff_gain				(1.0f)						//m FF項のゲイン
#define fb_gain				(1.0f)						//m FB項のゲイン

#define offset_voltage		(0.10f)						//m　摩擦項のかわり

//前壁補正関連
#define front_sensor_r_ref  (0.0115f)					//m 前壁補正時の目標値(1cm)
#define front_sensor_l_ref  (0.0095f)					//m 前壁補正時の目標値(1cm)
#define front_sensor_m_KP	(124.412292054546f)	 			//m 前壁距離のPゲイン
#define front_sensor_m_KI	(26.2444668681538f)				//m 前壁距離のIゲイン
#define front_sensor_m_KD	(3.28431998107001f)				//m 前壁距離のDゲイン
#define front_sensor_m_fil	(23.9567963129851f)				//m 前壁距離フィルタ係数
#define front_sensor_w_KP	(6.40482660641506f)	 			//m 前壁角度のPゲイン
#define front_sensor_w_KI	(2.52037335129739f)				//m 前壁角度のIゲイン
#define front_sensor_w_KD	(0.206728439324594f)				//m 前壁補正のDゲイン
#define front_sensor_w_fil	(44.5570260812328f)				//m 前壁角度フィルタ係数


#define front_sensor_th  	(0.0002)				//m 前壁補正時の補正閾値(0.3mm)
#define calib_tim           (100)					//m 前壁補正時間(100ms)


/*m 目標速度算出周りの定数 */
#define speed_m_max 		(0.3f)						//m 最大速度[m/s]
#define speed_w_max 		(2*PI)						//m 最大角速度[rad/s]
#define accelation_m		(1.5f)						//m 加速度[m/s^2]
#define accelation_w		(2*PI)						//m 角加速度[rad/s^2]

/* LED関数群 */
#define LED_D2_ON()			HAL_GPIO_WritePin( LED2_GPIO_Port, 	 LED2_Pin,	GPIO_PIN_SET)		// D2のLEDを点灯する
#define LED_D2_OFF()		HAL_GPIO_WritePin( LED2_GPIO_Port,	 LED2_Pin,	GPIO_PIN_RESET)		// D2のLEDを消灯する
#define LED_D2_TOGGLE()		HAL_GPIO_TogglePin(LED2_GPIO_Port,	 LED2_Pin)						// mこの関数を呼ぶたびにD2のLEDの点灯と消灯を切り替える
#define LED_D3_ON()			HAL_GPIO_WritePin( LED3_GPIO_Port, 	 LED3_Pin,	GPIO_PIN_SET)		// D3のLEDを点灯する
#define LED_D3_OFF()		HAL_GPIO_WritePin( LED3_GPIO_Port, 	 LED3_Pin,  GPIO_PIN_RESET)		// D3のLEDを消灯する
#define LED_D3_TOGGLE()		HAL_GPIO_TogglePin(LED3_GPIO_Port, 	 LED3_Pin)						// mこの関数を呼ぶたびにD3のLEDの点灯と消灯を切り替える
#define LED_D4_ON()			HAL_GPIO_WritePin( LED4_GPIO_Port,   LED4_Pin,	GPIO_PIN_SET)		// D4のLEDを点灯する
#define LED_D4_OFF()		HAL_GPIO_WritePin( LED4_GPIO_Port,	 LED4_Pin, 	GPIO_PIN_RESET)		// D4のLEDを消灯する
#define LED_D4_TOGGLE()		HAL_GPIO_TogglePin(LED4_GPIO_Port,	 LED4_Pin)						// mこの関数を呼ぶたびにD4のLEDの点灯と消灯を切り替える
#define LED_D5_ON()			HAL_GPIO_WritePin( LED5_GPIO_Port, 	 LED5_Pin, 	GPIO_PIN_SET)		// D5のLEDを点灯する
#define LED_D5_OFF()		HAL_GPIO_WritePin( LED5_GPIO_Port, 	 LED5_Pin, 	GPIO_PIN_RESET)		// D5のLEDをを消灯する
#define LED_D5_TOGGLE()		HAL_GPIO_TogglePin(LED5_GPIO_Port, 	 LED5_Pin)						// mこの関数を呼ぶたびにD5のLEDの点灯と消灯を切り替える
#define LED_ALL_ON()		HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET)		// m全LEDを点灯する
#define LED_ALL_OFF()		HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET)	// m全LEDを消灯する
#define LED_ALL_TOGGLE()	HAL_GPIO_TogglePin(GPIOA, LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin)					// 全LEDの点灯と消灯を切り替える

/* mスイッチ関数群 */
#define SWITCH_ONOFF()		HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin)				// mスイッチが押されるとハイが返ってくる

/* UART通信関数群(communication.c) */
/* m自動生成されたsyscalls.cをSrcファイルに移し、stdio.hをインクルードすることでprintfやscanfも使用可能 */
void 		Communication_TerminalSend( uint8_t );		// 1文字送信
uint8_t 	Communication_TerminalRecv( void );			// 1文字受信
void 		Communication_Initialize( void );			// printfとscanfを使用するための設定
void 		Communication_ClearScreen( void );			// m画面クリア&カーソル初期化

/* mモータ関数群(motor.c) */
void 		Motor_Initialize( void );					// mモータ駆動用タイマーの開始
void 		Motor_StopPWM( void );						// mモータを停止
void 		Motor_SetDuty_Left( int16_t );				// m左モータを指定したDutyで回転させる[0-1000]
void 		Motor_SetDuty_Right( int16_t );				// m右モータを指定したDutyで回転させる[0-1000]

/* mエンコーダ関数群(encoder.c) */
void 		Encoder_Initialize( void );					// mエンコーダ用タイマーの開始
void 		Encoder_ResetCount_Left( void );			// m左エンコーダのカウントを初期値にする
void 		Encoder_ResetCount_Right( void );			// m右エンコーダのカウントを初期値にする
float 		Encoder_GetAngle_Left( void );				// m左タイヤの角度を取得する[rad]
float 		Encoder_GetAngle_Right( void );				// m右タイヤの角度を取得する[rad]
void		Get_speed(void);							// m角速度、速度算出(1msタスク)
void 		speed_m_average( void );					// m 並進方向速度移動平均算出(1msタスク)

/* m慣性センサ関数群(imu.c) */
uint8_t		IMU_CheckWHOAMI( void );					// m 慣性センサの動作確認関数(0xE0が返ってくれば正常)
void		IMU_Initialize( void );						// m 慣性センサの初期設定
void 		IMU_ResetReference( void );					// m 慣性センサのリファレンスを補正する
float 		IMU_GetAccel_X( void );						// X軸加速度計の加速度を取得する[m/s^2]
float 		IMU_GetGyro_Z( void );						// Z軸ジャイロの角速度を取得する[rad/s]

/* m 赤外センサ関数群(ir_sensor.c) */
void 		Sensor_Initialize( void );					// AD変換の初期設定
void 		Sensor_StartADC( void );					// AD変換を開始する
void 		Sensor_StopADC( void );						// AD変換を停止する
uint16_t 	Sensor_GetBatteryValue( void );				// m 電源電圧のAD値を取得する
int16_t 	Sensor_GetValue( uint8_t );					// m 赤外センサのLEDオンオフ差分値を取得する
														// 0:前左、1:横左、2:横右、3:前右
double 		SensorValue2length( uint8_t);				//壁センサの距離変換

void		Sensor_DebugPrintf(void);
void		ADC_Start_DMA(void);
void		ADC_Stop_DMA(void);

/* m バッテリー関数群(battery.c) */
float 		Battery_GetVoltage( void );					// m バッテリの電圧を取得する[V]
void 		Battery_LimiterVoltage( void );				// m バッテリの電圧が3.2V以下になると起動しないように制限する

/* m 割り込み関数群(interrupt.c) */
void 		Interrupt_Initialize( void );				// m メインの割り込み処理の初期設定
void 		Interrupt_Main( void );						// m メインの割り込み処理を書く
uint16_t 	Interrupt_GetDuty( void );					// m 割り込み処理内の計算割合を取得する
uint16_t 	Interrupt_GetDuty_Max( void );				// m 割り込み処理内の最大計算割合を取得する
float		Interrupt_GetBootTime( void );				// m マイコンが起動してから経過した時間を取得する[s]

/* m モジュールテスト関数群(module_test.c) */
void 		module_test( void );						// m 全モジュールの動作確認用テスト関数
void		data_read(void);							// m ログ出力関数
void 		data_get (void);							// m ログ取得関数(1msタスク)
void 		log_init (void);							// m ログカウンタ初期化

/*m 目標速度関数群(target.c)*/
void 		Operation_amount_calc(void);				//a 操作量算出
void 		target_speed_inc(void);						//a 目標速度更新
void 		target_speed_m_calc(void);					//a 加速度更新(1ms
void 		target_speed_w_calc(void);					//a 角加速度更新(1ms
void 		target_distance_m_set(float);				//a 目標距離入力
void 		target_distance_w_set(float);				//a 目標角度入力
void		real_distance_m_calc(void);					//a 並進方向移動距離取得
void		real_distance_w_calc(void);					//a 移動角取得
void 		real_distance_m_clr (void);					//a 並進方向移動距離クリア
void 		real_distance_w_clr (void);					//a 移動角クリア
void 		ideal_distance_m_clr(void);					//a 理想現在移動距離クリア
void 		ideal_distance_w_clr(void);					//a 理想現在角度クリア


void 		start_acceleration (void);					//a スタート時の加速
void		half_acceleration (void);					//a 半区画加速
void		half_deceleration (void);					//a 半区画減速
void		constant_speed (void);						//a 一区画定速
void 		turn_clk_90 (void);							//a 時計回りに90度回転
void 		turn_conclk_90 (void);						//a 反時計回りに90度回転
void		turn_conclk_180 (void);						//a 180度回転
void 		move_front (void);							//a 一区画前進
void		move_right(void);							//a 右折
void        move_left(void);							//a 左折
void		move_back(void);							//a バック
//m 前壁補正用
void 		fornt_wall_calibrate (void);				//a　前壁補正用
void 		calibrate_tim (void);						//a　前壁補正のカウンタ[1msタスク]
void 		front_wall_calib_flg_clr(void);			//a 前壁補正フラグをクリア

/*delay 関数(delay.c)*/
void 		delay_us( uint32_t );						//a ディレイ関数（us)

/*mode関数　(mode.c)*/
void		mode_select(void);							//mode選択関数。右タイヤ速度でカウントアップ、ダウン
uint8_t 	modechangejud_stanby(void);					//stanbyモード移行関数


#endif /* INDEX_H_ */
