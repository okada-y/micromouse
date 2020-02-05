#ifndef IMU_H_
#define IMU_H_

#include <stdio.h>

uint8_t		IMU_CheckWHOAMI( void );		// 慣性センサの動作確認関数(0xE0が返ってくれば正常)
void		IMU_Initialize( void );			// 慣性センサの初期設定
void 		IMU_ResetReference( void );		// 慣性センサのリファレンスを補正する
float 		IMU_GetAccel_X( void );			// X軸加速度計の加速度を取得する[m/s^2]
float 		IMU_GetGyro_Z( void );			// Z軸ジャイロの角速度を取得する[rad/s]

#endif /* IMU_H_*/