/*
 * battery.c
 *
 *  Created on: Aug 12, 2019
 *      Author: 岡田 泰裕
 */

#include "index.h"

#define BATTERY_REFERENCE	(3.0f)
#define BATTERY_LIMIT		(3.2f)


/* ---------------------------------------------------------------
	バッテリの電圧を取得する関数
--------------------------------------------------------------- */
float Battery_GetVoltage( void )
{
	return (BATTERY_REFERENCE * (20.f / 10.f) * (float)Sensor_GetBatteryValue()) / 4096.f;
}

/* ---------------------------------------------------------------
	バッテリの電圧制限関数
--------------------------------------------------------------- */
void Battery_LimiterVoltage( void )
{
	volatile int	i;
	volatile float	battery_voltage_average;

	for( i = 0; i < 10; i++) {
		HAL_Delay(10);
		battery_voltage_average += Battery_GetVoltage();
	}
	battery_voltage_average /= 10;

	if( battery_voltage_average < BATTERY_LIMIT ) {
		while( 1 ) {
			LED_ALL_TOGGLE();
			HAL_Delay(200);
		}
	} else;
}
