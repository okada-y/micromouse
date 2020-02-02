#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_
#include <stdio.h>
#include <math.h>

void Sensor_Initialize( void );
void Sensor_StartADC( void );
void Sensor_StopADC( void );
uint16_t Sensor_GetBatteryValue( void );
int16_t Sensor_GetValue( uint8_t dir );
double SensorValue2length( uint8_t dir );
void Sensor_DebugPrintf( void );

#endif /* IR_SENSOR_H_*/