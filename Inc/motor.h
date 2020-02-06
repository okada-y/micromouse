#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>

void Motor_Initialize( void );
void Motor_StopPWM( void );
void set_duty_l( int16_t );
void set_duty_r( int16_t );
int16_t get_duty_r ( void );
int16_t get_duty_l ( void );


#endif /* MOTOR_H_*/