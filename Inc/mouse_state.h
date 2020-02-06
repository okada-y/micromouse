#ifndef MOUSE_STATE_H_
#define MOUSE_STATE_H_

#include <stdio.h>

void mouse_state_1ms ( void );
int16_t Get_diff_right_count( void );
int16_t Get_diff_left_count( void );
void calc_move_speed ( void );
float get_tire_r_speed ( void );
float get_tire_r_speed_max ( void );
float get_tire_r_speed_min ( void );
float get_tire_l_speed ( void );
float get_tire_l_speed_max ( void );
float get_tire_l_speed_min ( void );
void init_tire_speed ( void );
float get_move_speed ( void );
void filter_move_speed( void );
float get_move_speed_ave ( void );
float get_move_accel_ave ( void );
void calc_move_length ( void );
void calc_rotation_speed ( void );
float get_rotation_speed ( void );
void calc_rotation_angle ( void );
float get_rotation_angle ( void );
float get_move_length ( void );
void set_move_length ( float );
void set_rotation_angle ( float );


#endif /* MOUSE_STATE_H_*/