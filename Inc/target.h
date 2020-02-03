#ifndef TARGET_H_
#define TARGET_H_


typedef enum {
	forward_mode 	= 0,
	backward_mode 	= 1,
} direction_mode;

typedef enum {
	counter_clockwise 	= 0,
	clockwise 	= 1,
} rotation_mode;


typedef enum {
	acceleration 	= 0, //m加速
	deceleration 	= 1, //m減速
} accel_mode;


void target_1ms ( void );
float get_target_move_speed ( void );
float get_target_rotation_speed ( void );
void set_target_length ( float set_length );
void set_target_angle ( float set_angle );
void calc_target_move_accel(void);
void calc_rotate_accel(void);
void calc_target_move_speed(void);
void calc_target_rotation_speed ( void );





#endif /* TARGET_H_*/