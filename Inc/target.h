#ifndef TARGET_H_
#define TARGET_H_


//移動方向モード
typedef enum {
	forward_mode 	= 0,	//前進
	backward_mode 	= 1,	//後進
} direction_mode;

//回転方向モード
typedef enum {
	counter_clockwise 	= 0,	//反時計回り
	clockwise 	= 1,			//時計回り
} rotation_mode;

//加速モード
typedef enum {
	acceleration 	= 0, //加速
	deceleration 	= 1, //減速
} accel_mode;

//下限速度モード
typedef enum {
	zero 	= 0, 	//停止
	slow 	= 1, 	//スロー
} speed_under_lim_mode;

void target_1ms ( void );
void set_direction_mode ( direction_mode );
void set_rotation_mode ( rotation_mode );
void set_accel_mode ( accel_mode );
void set_speed_under_lim_flg ( speed_under_lim_mode );
float get_target_length ( void );
float get_ideal_length ( void );
float get_target_angle ( void );
float get_ideal_angle ( void );
void set_ideal_length ( float );
void set_ideal_angle ( float );
float get_target_move_speed ( void );
float get_target_rotation_speed ( void );
void set_target_length ( float );
void set_target_angle ( float );
void calc_target_move_accel(void);
void calc_rotate_accel(void);
void calc_target_move_speed(void);
void calc_target_rotation_speed ( void );





#endif /* TARGET_H_*/