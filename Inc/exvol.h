#ifndef EXVOL_H_
#define EXVOL_H_

//制御モード
typedef enum {
	trace = 0,
	front_wall = 1,
	side_wall = 2,
} ctrl_mode_num;

void motor_1ms ( void );
void set_mode_ctrl( ctrl_mode_num );
void set_motor_vol(void);
void set_motor_vol_trace(void);
void set_motor_vol_front_wall(void);
void set_motor_vol_side_wall(void);
void clr_motor_vol(void);
void clr_operate_history(void);
void calc_vol2duty ( void );
void motor_duty_adjust (void);
void calc_motor_vol ( void );
void set_motor_duty (void);
int16_t get_target_duty_r ( void );
int16_t get_target_duty_l ( void );



#endif /* EXVOL_H_*/