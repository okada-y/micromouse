#ifndef ADJUST_H_
#define ADJUST_H_

typedef enum{
    right,
    left,
    both_side,
    none
}side_wall_ctrl;

void adjust_1ms (void);
void set_target_side_sensor(void);
void calc_motor_vol_side_wall ( void );
void calc_motor_vol_front_wall ( void );
float get_target_vol_r_frontwall ( void );
float get_target_vol_l_frontwall ( void );
void clr_frontwall_operate_history ( void );
void fornt_wall_calibrate (void);
void calibrate_tim (void);

#endif /* ADJUST_H_*/