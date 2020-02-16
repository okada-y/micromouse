#ifndef CONTROL_H_
#define CONTROL_H_

float get_target_vol_sum_ctrl ( void );
float get_target_vol_diff_ctrl ( void );
void calc_motor_vol_ctrl(void);
void clr_trace_operate_history ( void );
void adjust_trace_theta ( void );

#endif /* CONTROL_H_*/

