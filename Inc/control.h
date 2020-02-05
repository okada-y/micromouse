#ifndef CONTROL_H_
#define CONTROL_H_

float get_target_vol_r_ctrl ( void );
float get_target_vol_l_ctrl ( void );
void calc_motor_vol_ctrl(void);
void clr_trace_operate_history ( void );

#endif /* CONTROL_H_*/

