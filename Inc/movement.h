#ifndef MOVEMENT_H_
#define MOVEMENT_H_


typedef enum {
	start 	= 0, //走行開始
	already = 1, //それ以外
} run_start;

typedef enum {
	nowall 	= 0, //壁なし
	wall 	= 1, //壁あり
} wall_flg;

uint8_t move_comp_jud ( void );
uint8_t move_comp_jud_stop ( void );
uint8_t rotate_comp_jud ( void );
void start_acceleration (void);
void half_acceleration (void);
void half_deceleration (void);
void constant_speed (void);
void constant_speed_offset (float);
void turn_clk_90 (void);
void turn_conclk_90 (void);
void turn_conclk_180 (void);
void clr_run_first_flg (void);
void clr_wall_flg (void);
void set_front_wall_flg ( void );
void set_rigth_wall_flg ( void );
void set_left_wall_flg ( void );
void move_front (void);
void move_right (void);
void move_left (void);
void move_back (void);

void slalom_clock_90 (void);
void slalom_conclock_90 (void);



#endif /* MOVEMENT_H_*/