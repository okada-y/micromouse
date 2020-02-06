#ifndef ENCORDER_H_
#define ENCORDER_H_

#define ENC_CNT_L		(TIM3 -> CNT)
#define ENC_CNT_R		(TIM2 -> CNT)
#define ENC_CNT_SR_L	(TIM3 -> SR)	//TIM3のstatus register
#define ENC_CNT_SR_R	(TIM2 -> SR) 	//TIM2のstatus register
#define ENC_ZERO		(32767 - 1)		/* encoderの初期値 */
#define ENC_RESOLUTION 	(1024 - 1)		/* encoderの分解能 */

uint16_t get_encordercount_r ( void );
uint16_t get_encordercount_l ( void );
void Encoder_Initialize( void );
void Encoder_ResetCount_Left( void );
void Encoder_ResetCount_Right( void );
float Encoder_GetAngle_Left( void );
float Encoder_GetAngle_Right( void );
void Get_speed(void);
void speed_m_average( void );


#endif /* ENCORDER_H_*/