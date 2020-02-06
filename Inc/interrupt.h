#ifndef INTERRUPT_H_
#define INTERRUPT_H_

void 		Interrupt_Initialize( void );	//メインの割り込み処理の初期設定
void 		Interrupt_Main( void );			//メインの割り込み処理を書く
uint16_t 	Interrupt_GetDuty( void );		//割り込み処理内の計算割合を取得する
uint16_t 	Interrupt_GetDuty_Max( void );	//割り込み処理内の最大計算割合を取得する
float		Interrupt_GetBootTime( void );	//マイコンが起動してから経過した時間を取得する[s]


#endif /* INTERRUPT_H_*/