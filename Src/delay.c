#include "index.h"



void delay_us(uint32_t us) {
	SysTick->LOAD = 96 - 1; // reset value for count-down timer
	SysTick->VAL = 0; // initial value
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // count start

	for(uint32_t i=0; i<us; i++){
		while( !(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) );
	}
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // count stop
}
