/*
 * communication.c
 *
 *  Created on: Jul 14, 2019
 *      Author: 岡田 泰裕
 */



#include "index.h"

/* ---------------------------------------------------------------
	printfとscanfを使用するための設定
--------------------------------------------------------------- */
void Communication_Initialize( void )
{
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	setbuf(stderr, NULL);
}

/* ---------------------------------------------------------------
	printfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch) {
HAL_UART_Transmit(&huart1/*使用しているusartに変更すること*/, &ch, 1, 1);
}


/* ---------------------------------------------------------------
	scanfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
int __io_getchar(void) {
HAL_StatusTypeDef Status = HAL_BUSY;
uint8_t Data;

while(Status != HAL_OK)
Status = HAL_UART_Receive(&huart1, &Data, 1, 10);

return(Data);
}

