/*
 * index.h
 *
 *  Created on: Jul 14, 2019
 *      Author: Yasuhiro Okada
 */

#ifndef INDEX_H_
#define INDEX_H_


#include <stdio.h>
#include <math.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/*extern 宣言*/




/* LED関数群 */
#define LED_D2_ON()			HAL_GPIO_WritePin( LED2_GPIO_Port, 	 LED2_Pin,	GPIO_PIN_SET)		// D2のLEDを点灯する
#define LED_D2_OFF()		HAL_GPIO_WritePin( LED2_GPIO_Port,	 LED2_Pin,	GPIO_PIN_RESET)		// D2のLEDを消灯する
#define LED_D2_TOGGLE()		HAL_GPIO_TogglePin(LED2_GPIO_Port,	 LED2_Pin)						// mこの関数を呼ぶたびにD2のLEDの点灯と消灯を切り替える
#define LED_D3_ON()			HAL_GPIO_WritePin( LED3_GPIO_Port, 	 LED3_Pin,	GPIO_PIN_SET)		// D3のLEDを点灯する
#define LED_D3_OFF()		HAL_GPIO_WritePin( LED3_GPIO_Port, 	 LED3_Pin,  GPIO_PIN_RESET)		// D3のLEDを消灯する
#define LED_D3_TOGGLE()		HAL_GPIO_TogglePin(LED3_GPIO_Port, 	 LED3_Pin)						// mこの関数を呼ぶたびにD3のLEDの点灯と消灯を切り替える
#define LED_D4_ON()			HAL_GPIO_WritePin( LED4_GPIO_Port,   LED4_Pin,	GPIO_PIN_SET)		// D4のLEDを点灯する
#define LED_D4_OFF()		HAL_GPIO_WritePin( LED4_GPIO_Port,	 LED4_Pin, 	GPIO_PIN_RESET)		// D4のLEDを消灯する
#define LED_D4_TOGGLE()		HAL_GPIO_TogglePin(LED4_GPIO_Port,	 LED4_Pin)						// mこの関数を呼ぶたびにD4のLEDの点灯と消灯を切り替える
#define LED_D5_ON()			HAL_GPIO_WritePin( LED5_GPIO_Port, 	 LED5_Pin, 	GPIO_PIN_SET)		// D5のLEDを点灯する
#define LED_D5_OFF()		HAL_GPIO_WritePin( LED5_GPIO_Port, 	 LED5_Pin, 	GPIO_PIN_RESET)		// D5のLEDをを消灯する
#define LED_D5_TOGGLE()		HAL_GPIO_TogglePin(LED5_GPIO_Port, 	 LED5_Pin)						// mこの関数を呼ぶたびにD5のLEDの点灯と消灯を切り替える
#define LED_ALL_ON()		HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_SET)		// m全LEDを点灯する
#define LED_ALL_OFF()		HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET)	// m全LEDを消灯する
#define LED_ALL_TOGGLE()	HAL_GPIO_TogglePin(GPIOA, LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin)					// 全LEDの点灯と消灯を切り替える

/* スイッチ関数群 */
#define SWITCH_ONOFF()		HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin)				// mスイッチが押されるとハイが返ってくる

/* UART通信関数群(communication.c) */
/* m自動生成されたsyscalls.cをSrcファイルに移し、stdio.hをインクルードすることでprintfやscanfも使用可能 */
void 		Communication_TerminalSend( uint8_t );		// 1文字送信
uint8_t 	Communication_TerminalRecv( void );			// 1文字受信
void 		Communication_Initialize( void );			// printfとscanfを使用するための設定
void 		Communication_ClearScreen( void );			// m画面クリア&カーソル初期化


/*delay 関数(delay.c)*/
void 		delay_us( uint32_t );						//a ディレイ関数（us)

#endif /* INDEX_H_ */
