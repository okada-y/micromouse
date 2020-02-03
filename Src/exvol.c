//軌道制御、軌道補正から得た印加電圧から、
//モータに出力する印加電圧を決定する。
static uint8_t correction_mode = 0;		//m 補正モードフラグ 0:目標速度　1:前壁センサ値



	/* m バッテリー電圧とモータに印加する電圧から、duty[*0.1%]を算出	*/
	operation_amount_r = target_vol_r / (Battery_GetVoltage()) * 1000;
	operation_amount_l = target_vol_l / (Battery_GetVoltage()) * 1000;


	if( operation_amount_r > 0){
		operation_amount_r = operation_amount_r + 40;  //40->30
	}
	else{
		operation_amount_r = operation_amount_r - 45; //45 ->35
	}

	if( operation_amount_l > 0){
		operation_amount_l = operation_amount_l + 40;  //40->30
	}
	else{
		operation_amount_l = operation_amount_l - 45; //45 ->35
	}

	/*mモータに電圧を印加 */
	Motor_SetDuty_Left((int16_t)operation_amount_l);
	Motor_SetDuty_Right((int16_t)operation_amount_r);
