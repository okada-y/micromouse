//irセンサによる補正記述予定
static uint16_t fornt_wall_calibrate_tim = 0; //m 前壁補正用カウンタ


	/*m 前壁補正*/
	if(correction_mode == 1){
		l_front_sensor_l = SensorValue2length(0);
		l_front_sensor_r = SensorValue2length(3);

		/*m 偏差取得*/
		l_front_sensor_l_err = l_front_sensor_l - front_sensor_l_ref ;		//m 目標距離　－　センサ距離
		l_front_sensor_r_err = l_front_sensor_r - front_sensor_r_ref ;		//m 目標距離　－　センサ距離

		/* m 偏差変換*/
		l_front_sensor_m_err = l_front_sensor_r_err + l_front_sensor_l_err;
		l_front_sensor_w_err = (l_front_sensor_r_err - l_front_sensor_l_err)/chassis_width; //角度に変換(atanを0近傍で線形化)

		/*m 偏差積分(I項)*/
		l_front_sensor_m_err_I  += front_sensor_m_KI * 0.001 * l_front_sensor_m_err;
		l_front_sensor_w_err_I  += front_sensor_w_KI * 0.001 * l_front_sensor_w_err;

		/*P項*/
		l_front_sensor_m_err_P = front_sensor_m_KP * l_front_sensor_m_err;
		l_front_sensor_w_err_P = front_sensor_w_KP * l_front_sensor_w_err;

		/*D項*/
		l_front_sensor_m_err_D = (l_front_sensor_m_D_prev+front_sensor_m_KD*front_sensor_m_fil*(l_front_sensor_m_err - l_front_sensor_m_err_prev))
									/(1+front_sensor_m_fil*0.001);
		l_front_sensor_w_err_D = (l_front_sensor_w_D_prev+front_sensor_w_KD*front_sensor_w_fil*(l_front_sensor_w_err - l_front_sensor_w_err_prev))
									/(1+front_sensor_w_fil*0.001);

		/*PID*/
		l_front_sensor_m_PID = l_front_sensor_m_err_P + l_front_sensor_m_err_I + l_front_sensor_m_err_D;
	    l_front_sensor_w_PID = l_front_sensor_w_err_P + l_front_sensor_w_err_I + l_front_sensor_w_err_D;

	    /*m 印加電圧算出*/
		target_vol_r = (l_front_sensor_m_PID + l_front_sensor_w_PID)/2;
		target_vol_l = (l_front_sensor_m_PID - l_front_sensor_w_PID)/2;

		/*m パラメータ更新*/
	    l_front_sensor_m_err_prev = l_front_sensor_m_err;   		//m 前回偏差和
	    l_front_sensor_w_err_prev = l_front_sensor_w_err;   		//m 前回偏差差
	    l_front_sensor_m_D_prev = l_front_sensor_m_err_D;		   	//m 前回偏差和微分
	    l_front_sensor_w_D_prev = l_front_sensor_w_err_D;		   	//m 前回偏差差微分

////////////////////////////////////////
/* a マウス位置補正関数					*/
/* a 壁を使った位置の補正用関数				*/
////////////////////////////////////////

/* memo:前壁補正
 * param:
 *  * */
void fornt_wall_calibrate (void)
{
	double temp_r;
	double temp_l;
	double temp;


	  correction_mode = 1; //m 前壁補正モードに切り替え
	  fornt_wall_calibrate_tim = 0; //m前壁補正タイマを初期化

	  while(1)
	  {
		  temp_r = ABS(front_sensor_r_ref - SensorValue2length(3));
		  temp_l = ABS(front_sensor_l_ref - SensorValue2length(0));
		  temp = MAX(temp_r,temp_l);
		  //mセンサ値が基準より差を持つとき、タイマをリセット
		  if(temp > front_sensor_th){
			  fornt_wall_calibrate_tim = 0;
		  }
		  //m キャリブレーション時間を超えるとき、ブレイク
		  if(fornt_wall_calibrate_tim >= calib_tim ){
			  break;
		  }
		  printf("temp: %8.5f,time:%8.5d \r\n",temp,fornt_wall_calibrate_tim);
	  }

	  //m 補正終了時、移動距離、角度を初期化
	  real_distance_m_clr();
	  real_distance_w_clr();
	  target_distance_m_clr();
	  target_distance_w_clr();
	  ideal_distance_m_clr();
	  ideal_distance_w_clr();

	  //m 補正モードを目標速度に変更
	  correction_mode = 0;
}

/* memo:前壁補正用タイマ
 * param:
 *  * */
void calibrate_tim (void){

	fornt_wall_calibrate_tim += 1;

	if(fornt_wall_calibrate_tim > calib_tim){
		fornt_wall_calibrate_tim = calib_tim;
	}
}



