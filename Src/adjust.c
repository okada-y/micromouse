//irセンサによる補正記述予定

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
