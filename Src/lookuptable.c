#include <stdio.h>
#include "lookuptable.h"

float table_sidewall[table_num_sidewall] = {
    0,
    0.0249860385419959,
    0.0353832462503951,
    0.0414652228720175,
    0.0457804539587943,
    0.0491276072285074,
    0.0518624305804167,
    0.0541746907778256,
    0.0561776616671934,
    0.0579444072020392,
    0.0595248149369066,
    0.0609544676339715,
    0.0622596382888159,
    0.0634602789039189,
    0.0645718984862248,
    0.0656067915585291,
    0.0665748693755926,
    0.0674842387028391,
    0.0683416149104384,
    0.0691526232294925,
    0.0699220226453058,
    0.0706538751078472,
    0.0713516753423706,
    0.0720184517809331,
    0.0726568459972151,
    0.0732691759150189,
    0.0738574866123181,
    0.0744235915320608,
    0.0749691061946239,
    0.075495475991793,
    0.0760039992669282,
    0.0764958466092731,
    0.0769720770839918,
    0.0774336519639931,
    0.0778814464112383,
    0.0783162594643371,
    0.0787388226188376,
    0.0791498072316593,
    0.0795498309378917,
    0.0799394632339406,
    0.0803192303537049,
    0.0806896195425605,
    0.0810510828162464,
    0.0814040402773993,
    0.0817488830507698,
    0.0820859758885507,
    0.0824156594893323,
    0.0827382525676468,
    0.0830540537056143,
    0.0833633430136553,
    0.0836663836234181,
    0.0839634230328608,
    0.0842546943207173,
    0.0845404172452777,
    0.08482079924046,
    0.085096036320483,
    0.0853663139030231,
    0.0856318075595141,
    0.0858926837001922,
    0.0861491002005817,
    0.0864012069753274,
    0.0866491465045956,
    0.0868930543176723,
    0.0871330594378689
};

float table_front_right_wall[table_num_frontwall] = {
    0.09,
    0.06023,
    0.050611,
    0.04318,
    0.036382,
    0.030802,
    0.027069,
    0.024288,
    0.021629,
    0.02021,
    0.018828,
    0.017416,
    0.016468,
    0.014989,
    0.014015,
    0.013527,
    0.012541,
    0.011573,
    0.011085,
    0.010596,
    0.010094,
    0.0096,
    0.009114,
    0.00863499999999999,
    0.008129,
    0.0079,
    0.00762499999999999,
    0.00713699999999999,
    0.0068,
    0.00663999999999999,
    0.006161,
    0.005674,
};

float table_front_left_wall[table_num_frontwall] = {
        0.09,
    0.054043,
    0.045573,
    0.038322,
    0.031608,
    0.026939,
    0.024025,
    0.021606,
    0.019535,
    0.017441,
    0.015916,
    0.014976,
    0.01361,
    0.012644,
    0.011652,
    0.010634,
    0.010111,
    0.009085,
    0.00856999999999999,
    0.008059,
    0.00755,
    0.007033,
    0.006524,
    0.006025,
    0.00551,
    0.00498899999999999,
    0.0047,
    0.004469,
    0.003938,
    0.003402,
    0.002862,
    0.002313,
};

//機能	: 前右壁センサ値をルックアップテーブルにより距離に変換する
//引数	: 前右壁センサ値
//返り値	: 距離
float get_frontrightwall_dis_table ( int16_t sensor_value )
{
    int16_t table_int = 0;//テーブルの整数部
    int16_t table_deci = 0;//テーブルの少数部
    float table_tilt = 0;//テーブル間の傾き
    float wall_dis = 0; //距離変換結果

    table_int = sensor_value / table_step_frontwall;

    if( table_int < 0 ){
        wall_dis = table_front_right_wall[0];
    }
    else if (table_int >= table_num_frontwall -1 ){
        wall_dis = table_front_right_wall[table_num_frontwall -1 ];
    }
    else
    {
        table_deci = sensor_value % table_step_frontwall;
        table_tilt = (table_front_right_wall[table_int+1]-table_front_right_wall[table_int])/table_step_frontwall;
        wall_dis = table_front_right_wall[table_int] + (float)table_deci * table_tilt;
    }

    return wall_dis;
}

//機能	: 前左壁センサ値をルックアップテーブルにより距離に変換する
//引数	: 前左壁センサ値
//返り値	: 距離
float get_frontleftwall_dis_table ( int16_t sensor_value )
{
    int16_t table_int = 0;//テーブルの整数部
    int16_t table_deci = 0;//テーブルの少数部
    float table_tilt = 0;//テーブル間の傾き
    float wall_dis = 0; //距離変換結果

    table_int = sensor_value / table_step_frontwall;

    if( table_int < 0 ){
        wall_dis = table_front_left_wall[0];
    }
    else if (table_int >= table_num_frontwall -1 ){
        wall_dis = table_front_left_wall[table_num_frontwall -1 ];
    }
    else
    {
        table_deci = sensor_value % table_step_frontwall;
        table_tilt = (table_front_left_wall[table_int+1]-table_front_left_wall[table_int])/table_step_frontwall;
        wall_dis = table_front_left_wall[table_int] + (float)table_deci * table_tilt;
    }

    return wall_dis;
}



//機能	: 横壁センサ値をルックアップテーブルにより距離に変換する
//引数	: 横壁センサ値
//返り値	: 距離
float get_sidewall_dis_table ( int16_t sensor_value )
{
    int16_t table_int = 0;//テーブルの整数部
    int16_t table_deci = 0;//テーブルの少数部
    float table_tilt = 0;//テーブル間の傾き
    float side_wall_dis = 0; //距離変換結果

    table_int = sensor_value / table_step_sidewall;

    if( table_int < 0 ){
        side_wall_dis = table_sidewall[0];
    }
    else if (table_int >= table_num_sidewall -1 ){
        side_wall_dis = table_sidewall[table_num_sidewall -1 ];
    }
    else
    {
        table_deci = sensor_value % table_step_sidewall;
        table_tilt = (table_sidewall[table_int+1]-table_sidewall[table_int])/table_step_sidewall;
        side_wall_dis = table_sidewall[table_int] + (float)table_deci * table_tilt;
    }

    side_wall_dis = 0.09 - side_wall_dis;
    return side_wall_dis;
}