#ifndef LOOKUPTABLE_H_
#define LOOKUPTABLE_H_

#define table_num_sidewall  (64)            //横壁距離用テーブルの数
#define table_step_sidewall  (32)           //横壁距離用テーブル間隔

#define table_num_frontwall (32)            //前壁距離用テーブルの数        
#define table_step_frontwall (100)          //前壁距離用テーブル間隔

float get_sidewall_dis_table ( int16_t );
float get_frontrightwall_dis_table ( int16_t );
float get_frontleftwall_dis_table ( int16_t );


#endif /* LOOKUPTABLE_H_*/