/*
 * maze.c
 * matlabとCの統合
 *  Created on: 2019/10/07
 *      Author: 岡田 泰裕
 */

typedef enum
{
    no_calib,   //補正しない
    calib       //補正する
}run_calib;

struct fwallcalib
{
    run_calib right_wall;
    run_calib front_wall;
    run_calib left_wall;
}front_wall_calib_flg;

void set_calib_flg(run_calib wall)
{
    
}