/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_init.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 18-Nov-2019 23:53:15
 */

/* Include Files */
#include <string.h>
#include "maze_init.h"
#include "maze_solve.h"

/* Include Original*/
#include "index.h"

/* Function Definitions */

/*
 * maze_init 迷路情報の初期化
 * 入力 迷路横サイズ, 迷路縦サイズ
 * 出力 迷路壁情報,迷路壁探索済み情報
 * Arguments    : unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 * Return Type  : void
 */
void maze_init(unsigned char maze_row_size, unsigned char maze_col_size,
               unsigned char maze_wall[1024], unsigned char maze_wall_search
               [1024])
{
  unsigned int qY;
  int i0;
  int i;
  int i1;
  int n;
  int maze_wall_tmp;

  /* 方角定義 */
  /* 迷路情報、探索済情報初期化 */
  /*  maze_wall = uint8(zeros(maze_col_size-1,maze_row_size-1)); */
  /*  maze_wall_search = uint8(zeros(maze_col_size-1,maze_row_size-1)); */
  memset(&maze_wall[0], 0, sizeof(unsigned char) << 10);

  /* 既知となる外周4辺の壁情報を入力、探索済みとする。 */
  qY = maze_row_size - 1U;
  if (qY > maze_row_size) {
    qY = 0U;
  }

  i0 = (int)qY;
  for (i = 0; i < i0; i++) {
    qY = maze_col_size - 1U;
    if (qY > maze_col_size) {
      qY = 0U;
    }

    i1 = (int)qY;
    for (n = 0; n < i1; n++) {
      /* 北側 */
      qY = maze_row_size - 1U;
      if (qY > maze_row_size) {
        qY = 0U;
      }

      if (1 + i == (int)qY) {
        maze_wall_tmp = i + (n << 5);
        maze_wall[maze_wall_tmp] |= 1;
      }

      /* 東側 */
      qY = maze_col_size - 1U;
      if (qY > maze_col_size) {
        qY = 0U;
      }

      if (1 + n == (int)qY) {
        maze_wall[i + (n << 5)] = (unsigned char)(maze_wall[i + (n << 5)] | 2);
      }

      /* 南側 */
      if (1 + i == 1) {
        maze_wall[n << 5] = (unsigned char)(maze_wall[n << 5] | 4);
      }

      /* 西側 */
      if (1 + n == 1) {
        maze_wall[i] = (unsigned char)(maze_wall[i] | 8);
      }
    }
  }

  /* スタート時の壁は|_|となっているので、あらかじめ入力 */
  /* スタートマスから見て東側 */
  maze_wall[0] = (unsigned char)(maze_wall[0] | 2);

  /* スタートマスの一マス東側のマスから見て西側 */
  maze_wall[32] = (unsigned char)(maze_wall[32] | 8);

  /* 以上の壁情報は探索済みとする */
  memcpy(&maze_wall_search[0], &maze_wall[0], sizeof(unsigned char) << 10);

  /* 座標(1,1)の北側、(2,1)の南側は壁がないことがわかっているので探索済みとする。 */
  maze_wall_search[0] = (unsigned char)(maze_wall[0] | 1);
  maze_wall_search[1] = (unsigned char)(maze_wall[1] | 4);
}

/*
 * File trailer for maze_init.c
 *
 * [EOF]
 */
