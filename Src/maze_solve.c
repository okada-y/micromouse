/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_solve.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 18-Nov-2019 23:53:15
 */

/* Include Files */
#include "maze_init.h"
#include "maze_solve.h"
#include "idivide.h"
#include "rem.h"
#include "matlab_code_gen_data.h"

/*Include original*/
#include "index.h"

/*original parameter*/
uint8_t front_calib_flg;
uint8_t right_calib_flg;
uint8_t left_calib_flg;

/* Type Definitions */
#ifndef typedef_coder_internal_ref
#define typedef_coder_internal_ref

typedef struct {
  unsigned char contents;
} coder_internal_ref;

#endif                                 /*typedef_coder_internal_ref*/

#ifndef typedef_d_struct_T
#define typedef_d_struct_T

typedef struct {
  unsigned char unknown;
  unsigned char known;
} d_struct_T;

#endif                                 /*typedef_d_struct_T*/

#ifndef typedef_coder_internal_ref_1
#define typedef_coder_internal_ref_1

typedef struct {
  d_struct_T contents;
} coder_internal_ref_1;

#endif                                 /*typedef_coder_internal_ref_1*/

#ifndef typedef_c_struct_T
#define typedef_c_struct_T

typedef struct {
  unsigned char nowall;
  unsigned char wall;
} c_struct_T;

#endif                                 /*typedef_c_struct_T*/

#ifndef typedef_coder_internal_ref_2
#define typedef_coder_internal_ref_2

typedef struct {
  c_struct_T contents;
} coder_internal_ref_2;

#endif                                 /*typedef_coder_internal_ref_2*/

#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
  unsigned char front;
  unsigned char right;
  unsigned char back;
  unsigned char left;
} b_struct_T;

#endif                                 /*typedef_b_struct_T*/

#ifndef typedef_coder_internal_ref_3
#define typedef_coder_internal_ref_3

typedef struct {
  b_struct_T contents;
} coder_internal_ref_3;

#endif                                 /*typedef_coder_internal_ref_3*/

#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  unsigned char North;
  unsigned char East;
  unsigned char South;
  unsigned char West;
} struct_T;

#endif                                 /*typedef_struct_T*/

#ifndef typedef_coder_internal_ref_4
#define typedef_coder_internal_ref_4

typedef struct {
  struct_T contents;
} coder_internal_ref_4;

#endif                                 /*typedef_coder_internal_ref_4*/

/* Function Declarations */
static void fust_run(const coder_internal_ref_4 *g_direction, coder_internal_ref
                     *current_x, coder_internal_ref *current_y,
                     coder_internal_ref *current_dir, const coder_internal_ref
                     *goal_size, const coder_internal_ref_2 *wall, const
                     coder_internal_ref_3 *l_direction, const unsigned char
                     maze_wall[1024], const unsigned char contour_map[1024],
                     const unsigned char maze_goal[18], unsigned char max_length);
static void make_map_find(const coder_internal_ref_4 *g_direction, const
  coder_internal_ref_2 *wall, unsigned char maze_row_size, unsigned char
  maze_col_size, const unsigned char maze_goal[18], unsigned char l_goal_size,
  const unsigned char maze_wall[1024], unsigned char contour_map[1024], unsigned
  char *max_length);
static void make_map_fustrun(const coder_internal_ref *goal_size, const
  coder_internal_ref_4 *g_direction, const coder_internal_ref_2 *wall, const
  coder_internal_ref_1 *search, unsigned char maze_row_size, unsigned char
  maze_col_size, const unsigned char maze_goal[18], const unsigned char
  maze_wall[1024], const unsigned char maze_wall_search[1024], unsigned char
  contour_map[1024], unsigned char *max_length);
static void move_step(const coder_internal_ref_4 *g_direction, unsigned char
                      *current_x, unsigned char *current_y, unsigned char
                      current_dir);
static void search_adachi(const coder_internal_ref_2 *wall, const
  coder_internal_ref_1 *search, const coder_internal_ref_4 *g_direction, const
  coder_internal_ref_3 *l_direction, unsigned char *current_x, unsigned char
  *current_y, unsigned char *current_dir, unsigned char maze_row_size, unsigned
  char maze_col_size, unsigned char maze_wall[1024], unsigned char
  maze_wall_search[1024], const unsigned char maze_goal[18], unsigned char
  l_goal_size);

/* Function Definitions */

/*
 * 入力　壁情報,壁探索情報,等高線MAP,ゴール座標,最大経路長
 * 出力
 * Arguments    : const coder_internal_ref_4 *g_direction
 *                coder_internal_ref *current_x
 *                coder_internal_ref *current_y
 *                coder_internal_ref *current_dir
 *                const coder_internal_ref *goal_size
 *                const coder_internal_ref_2 *wall
 *                const coder_internal_ref_3 *l_direction
 *                const unsigned char maze_wall[1024]
 *                const unsigned char contour_map[1024]
 *                const unsigned char maze_goal[18]
 *                unsigned char max_length
 * Return Type  : void
 */
static void fust_run(const coder_internal_ref_4 *g_direction, coder_internal_ref
                     *current_x, coder_internal_ref *current_y,
                     coder_internal_ref *current_dir, const coder_internal_ref
                     *goal_size, const coder_internal_ref_2 *wall, const
                     coder_internal_ref_3 *l_direction, const unsigned char
                     maze_wall[1024], const unsigned char contour_map[1024],
                     const unsigned char maze_goal[18], unsigned char max_length)
{
  unsigned char goal_flag;
  unsigned char little;
  unsigned char next_dir;
  int exitg1;
  unsigned char k;
  int q0;
  int tempi;
  unsigned char a;
  int b_k;
  int c_k;
  int d_k;
  unsigned int qY;
  int e_k;
  unsigned char varargin_2;
  unsigned char varargin_3;
  unsigned char varargin_4;

  /*  fust_run 最短経路導出 */
  /* local変数宣言 */
  goal_flag = 0U;

  /* ゴール判定フラグ */
  little = max_length;

  /* 進行方向選定用閾値 */
  /* マウス位置表示用オブジェクト */
  /* マウスの初期位置設定 */
  current_x->contents = 1U;
  current_y->contents = 1U;
  current_dir->contents = g_direction->contents.North;
  next_dir = g_direction->contents.North;

  /* 探索開始時x */
  /* 探索開始時y */
  /* 探索開始位置プロット */
  /* 足跡プロット */
  do {
    exitg1 = 0;

    /* 現在位置がゴールか判定 */
    k = goal_size->contents;
    q0 = k;
    for (tempi = 0; tempi < q0; tempi++) {
      if ((current_x->contents == maze_goal[tempi]) && (current_y->contents ==
           maze_goal[tempi + 9])) {
        goal_flag = 1U;
      }
    }

    if (goal_flag == 1) {
      exitg1 = 1;
    } else {
      /*             %%進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      /* 北側の壁のありなし */
      k = g_direction->contents.North;
      a = maze_wall[(current_y->contents + ((current_x->contents - 1) << 5)) - 1];
      if (k <= 7) {
        b_k = (unsigned char)(1 << k);
      } else {
        b_k = 0;
      }

      if ((a & b_k) == wall->contents.nowall) {
        /* 北側の等高線mapが閾値より低ければ、 */
        q0 = (int)(current_y->contents + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        if (contour_map[(q0 + ((current_x->contents - 1) << 5)) - 1] < little) {
          /* 閾値を北側の等高map値に変更 */
          q0 = (int)(current_y->contents + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          little = contour_map[(q0 + ((current_x->contents - 1) << 5)) - 1];

          /* 北側を進行方向に変更y */
          next_dir = g_direction->contents.North;
        }
      }

      /* 東側 */
      k = g_direction->contents.East;
      a = maze_wall[(current_y->contents + ((current_x->contents - 1) << 5)) - 1];
      if (k <= 7) {
        c_k = (unsigned char)(1 << k);
      } else {
        c_k = 0;
      }

      if ((a & c_k) == wall->contents.nowall) {
        q0 = (int)(current_x->contents + 1U);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        if (contour_map[(current_y->contents + ((q0 - 1) << 5)) - 1] < little) {
          q0 = (int)(current_x->contents + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          little = contour_map[(current_y->contents + ((q0 - 1) << 5)) - 1];
          next_dir = g_direction->contents.East;
        }
      }

      /* 南側 */
      k = g_direction->contents.South;
      a = maze_wall[(current_y->contents + ((current_x->contents - 1) << 5)) - 1];
      if (k <= 7) {
        d_k = (unsigned char)(1 << k);
      } else {
        d_k = 0;
      }

      if ((a & d_k) == wall->contents.nowall) {
        q0 = current_y->contents;
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        if (contour_map[((int)qY + ((current_x->contents - 1) << 5)) - 1] <
            little) {
          q0 = current_y->contents;
          qY = q0 - 1U;
          if (qY > (unsigned int)q0) {
            qY = 0U;
          }

          little = contour_map[((int)qY + ((current_x->contents - 1) << 5)) - 1];
          next_dir = g_direction->contents.South;
        }
      }

      /* 西側 */
      k = g_direction->contents.West;
      a = maze_wall[(current_y->contents + ((current_x->contents - 1) << 5)) - 1];
      if (k <= 7) {
        e_k = (unsigned char)(1 << k);
      } else {
        e_k = 0;
      }

      if ((a & e_k) == wall->contents.nowall) {
        q0 = current_x->contents;
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        if (contour_map[(current_y->contents + (((int)qY - 1) << 5)) - 1] <
            little) {
          q0 = current_x->contents;
          qY = q0 - 1U;
          if (qY > (unsigned int)q0) {
            qY = 0U;
          }

          little = contour_map[(current_y->contents + (((int)qY - 1) << 5)) - 1];
          next_dir = g_direction->contents.West;
        }
      }

      /*             %%現在方向と進行方向に応じた処理 */
      q0 = (int)(4U + next_dir);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      qY = (unsigned int)q0 - current_dir->contents;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      k = (unsigned char)((unsigned char)qY % 4);
      a = l_direction->contents.front;
      varargin_2 = l_direction->contents.right;
      varargin_3 = l_direction->contents.back;
      varargin_4 = l_direction->contents.left;
      if (a == k) {
        q0 = 0;
      } else if (varargin_2 == k) {
        q0 = 1;
      } else if (varargin_3 == k) {
        q0 = 2;
      } else if (varargin_4 == k) {
        q0 = 3;
      } else {
        q0 = -1;
      }

      switch (q0) {
       case 0:
        k = current_x->contents;
        a = current_y->contents;
        varargin_2 = current_dir->contents;

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (varargin_2 == g_direction->contents.North) {
          q0 = (int)(a + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          a = (unsigned char)q0;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (varargin_2 == g_direction->contents.East) {
          q0 = (int)(k + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          k = (unsigned char)q0;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (varargin_2 == g_direction->contents.South) {
          qY = a - 1U;
          if (qY > a) {
            qY = 0U;
          }

          a = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (varargin_2 == g_direction->contents.West) {
          qY = k - 1U;
          if (qY > k) {
            qY = 0U;
          }

          k = (unsigned char)qY;

          /* disp("west_step") */
        }

        current_x->contents = k;
        current_y->contents = a;

        /* disp("front") */
        break;

       case 1:
        k = current_dir->contents;

        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*  turn_clk_90deg 時計周りに90度ターンする関数  */
        q0 = (int)(4U + k);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        q0++;
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_dir->contents = (unsigned char)(q0 % 4);
        k = current_x->contents;
        a = current_y->contents;
        varargin_2 = current_dir->contents;

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (varargin_2 == g_direction->contents.North) {
          q0 = (int)(a + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          a = (unsigned char)q0;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (varargin_2 == g_direction->contents.East) {
          q0 = (int)(k + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          k = (unsigned char)q0;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (varargin_2 == g_direction->contents.South) {
          qY = a - 1U;
          if (qY > a) {
            qY = 0U;
          }

          a = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (varargin_2 == g_direction->contents.West) {
          qY = k - 1U;
          if (qY > k) {
            qY = 0U;
          }

          k = (unsigned char)qY;

          /* disp("west_step") */
        }

        current_x->contents = k;
        current_y->contents = a;

        /* disp("right") */
        break;

       case 2:
        k = current_dir->contents;

        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*  turn_180deg 180度ターンする関数 */
        q0 = (int)(4U + k);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_dir->contents = (unsigned char)((q0 - 2) % 4);
        k = current_x->contents;
        a = current_y->contents;
        varargin_2 = current_dir->contents;

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (varargin_2 == g_direction->contents.North) {
          q0 = (int)(a + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          a = (unsigned char)q0;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (varargin_2 == g_direction->contents.East) {
          q0 = (int)(k + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          k = (unsigned char)q0;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (varargin_2 == g_direction->contents.South) {
          qY = a - 1U;
          if (qY > a) {
            qY = 0U;
          }

          a = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (varargin_2 == g_direction->contents.West) {
          qY = k - 1U;
          if (qY > k) {
            qY = 0U;
          }

          k = (unsigned char)qY;

          /* disp("west_step") */
        }

        current_x->contents = k;
        current_y->contents = a;

        /* disp("back") */
        break;

       case 3:
        k = current_dir->contents;

        /* 入力　現在方向 */
        /* 出力　現在方向 */
        /*  turn_conclk_90deg 反時計周りに90度回る関数  */
        q0 = (int)(4U + k);
        if ((unsigned int)q0 > 255U) {
          q0 = 255;
        }

        current_dir->contents = (unsigned char)((q0 - 1) % 4);
        k = current_x->contents;
        a = current_y->contents;
        varargin_2 = current_dir->contents;

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (varargin_2 == g_direction->contents.North) {
          q0 = (int)(a + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          a = (unsigned char)q0;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (varargin_2 == g_direction->contents.East) {
          q0 = (int)(k + 1U);
          if ((unsigned int)q0 > 255U) {
            q0 = 255;
          }

          k = (unsigned char)q0;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (varargin_2 == g_direction->contents.South) {
          qY = a - 1U;
          if (qY > a) {
            qY = 0U;
          }

          a = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (varargin_2 == g_direction->contents.West) {
          qY = k - 1U;
          if (qY > k) {
            qY = 0U;
          }

          k = (unsigned char)qY;

          /* disp("west_step") */
        }

        current_x->contents = k;
        current_y->contents = a;

        /* disp("left") */
        break;
      }

      /* for code generation */
    }
  } while (exitg1 == 0);
}

/*
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : const coder_internal_ref_4 *g_direction
 *                const coder_internal_ref_2 *wall
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                const unsigned char maze_goal[18]
 *                unsigned char l_goal_size
 *                const unsigned char maze_wall[1024]
 *                unsigned char contour_map[1024]
 *                unsigned char *max_length
 * Return Type  : void
 */
static void make_map_find(const coder_internal_ref_4 *g_direction, const
  coder_internal_ref_2 *wall, unsigned char maze_row_size, unsigned char
  maze_col_size, const unsigned char maze_goal[18], unsigned char l_goal_size,
  const unsigned char maze_wall[1024], unsigned char contour_map[1024], unsigned
  char *max_length)
{
  unsigned int qY;
  unsigned int b_qY;
  int q0;
  int i2;
  unsigned char change_flag;
  unsigned char tempi;
  bool exitg1;
  int idx;
  bool exitg2;
  int loop_ub;
  static short ii_data[1024];
  int num_temp_size[1];
  static unsigned short num_temp_data[1024];
  int b_num_temp_size[1];
  static unsigned short b_num_temp_data[1024];
  unsigned short tmp_data[1024];
  int tmp_size[1];
  unsigned short u0;
  unsigned char row_data[1024];
  unsigned char col_data[1024];
  unsigned char k;
  int i3;
  int b_k;
  int i4;
  int i5;
  int c_k;
  int d_k;
  int e_k;

  /*   make_map_find 壁情報から等高線MAPを生成 */
  /*  迷路パラメータ設定 */
  qY = maze_col_size - 1U;
  if (qY > maze_col_size) {
    qY = 0U;
  }

  b_qY = maze_row_size - 1U;
  if (b_qY > maze_row_size) {
    b_qY = 0U;
  }

  q0 = (int)(qY * b_qY);
  if ((unsigned int)q0 > 255U) {
    q0 = 255;
  }

  qY = q0 - 1U;
  if (qY > (unsigned int)q0) {
    qY = 0U;
  }

  *max_length = (unsigned char)qY;

  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* 32マス分mapを保持 */
  for (i2 = 0; i2 < 1024; i2++) {
    contour_map[i2] = (unsigned char)qY;
  }

  /* ゴール座標に0を入力 */
  i2 = l_goal_size;
  for (q0 = 0; q0 < i2; q0++) {
    contour_map[(maze_goal[q0 + 9] + ((maze_goal[q0] - 1) << 5)) - 1] = 0U;
  }

  do {
    change_flag = 0U;

    /* map更新確認用フラグ */
    tempi = 0U;
    exitg1 = false;
    while ((!exitg1) && (tempi <= (unsigned char)qY)) {
      /* 歩数カウントは0~max_length */
      /* 歩数が確定している座標を検索 */
      /* 最初は0,更新され、増加したマスを次々検索していく */
      idx = 0;
      q0 = 0;
      exitg2 = false;
      while ((!exitg2) && (q0 < 1024)) {
        if (contour_map[q0] == tempi) {
          idx++;
          ii_data[idx - 1] = (short)(q0 + 1);
          if (idx >= 1024) {
            exitg2 = true;
          } else {
            q0++;
          }
        } else {
          q0++;
        }
      }

      if (1 > idx) {
        loop_ub = 0;
        q0 = 0;
      } else {
        loop_ub = idx;
        q0 = idx;
      }

      num_temp_size[0] = loop_ub;
      for (i2 = 0; i2 < loop_ub; i2++) {
        num_temp_data[i2] = (unsigned short)ii_data[i2];
      }

      /* 32行なので、行番号:32で割ったあまり */
      b_num_temp_size[0] = q0;
      for (i2 = 0; i2 < q0; i2++) {
        b_num_temp_data[i2] = (unsigned short)(num_temp_data[i2] - 1U);
      }

      b_rem(b_num_temp_data, b_num_temp_size, tmp_data, tmp_size);
      idx = tmp_size[0];
      loop_ub = tmp_size[0];
      for (i2 = 0; i2 < loop_ub; i2++) {
        b_qY = tmp_data[i2] + 1U;
        if (b_qY > 65535U) {
          b_qY = 65535U;
        }

        u0 = (unsigned short)b_qY;
        if ((unsigned short)b_qY > 255) {
          u0 = 255U;
        }

        row_data[i2] = (unsigned char)u0;
      }

      /* 列番号:32で割ったときの商 */
      for (i2 = 0; i2 < q0; i2++) {
        num_temp_data[i2]--;
      }

      idivide(num_temp_data, num_temp_size, tmp_data, tmp_size);
      loop_ub = tmp_size[0];
      for (i2 = 0; i2 < loop_ub; i2++) {
        b_qY = tmp_data[i2] + 1U;
        if (b_qY > 65535U) {
          b_qY = 65535U;
        }

        u0 = (unsigned short)b_qY;
        if ((unsigned short)b_qY > 255) {
          u0 = 255U;
        }

        col_data[i2] = (unsigned char)u0;
      }

      /* 見つかったマスの数 */
      if (idx < 0) {
        idx = 0;
      } else {
        if (idx > 255) {
          idx = 255;
        }
      }

      /* 更新マスが見つからなければ終了 */
      if ((unsigned char)idx == 0) {
        exitg1 = true;
      } else {
        /* 検索した座標に対し、歩数mapを更新 */
        i2 = (unsigned char)idx;
        for (q0 = 0; q0 < i2; q0++) {
          /* 北側 */
          k = g_direction->contents.North;
          idx = (col_data[q0] - 1) << 5;
          loop_ub = row_data[q0] + idx;
          i3 = maze_wall[loop_ub - 1];
          if (k <= 7) {
            b_k = (unsigned char)(1 << k);
          } else {
            b_k = 0;
          }

          if ((i3 & b_k) == wall->contents.nowall) {
            /* 北側のMAPが更新されているか判断、されていなければ書き込み */
            i4 = (int)(row_data[q0] + 1U);
            i5 = i4;
            if ((unsigned int)i4 > 255U) {
              i5 = 255;
            }

            if (contour_map[(i5 + idx) - 1] == (unsigned char)qY) {
              if ((unsigned int)i4 > 255U) {
                i4 = 255;
              }

              contour_map[(i4 + idx) - 1] = (unsigned char)(tempi + 1);
              change_flag = 1U;
            }
          }

          /* 東側 */
          k = g_direction->contents.East;
          if (k <= 7) {
            c_k = (unsigned char)(1 << k);
          } else {
            c_k = 0;
          }

          if ((i3 & c_k) == wall->contents.nowall) {
            /* 東側のMAPが更新されているか判断、されていなければ書き込み */
            idx = (int)(col_data[q0] + 1U);
            i4 = idx;
            if ((unsigned int)idx > 255U) {
              i4 = 255;
            }

            if (contour_map[(row_data[q0] + ((i4 - 1) << 5)) - 1] == (unsigned
                 char)qY) {
              if ((unsigned int)idx > 255U) {
                idx = 255;
              }

              contour_map[(row_data[q0] + ((idx - 1) << 5)) - 1] = (unsigned
                char)(tempi + 1);
              change_flag = 1U;
            }
          }

          /* 南側 */
          k = g_direction->contents.South;
          if (k <= 7) {
            d_k = (unsigned char)(1 << k);
          } else {
            d_k = 0;
          }

          if ((i3 & d_k) == wall->contents.nowall) {
            /* 南側のMAPが更新されているか判断、されていなければ書き込み */
            idx = loop_ub - 2;
            if (contour_map[idx] == (unsigned char)qY) {
              contour_map[idx] = (unsigned char)(tempi + 1);
              change_flag = 1U;
            }
          }

          /* 西側 */
          k = g_direction->contents.West;
          if (k <= 7) {
            e_k = (unsigned char)(1 << k);
          } else {
            e_k = 0;
          }

          if ((i3 & e_k) == wall->contents.nowall) {
            /* 西側のMAPが更新されているか判断、されていなければ書き込み */
            idx = (row_data[q0] + ((col_data[q0] - 2) << 5)) - 1;
            if (contour_map[idx] == (unsigned char)qY) {
              contour_map[idx] = (unsigned char)(tempi + 1);
              change_flag = 1U;
            }
          }
        }

        tempi++;
      }
    }

    /* 更新がなければ終了     */
  } while (!(change_flag == 0));
}

/*
 * 未知壁の領域は仮想壁をおいて侵入しない。
 * 入力 迷路縦サイズ,迷路横サイズ,ゴール座標,迷路情報(16進数),迷路探索情報(16進数)
 * 出力 等高線map,最大経路長
 * Arguments    : const coder_internal_ref *goal_size
 *                const coder_internal_ref_4 *g_direction
 *                const coder_internal_ref_2 *wall
 *                const coder_internal_ref_1 *search
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                const unsigned char maze_goal[18]
 *                const unsigned char maze_wall[1024]
 *                const unsigned char maze_wall_search[1024]
 *                unsigned char contour_map[1024]
 *                unsigned char *max_length
 * Return Type  : void
 */
static void make_map_fustrun(const coder_internal_ref *goal_size, const
  coder_internal_ref_4 *g_direction, const coder_internal_ref_2 *wall, const
  coder_internal_ref_1 *search, unsigned char maze_row_size, unsigned char
  maze_col_size, const unsigned char maze_goal[18], const unsigned char
  maze_wall[1024], const unsigned char maze_wall_search[1024], unsigned char
  contour_map[1024], unsigned char *max_length)
{
  unsigned int qY;
  unsigned int b_qY;
  int q0;
  int i6;
  unsigned char change_flag;
  unsigned char tempi;
  int idx;
  bool exitg1;
  int loop_ub;
  static short ii_data[1024];
  int num_temp_size[1];
  static unsigned short num_temp_data[1024];
  int b_num_temp_size[1];
  unsigned short b_num_temp_data[1024];
  unsigned short tmp_data[1024];
  int tmp_size[1];
  unsigned short u1;
  unsigned char row_data[1024];
  unsigned char k;
  unsigned char col_data[1024];
  int i7;
  int b_k;
  int c_k;
  int d_k;
  int i8;
  int e_k;
  int f_k;
  int g_k;
  int h_k;
  int i_k;

  /*  make_map_fustrun 最短走行用等高線MAPを生成 */
  /* ローカル変数設定 */
  /*  迷路パラメータ設定 */
  qY = maze_col_size - 1U;
  if (qY > maze_col_size) {
    qY = 0U;
  }

  b_qY = maze_row_size - 1U;
  if (b_qY > maze_row_size) {
    b_qY = 0U;
  }

  q0 = (int)(qY * b_qY);
  if ((unsigned int)q0 > 255U) {
    q0 = 255;
  }

  qY = q0 - 1U;
  if (qY > (unsigned int)q0) {
    qY = 0U;
  }

  *max_length = (unsigned char)qY;

  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* MAPの初期化(すべての要素にmax_lengthを入力) */
  /* 32マス分mapを保持 */
  for (i6 = 0; i6 < 1024; i6++) {
    contour_map[i6] = (unsigned char)qY;
  }

  /* ゴール座標に0を入力 */
  change_flag = goal_size->contents;
  i6 = change_flag;
  for (q0 = 0; q0 < i6; q0++) {
    contour_map[(maze_goal[q0 + 9] + ((maze_goal[q0] - 1) << 5)) - 1] = 0U;
  }

  do {
    change_flag = 0U;

    /* map更新確認用フラグ */
    for (tempi = 0; tempi <= *max_length; tempi++) {
      /* 歩数カウントは0~max_length */
      /* 歩数が確定している座標を検索 */
      /* 最初は0,更新され、増加したマスを次々検索していく */
      idx = 0;
      q0 = 0;
      exitg1 = false;
      while ((!exitg1) && (q0 < 1024)) {
        if (contour_map[q0] == tempi) {
          idx++;
          ii_data[idx - 1] = (short)(q0 + 1);
          if (idx >= 1024) {
            exitg1 = true;
          } else {
            q0++;
          }
        } else {
          q0++;
        }
      }

      if (1 > idx) {
        loop_ub = 0;
        q0 = 0;
      } else {
        loop_ub = idx;
        q0 = idx;
      }

      num_temp_size[0] = loop_ub;
      for (i6 = 0; i6 < loop_ub; i6++) {
        num_temp_data[i6] = (unsigned short)ii_data[i6];
      }

      /* 32行なので、行番号:32で割ったあまり */
      b_num_temp_size[0] = q0;
      for (i6 = 0; i6 < q0; i6++) {
        b_num_temp_data[i6] = (unsigned short)(num_temp_data[i6] - 1U);
      }

      b_rem(b_num_temp_data, b_num_temp_size, tmp_data, tmp_size);
      loop_ub = tmp_size[0];
      for (i6 = 0; i6 < loop_ub; i6++) {
        b_qY = tmp_data[i6] + 1U;
        if (b_qY > 65535U) {
          b_qY = 65535U;
        }

        u1 = (unsigned short)b_qY;
        if ((unsigned short)b_qY > 255) {
          u1 = 255U;
        }

        row_data[i6] = (unsigned char)u1;
      }

      /* 列番号:32で割ったときの商 */
      for (i6 = 0; i6 < q0; i6++) {
        num_temp_data[i6]--;
      }

      idivide(num_temp_data, num_temp_size, tmp_data, tmp_size);
      q0 = tmp_size[0];
      for (i6 = 0; i6 < q0; i6++) {
        b_qY = tmp_data[i6] + 1U;
        if (b_qY > 65535U) {
          b_qY = 65535U;
        }

        u1 = (unsigned short)b_qY;
        if ((unsigned short)b_qY > 255) {
          u1 = 255U;
        }

        col_data[i6] = (unsigned char)u1;
      }

      /* 見つかったマスの数 */
      /* 検索した座標に対し、歩数mapを更新 */
      for (q0 = 0; q0 < loop_ub; q0++) {
        /* 北側 */
        /* 壁が無い & 探索済みであるとき */
        k = g_direction->contents.North;
        i6 = (unsigned short)(1 + q0) - 1;
        idx = (col_data[i6] - 1) << 5;
        i6 = row_data[i6] + idx;
        i7 = i6 - 1;
        if (k <= 7) {
          b_k = (unsigned char)(1 << k);
        } else {
          b_k = 0;
        }

        if (((maze_wall[i7] & b_k) != 0) == wall->contents.nowall) {
          k = g_direction->contents.North;
          if (k <= 7) {
            c_k = (unsigned char)(1 << k);
          } else {
            c_k = 0;
          }

          if (((maze_wall_search[i7] & c_k) != 0) == search->contents.known) {
            /* 北側のMAPが更新されているか判断、されていなければ書き込み */
            i7 = (int)(row_data[(unsigned short)(1 + q0) - 1] + 1U);
            i8 = i7;
            if ((unsigned int)i7 > 255U) {
              i8 = 255;
            }

            if (contour_map[(i8 + idx) - 1] == (unsigned char)qY) {
              if ((unsigned int)i7 > 255U) {
                i7 = 255;
              }

              contour_map[(i7 + idx) - 1] = (unsigned char)(tempi + 1);
              change_flag = 1U;
            }
          }
        }

        /* 東側 */
        k = g_direction->contents.East;
        if (k <= 7) {
          d_k = (unsigned char)(1 << k);
        } else {
          d_k = 0;
        }

        if (((maze_wall[(row_data[(unsigned short)(1 + q0) - 1] + ((col_data
                 [(unsigned short)(1 + q0) - 1] - 1) << 5)) - 1] & d_k) != 0) ==
            wall->contents.nowall) {
          k = g_direction->contents.East;
          if (k <= 7) {
            e_k = (unsigned char)(1 << k);
          } else {
            e_k = 0;
          }

          if (((maze_wall_search[(row_data[(unsigned short)(1 + q0) - 1] +
                                  ((col_data[(unsigned short)(1 + q0) - 1] - 1) <<
                  5)) - 1] & e_k) != 0) == search->contents.known) {
            /* 東側のMAPが更新されているか判断、されていなければ書き込み */
            idx = (int)(col_data[(unsigned short)(1 + q0) - 1] + 1U);
            i7 = idx;
            if ((unsigned int)idx > 255U) {
              i7 = 255;
            }

            if (contour_map[(row_data[(unsigned short)(1 + q0) - 1] + ((i7 - 1) <<
                  5)) - 1] == (unsigned char)qY) {
              if ((unsigned int)idx > 255U) {
                idx = 255;
              }

              contour_map[(row_data[(unsigned short)(1 + q0) - 1] + ((idx - 1) <<
                5)) - 1] = (unsigned char)(tempi + 1);
              change_flag = 1U;
            }
          }
        }

        /* 南側 */
        k = g_direction->contents.South;
        if (k <= 7) {
          f_k = (unsigned char)(1 << k);
        } else {
          f_k = 0;
        }

        if (((maze_wall[(row_data[(unsigned short)(1 + q0) - 1] + ((col_data
                 [(unsigned short)(1 + q0) - 1] - 1) << 5)) - 1] & f_k) != 0) ==
            wall->contents.nowall) {
          k = g_direction->contents.South;
          if (k <= 7) {
            g_k = (unsigned char)(1 << k);
          } else {
            g_k = 0;
          }

          if (((maze_wall_search[(row_data[(unsigned short)(1 + q0) - 1] +
                                  ((col_data[(unsigned short)(1 + q0) - 1] - 1) <<
                  5)) - 1] & g_k) != 0) == search->contents.known) {
            /* 南側のMAPが更新されているか判断、されていなければ書き込み */
            i6 -= 2;
            if (contour_map[i6] == (unsigned char)qY) {
              contour_map[i6] = (unsigned char)(tempi + 1);
              change_flag = 1U;
            }
          }
        }

        /* 西側 */
        k = g_direction->contents.West;
        if (k <= 7) {
          h_k = (unsigned char)(1 << k);
        } else {
          h_k = 0;
        }

        if (((maze_wall[(row_data[(unsigned short)(1 + q0) - 1] + ((col_data
                 [(unsigned short)(1 + q0) - 1] - 1) << 5)) - 1] & h_k) != 0) ==
            wall->contents.nowall) {
          k = g_direction->contents.West;
          if (k <= 7) {
            i_k = (unsigned char)(1 << k);
          } else {
            i_k = 0;
          }

          if (((maze_wall_search[(row_data[(unsigned short)(1 + q0) - 1] +
                                  ((col_data[(unsigned short)(1 + q0) - 1] - 1) <<
                  5)) - 1] & i_k) != 0) == search->contents.known) {
            /* 西側のMAPが更新されているか判断、されていなければ書き込み */
            i6 = (row_data[(unsigned short)(1 + q0) - 1] + ((col_data[(unsigned
                     short)(1 + q0) - 1] - 2) << 5)) - 1;
            if (contour_map[i6] == (unsigned char)qY) {
              contour_map[i6] = (unsigned char)(tempi + 1);
              change_flag = 1U;
            }
          }
        }
      }
    }

    /* 更新がなければ終了     */
  } while (!(change_flag == 0));
}

/*
 * 入力 現在位置x,y,現在方向
 * 出力 現在位置x,y
 * Arguments    : const coder_internal_ref_4 *g_direction
 *                unsigned char *current_x
 *                unsigned char *current_y
 *                unsigned char current_dir
 * Return Type  : void
 */
static void move_step(const coder_internal_ref_4 *g_direction, unsigned char
                      *current_x, unsigned char *current_y, unsigned char
                      current_dir)
{
  int q0;
  unsigned int qY;

  /*  move_step 一マス前進する関数 */
  /* 北に一マス */
  if (current_dir == g_direction->contents.North) {
    q0 = (int)(*current_y + 1U);
    if ((unsigned int)q0 > 255U) {
      q0 = 255;
    }

    *current_y = (unsigned char)q0;

    /* disp("north_step") */
  }

  /* 東に一マス */
  if (current_dir == g_direction->contents.East) {
    q0 = (int)(*current_x + 1U);
    if ((unsigned int)q0 > 255U) {
      q0 = 255;
    }

    *current_x = (unsigned char)q0;

    /* disp("east_step") */
  }

  /* 南に一マス */
  if (current_dir == g_direction->contents.South) {
    q0 = *current_y;
    qY = q0 - 1U;
    if (qY > (unsigned int)q0) {
      qY = 0U;
    }

    *current_y = (unsigned char)qY;

    /* disp("south_step") */
  }

  /* 西に一マス */
  if (current_dir == g_direction->contents.West) {
    q0 = *current_x;
    qY = q0 - 1U;
    if (qY > (unsigned int)q0) {
      qY = 0U;
    }

    *current_x = (unsigned char)qY;

    /* disp("west_step") */
  }
}

/*
 * 入力　現在位置x,y,現在方向,迷路行方向サイズ,迷路列方向サイズ,迷路壁情報,迷路壁の探索情報,ゴール座標
 * 出力  現在位置x,y,現在方向,壁情報,探索情報
 * Arguments    : const coder_internal_ref_2 *wall
 *                const coder_internal_ref_1 *search
 *                const coder_internal_ref_4 *g_direction
 *                const coder_internal_ref_3 *l_direction
 *                unsigned char *current_x
 *                unsigned char *current_y
 *                unsigned char *current_dir
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 *                const unsigned char maze_goal[18]
 *                unsigned char l_goal_size
 * Return Type  : void
 */
static void search_adachi(const coder_internal_ref_2 *wall, const
  coder_internal_ref_1 *search, const coder_internal_ref_4 *g_direction, const
  coder_internal_ref_3 *l_direction, unsigned char *current_x, unsigned char
  *current_y, unsigned char *current_dir, unsigned char maze_row_size, unsigned
  char maze_col_size, unsigned char maze_wall[1024], unsigned char
  maze_wall_search[1024], const unsigned char maze_goal[18], unsigned char
  l_goal_size)
{
  unsigned char goal_flag;
  int i9;
  int exitg1;
  unsigned char wall_write[4];
  unsigned char serch_write[4];
  int i10;
  unsigned char k;
  int b_k;
  int i11;
  int i12;
  int c_k;
  int d_k;
  int e_k;
  int f_k;
  int g_k;
  int h_k;
  int i_k;
  unsigned int qY;
  int j_k;
  int q0;
  int k_k;
  int i13;
  int l_k;
  int i14;
  int m_k;
  unsigned char cmap[1024];
  unsigned char little;
  int n_k;
  unsigned int b_qY;
  unsigned char next_dir;
  int o_k;
  int p_k;
  int q_k;
  int r_k;
  int s_k;
  int t_k;
  int u_k;
  unsigned char varargin_3;
  unsigned char varargin_4;

  /*  search_adachi 足立法での探索 */
  /* local変数宣言 */
  goal_flag = 0U;

  /* 探索開始時x */
  /* 探索開始時y */
  /* for C gen */
  i9 = l_goal_size;
  do {
    exitg1 = 0;

    /* 壁情報取得 */
    /* matlab上では画像から取得した壁情報を参照する。 */
    /* 入力:画像から得た迷路情報,迷路行方向壁枚数,迷路列方向壁枚数,  */
    /*      現在地座標x,y,現在進行方向,迷路壁情報,迷路壁探索情報             */
    /* 出力:迷路壁情報,迷路壁探索情報  */
    /*   wall_set 壁情報取得 */
    /* グローバル変数(matlabでは迷路データを、Cでは壁センサ値を参照) */
    /* for matlab */
    /* for C gen */
    /* 壁センサAD値格納変数 */
    /* 壁センサ閾値 */
    /* ローカル変数宣言 */
    /* 壁情報書き込み用バッファ(N,E,S,W) */
    wall_write[0] = 0U;
    serch_write[0] = 0U;
    wall_write[1] = 0U;
    serch_write[1] = 0U;
    wall_write[2] = 0U;
    serch_write[2] = 0U;
    wall_write[3] = 0U;
    serch_write[3] = 0U;

    /* 探索情報書き込み用バッファ(N,E,S,W) */
    /* マウスの方向に基づく壁情報取得 */
    /* マウスの方向と絶対方向の差=マウスの方向となることを利用し、 */
    /* 絶対角度と整合をとる。 */
    /*  実機はここをセンサ値に対応させる  */
    wall_sensor_front = (Sensor_GetValue(0) + Sensor_GetValue(3))/2;
    wall_sensor_right =  Sensor_GetValue(2);
    wall_sensor_left =  Sensor_GetValue(1);
    /* センサ値取得(要手直し) */
    /* ここまで */
    /* 前方の壁判定 */
    /* for Cgen */
    /* センサ値をもとに、壁の有無を判定 */


    if (wall_sensor_front > wall_sensor_front_th) {
      /* 壁情報取得 */
      wall_write[*current_dir % 4] = wall->contents.wall;
      front_calib_flg = 1; //m 前壁補正フラグを立てる
    }

    /* 探索情報取更新 */
    serch_write[*current_dir % 4] = search->contents.known;

    /* 右壁判定 */
    /* for Cgen */
    /* センサ値をもとに、壁の有無を判定 */
    if (wall_sensor_right > wall_sensor_right_th) {
      /* 壁情報取得 */
      i10 = (int)(*current_dir + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      wall_write[i10 % 4] = wall->contents.wall;
      right_calib_flg = 1; //m 前壁補正フラグを立てる
    }

    /* 探索情報取更新 */
    i10 = (int)(*current_dir + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    serch_write[i10 % 4] = search->contents.known;

    /* 後方は情報を得ることができないので処理しない。 */
    /* 左壁判定 */
    /* for Cgen */
    /* センサ値をもとに、壁の有無を判定 */
    if (wall_sensor_left > wall_sensor_left_th) {
      /* 壁情報取得 */
      i10 = (int)(*current_dir + 3U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      wall_write[i10 % 4] = wall->contents.wall;
      left_calib_flg = 1; //m 前壁補正フラグを立てる

    }

    /* 探索情報取更新 */
    i10 = (int)(*current_dir + 3U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    serch_write[i10 % 4] = search->contents.known;

    /* ここまで */
    /* 壁情報,探索情報を入力 */
    /* 北側 */
    k = g_direction->contents.North;
    i10 = (int)(g_direction->contents.North + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      b_k = (unsigned char)(1 << k);
    } else {
      b_k = 0;
    }

    i10 = (int)((unsigned int)b_k * wall_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    i11 = (*current_x - 1) << 5;
    i12 = (*current_y + i11) - 1;
    maze_wall[i12] |= (unsigned char)i10;
    k = g_direction->contents.North;
    i10 = (int)(g_direction->contents.North + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      c_k = (unsigned char)(1 << k);
    } else {
      c_k = 0;
    }

    i10 = (int)((unsigned int)c_k * serch_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    maze_wall_search[i12] |= (unsigned char)i10;

    /* 東側 */
    k = g_direction->contents.East;
    i10 = (int)(g_direction->contents.East + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      d_k = (unsigned char)(1 << k);
    } else {
      d_k = 0;
    }

    i10 = (int)((unsigned int)d_k * wall_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    maze_wall[i12] |= (unsigned char)i10;
    k = g_direction->contents.East;
    i10 = (int)(g_direction->contents.East + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      e_k = (unsigned char)(1 << k);
    } else {
      e_k = 0;
    }

    i10 = (int)((unsigned int)e_k * serch_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    maze_wall_search[i12] |= (unsigned char)i10;

    /* 南側 */
    k = g_direction->contents.South;
    i10 = (int)(g_direction->contents.South + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      f_k = (unsigned char)(1 << k);
    } else {
      f_k = 0;
    }

    i10 = (int)((unsigned int)f_k * wall_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    maze_wall[i12] |= (unsigned char)i10;
    k = g_direction->contents.South;
    i10 = (int)(g_direction->contents.South + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      g_k = (unsigned char)(1 << k);
    } else {
      g_k = 0;
    }

    i10 = (int)((unsigned int)g_k * serch_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    maze_wall_search[i12] |= (unsigned char)i10;

    /* 西側 */
    k = g_direction->contents.West;
    i10 = (int)(g_direction->contents.West + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      h_k = (unsigned char)(1 << k);
    } else {
      h_k = 0;
    }

    i10 = (int)((unsigned int)h_k * wall_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    maze_wall[i12] |= (unsigned char)i10;
    k = g_direction->contents.West;
    i10 = (int)(g_direction->contents.West + 1U);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    if (k <= 7) {
      i_k = (unsigned char)(1 << k);
    } else {
      i_k = 0;
    }

    i10 = (int)((unsigned int)i_k * serch_write[i10 - 1]);
    if ((unsigned int)i10 > 255U) {
      i10 = 255;
    }

    maze_wall_search[i12] |= (unsigned char)i10;

    /* 隣り合うマスの情報にも入力 */
    /* 北側のマスの南側の壁情報 */
    qY = maze_row_size - 1U;
    if (qY > maze_row_size) {
      qY = 0U;
    }

    if (*current_y < (int)qY) {
      k = g_direction->contents.South;
      i10 = (int)(g_direction->contents.North + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        j_k = (unsigned char)(1 << k);
      } else {
        j_k = 0;
      }

      i10 = (int)((unsigned int)j_k * wall_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      q0 = (int)(*current_y + 1U);
      i13 = q0;
      if ((unsigned int)q0 > 255U) {
        i13 = 255;
      }

      i14 = q0;
      if ((unsigned int)q0 > 255U) {
        i14 = 255;
      }

      maze_wall[(i13 + i11) - 1] = (unsigned char)(maze_wall[(i14 + i11) - 1] |
        (unsigned char)i10);
      k = g_direction->contents.South;
      i10 = (int)(g_direction->contents.North + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        n_k = (unsigned char)(1 << k);
      } else {
        n_k = 0;
      }

      i10 = (int)((unsigned int)n_k * serch_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      i13 = q0;
      if ((unsigned int)q0 > 255U) {
        i13 = 255;
        q0 = 255;
      }

      maze_wall_search[(i13 + i11) - 1] = (unsigned char)(maze_wall_search[(q0 +
        i11) - 1] | (unsigned char)i10);
    }

    /* 東側のマスの西側の壁情報 */
    qY = maze_col_size - 1U;
    if (qY > maze_col_size) {
      qY = 0U;
    }

    if (*current_x < (int)qY) {
      k = g_direction->contents.West;
      i10 = (int)(g_direction->contents.East + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        k_k = (unsigned char)(1 << k);
      } else {
        k_k = 0;
      }

      i10 = (int)((unsigned int)k_k * wall_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      q0 = (int)(*current_x + 1U);
      i13 = q0;
      if ((unsigned int)q0 > 255U) {
        i13 = 255;
      }

      i14 = q0;
      if ((unsigned int)q0 > 255U) {
        i14 = 255;
      }

      maze_wall[(*current_y + ((i13 - 1) << 5)) - 1] = (unsigned char)
        (maze_wall[(*current_y + ((i14 - 1) << 5)) - 1] | (unsigned char)i10);
      k = g_direction->contents.West;
      i10 = (int)(g_direction->contents.East + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        o_k = (unsigned char)(1 << k);
      } else {
        o_k = 0;
      }

      i10 = (int)((unsigned int)o_k * serch_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      i13 = q0;
      if ((unsigned int)q0 > 255U) {
        i13 = 255;
        q0 = 255;
      }

      maze_wall_search[(*current_y + ((i13 - 1) << 5)) - 1] = (unsigned char)
        (maze_wall_search[(*current_y + ((q0 - 1) << 5)) - 1] | (unsigned char)
         i10);
    }

    /* 南側のマスの北側の壁情報 */
    if (*current_y > 1) {
      k = g_direction->contents.North;
      i10 = (int)(g_direction->contents.South + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        l_k = (unsigned char)(1 << k);
      } else {
        l_k = 0;
      }

      i10 = (int)((unsigned int)l_k * wall_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      q0 = *current_y;
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      q0 = *current_y;
      b_qY = q0 - 1U;
      if (b_qY > (unsigned int)q0) {
        b_qY = 0U;
      }

      maze_wall[((int)qY + i11) - 1] = (unsigned char)(maze_wall[((int)b_qY +
        i11) - 1] | (unsigned char)i10);
      k = g_direction->contents.North;
      i10 = (int)(g_direction->contents.South + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        q_k = (unsigned char)(1 << k);
      } else {
        q_k = 0;
      }

      i10 = (int)((unsigned int)q_k * serch_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      q0 = *current_y;
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      q0 = *current_y;
      b_qY = q0 - 1U;
      if (b_qY > (unsigned int)q0) {
        b_qY = 0U;
      }

      maze_wall_search[((int)qY + i11) - 1] = (unsigned char)(maze_wall_search
        [((int)b_qY + i11) - 1] | (unsigned char)i10);
    }

    /* 西側のマスの東側の壁情報 */
    if (*current_x > 1) {
      k = g_direction->contents.East;
      i10 = (int)(g_direction->contents.West + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        m_k = (unsigned char)(1 << k);
      } else {
        m_k = 0;
      }

      i10 = (int)((unsigned int)m_k * wall_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      q0 = *current_x;
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      q0 = *current_x;
      b_qY = q0 - 1U;
      if (b_qY > (unsigned int)q0) {
        b_qY = 0U;
      }

      maze_wall[(*current_y + (((int)qY - 1) << 5)) - 1] = (unsigned char)
        (maze_wall[(*current_y + (((int)b_qY - 1) << 5)) - 1] | (unsigned char)
         i10);
      k = g_direction->contents.East;
      i10 = (int)(g_direction->contents.West + 1U);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      if (k <= 7) {
        r_k = (unsigned char)(1 << k);
      } else {
        r_k = 0;
      }

      i10 = (int)((unsigned int)r_k * serch_write[i10 - 1]);
      if ((unsigned int)i10 > 255U) {
        i10 = 255;
      }

      q0 = *current_x;
      qY = q0 - 1U;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      q0 = *current_x;
      b_qY = q0 - 1U;
      if (b_qY > (unsigned int)q0) {
        b_qY = 0U;
      }

      maze_wall_search[(*current_y + (((int)qY - 1) << 5)) - 1] = (unsigned char)
        (maze_wall_search[(*current_y + (((int)b_qY - 1) << 5)) - 1] | (unsigned
          char)i10);
    }

    /* m現在位置がゴールか判定 */
    for (q0 = 0; q0 < i9; q0++) {
      if ((*current_x == maze_goal[q0]) && (*current_y == maze_goal[q0 + 9])) {
        goal_flag = 1U;
      }
    }

    if (goal_flag == 1) {
      exitg1 = 1;
      half_deceleration();


    } else {
      /*  等高線MAP生成 */
      /*  [contour_map,max_length] = make_map2(maze_row_size,maze_col_size,maze_goal,maze_wall); */
      make_map_find(g_direction, wall, maze_row_size, maze_col_size, maze_goal,
                    l_goal_size, maze_wall, cmap, &little);

      /*  進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      /*  入力 現在地x,y,壁情報,等高線map,最大経路長 */
      /*  出力 次の進行方角 */
      /*  get_nextdir2 等高線mapから次に向かう方向を選択 */
      /* 出力の初期化 */
      next_dir = 0U;

      /*             %%進行方向選定 */
      /* 優先順位　北⇒東⇒南⇒西 */
      /* 北側の壁のありなし判定 */
      k = g_direction->contents.North;
      if (k <= 7) {
        p_k = (unsigned char)(1 << k);
      } else {
        p_k = 0;
      }

      if ((maze_wall[i12] & p_k) == 0) {
        /* 北側の等高線mapが閾値より低ければ、 */
        i10 = (int)(*current_y + 1U);
        if ((unsigned int)i10 > 255U) {
          i10 = 255;
        }

        if (cmap[(i10 + i11) - 1] < little) {
          /* 閾値を北側の等高map値に変更 */
          i10 = (int)(*current_y + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          little = cmap[(i10 + i11) - 1];

          /* 北側を進行方向に変更y */
          next_dir = g_direction->contents.North;
        }
      }

      /* 東側 */
      k = g_direction->contents.East;
      if (k <= 7) {
        s_k = (unsigned char)(1 << k);
      } else {
        s_k = 0;
      }

      if ((maze_wall[(*current_y + ((*current_x - 1) << 5)) - 1] & s_k) == 0) {
        i10 = (int)(*current_x + 1U);
        if ((unsigned int)i10 > 255U) {
          i10 = 255;
        }

        if (cmap[(*current_y + ((i10 - 1) << 5)) - 1] < little) {
          i10 = (int)(*current_x + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          little = cmap[(*current_y + ((i10 - 1) << 5)) - 1];
          next_dir = g_direction->contents.East;
        }
      }

      /* 南側 */
      k = g_direction->contents.South;
      if (k <= 7) {
        t_k = (unsigned char)(1 << k);
      } else {
        t_k = 0;
      }

      if ((maze_wall[(*current_y + ((*current_x - 1) << 5)) - 1] & t_k) == 0) {
        q0 = *current_y;
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        if (cmap[((int)qY + i11) - 1] < little) {
          q0 = *current_y;
          qY = q0 - 1U;
          if (qY > (unsigned int)q0) {
            qY = 0U;
          }

          little = cmap[((int)qY + i11) - 1];
          next_dir = g_direction->contents.South;
        }
      }

      /* 西側 */
      k = g_direction->contents.West;
      if (k <= 7) {
        u_k = (unsigned char)(1 << k);
      } else {
        u_k = 0;
      }

      if ((maze_wall[(*current_y + ((*current_x - 1) << 5)) - 1] & u_k) == 0) {
        q0 = *current_x;
        qY = q0 - 1U;
        if (qY > (unsigned int)q0) {
          qY = 0U;
        }

        if (cmap[(*current_y + (((int)qY - 1) << 5)) - 1] < little) {
          /*  little = contour_map(current_y,current_x-1); */
          next_dir = g_direction->contents.West;
        }
      }

      /*  現在方向と進行方向に応じた処理 */
      q0 = (int)(4U + next_dir);
      if ((unsigned int)q0 > 255U) {
        q0 = 255;
      }

      qY = (unsigned int)q0 - *current_dir;
      if (qY > (unsigned int)q0) {
        qY = 0U;
      }

      k = (unsigned char)((int)qY % 4);
      next_dir = l_direction->contents.front;
      little = l_direction->contents.right;
      varargin_3 = l_direction->contents.back;
      varargin_4 = l_direction->contents.left;
      if (next_dir == k) {
        q0 = 0;
      } else if (little == k) {
        q0 = 1;
      } else if (varargin_3 == k) {
        q0 = 2;
      } else if (varargin_4 == k) {
        q0 = 3;
      } else {
        q0 = -1;
      }

      switch (q0) {
       case 0:	//前進時
    	move_front();
        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (*current_dir == g_direction->contents.North) {
          i10 = (int)(*current_y + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_y = (unsigned char)i10;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (*current_dir == g_direction->contents.East) {
          i10 = (int)(*current_x + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_x = (unsigned char)i10;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (*current_dir == g_direction->contents.South) {
          qY = *current_y - 1U;
          if (qY > *current_y) {
            qY = 0U;
          }

          *current_y = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (*current_dir == g_direction->contents.West) {
          qY = *current_x - 1U;
          if (qY > *current_x) {
            qY = 0U;
          }

          *current_x = (unsigned char)qY;

          /* disp("west_step") */
        }

        /* disp("front") */
        break;

       case 1: //右折時
    	move_right();
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*  turn_clk_90deg 時計周りに90度ターンする関数  */
        i10 = (int)(4U + *current_dir);
        if ((unsigned int)i10 > 255U) {
          i10 = 255;
        }

        i10++;
        if ((unsigned int)i10 > 255U) {
          i10 = 255;
        }

        *current_dir = (unsigned char)(i10 % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (*current_dir == g_direction->contents.North) {
          i10 = (int)(*current_y + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_y = (unsigned char)i10;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (*current_dir == g_direction->contents.East) {
          i10 = (int)(*current_x + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_x = (unsigned char)i10;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (*current_dir == g_direction->contents.South) {
          qY = *current_y - 1U;
          if (qY > *current_y) {
            qY = 0U;
          }

          *current_y = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (*current_dir == g_direction->contents.West) {
          qY = *current_x - 1U;
          if (qY > *current_x) {
            qY = 0U;
          }

          *current_x = (unsigned char)qY;

          /* disp("west_step") */
        }

        /* disp("right") */
        break;

       case 2:	//バック
    	move_back();
        /* 入力 現在方向 */
        /* 出力 現在方向 */
        /*  turn_180deg 180度ターンする関数 */
        i10 = (int)(4U + *current_dir);
        if ((unsigned int)i10 > 255U) {
          i10 = 255;
        }

        *current_dir = (unsigned char)((i10 - 2) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (*current_dir == g_direction->contents.North) {
          i10 = (int)(*current_y + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_y = (unsigned char)i10;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (*current_dir == g_direction->contents.East) {
          i10 = (int)(*current_x + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_x = (unsigned char)i10;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (*current_dir == g_direction->contents.South) {
          qY = *current_y - 1U;
          if (qY > *current_y) {
            qY = 0U;
          }

          *current_y = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (*current_dir == g_direction->contents.West) {
          qY = *current_x - 1U;
          if (qY > *current_x) {
            qY = 0U;
          }

          *current_x = (unsigned char)qY;

          /* disp("west_step") */
        }

        /* disp("back") */
        break;

       case 3://左折時
    	move_left();
        /* 入力　現在方向 */
        /* 出力　現在方向 */
        /*  turn_conclk_90deg 反時計周りに90度回る関数  */
        i10 = (int)(4U + *current_dir);
        if ((unsigned int)i10 > 255U) {
          i10 = 255;
        }

        *current_dir = (unsigned char)((i10 - 1) % 4);

        /* 入力 現在位置x,y,現在方向 */
        /* 出力 現在位置x,y */
        /*  move_step 一マス前進する関数 */
        /* 北に一マス */
        if (*current_dir == g_direction->contents.North) {
          i10 = (int)(*current_y + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_y = (unsigned char)i10;

          /* disp("north_step") */
        }

        /* 東に一マス */
        if (*current_dir == g_direction->contents.East) {
          i10 = (int)(*current_x + 1U);
          if ((unsigned int)i10 > 255U) {
            i10 = 255;
          }

          *current_x = (unsigned char)i10;

          /* disp("east_step") */
        }

        /* 南に一マス */
        if (*current_dir == g_direction->contents.South) {
          qY = *current_y - 1U;
          if (qY > *current_y) {
            qY = 0U;
          }

          *current_y = (unsigned char)qY;

          /* disp("south_step") */
        }

        /* 西に一マス */
        if (*current_dir == g_direction->contents.West) {
          qY = *current_x - 1U;
          if (qY > *current_x) {
            qY = 0U;
          }

          *current_x = (unsigned char)qY;

          /* disp("west_step") */
        }

        /* disp("left") */
        break;
      }

      /* for code generation */
    }
  } while (exitg1 == 0);

  /* for code generation */
}

/*
 * maze_solve 実機での迷路探索関数
 * 入力 迷路壁情報,迷路探索情報,迷路縦サイズ,迷路横サイズ,ゴール座標,
 * 出力 壁情報,探索情報
 * Arguments    : unsigned char maze_wall[1024]
 *                unsigned char maze_wall_search[1024]
 *                unsigned char maze_row_size
 *                unsigned char maze_col_size
 *                const unsigned char maze_goal[18]
 *                unsigned char goal_size
 *                unsigned char run_mode
 * Return Type  : void
 */
void maze_solve(unsigned char maze_wall[1024], unsigned char maze_wall_search
                [1024], unsigned char maze_row_size, unsigned char maze_col_size,
                const unsigned char maze_goal[18], unsigned char goal_size,
                unsigned char run_mode)
{
  coder_internal_ref b_goal_size;
  int N;
  unsigned char new_goal[18];
  coder_internal_ref_4 g_direction;
  coder_internal_ref_3 l_direction;
  coder_internal_ref_2 wall;
  coder_internal_ref_1 search;
  coder_internal_ref current_x;
  unsigned char contour_map[1024];
  unsigned char search_flag;
  coder_internal_ref current_y;
  coder_internal_ref current_dir;
  int exitg1;
  bool exitg2;
  unsigned char u2;
  b_goal_size.contents = goal_size;

  /*original parameter*/
  front_calib_flg = 0; //m　前壁補正フラグの初期化
  right_calib_flg = 0; //m　前壁補正フラグの初期化
  left_calib_flg = 0; //m　前壁補正フラグの初期化

  /* ローカル変数宣言 */
  for (N = 0; N < 18; N++) {
    new_goal[N] = 0U;
  }

  /* 絶対方角定義 */
  g_direction.contents.North = 0U;
  g_direction.contents.East = 1U;
  g_direction.contents.South = 2U;
  g_direction.contents.West = 3U;

  /* マウス方向定義 */
  l_direction.contents.front = 0U;
  l_direction.contents.right = 1U;
  l_direction.contents.back = 2U;
  l_direction.contents.left = 3U;

  /* 壁情報定義 */
  wall.contents.nowall = 0U;
  wall.contents.wall = 1U;

  /* 探索情報定義 */
  search.contents.unknown = 0U;
  search.contents.known = 1U;

  /* m走行モード定義 */
  /* m探索時 */
  if (run_mode == 0) {
    /* mマウスの初期位置設定 */
    /* m一マス前進 */

	start_acceleration(  );//mスタート時の加速

    current_x.contents = 1U;
    current_y.contents = 1U;
    move_step(&g_direction, &current_x.contents, &current_y.contents, 0U);
    move_front();

    /* m足立法による探索 */
    current_dir.contents = g_direction.contents.North;
    search_adachi(&wall, &search, &g_direction, &l_direction,
                  &current_x.contents, &current_y.contents,
                  &current_dir.contents, maze_row_size, maze_col_size, maze_wall,
                  maze_wall_search, maze_goal, goal_size);

    HAL_Delay(1000);

    /* mゴールをすべて探索 */
    do {
      exitg1 = 0;
      search_flag = 0U;
      N = 0;
      exitg2 = false;
      while ((!exitg2) && (N <= goal_size - 1)) {
        u2 = maze_goal[N + 9];
        if (maze_wall_search[(u2 + ((maze_goal[N] - 1) << 5)) - 1] != 15) {
          new_goal[0] = maze_goal[N];
          new_goal[9] = u2;
          search_flag = 1U;
          exitg2 = true;
        } else {
          N++;
        }
      }

      if (search_flag == 1) {
    	run_first_flg = 0; //m走行開始フラグのクリア
        search_adachi(&wall, &search, &g_direction, &l_direction,
                      &current_x.contents, &current_y.contents,
                      &current_dir.contents, maze_row_size, maze_col_size,
                      maze_wall, maze_wall_search, new_goal, 1U);
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    HAL_Delay(1000);

    /* mスタートを目的地として足立法で再探索 */
    new_goal[0] = 1U;
    new_goal[9] = 1U;
    run_first_flg = 0; //m走行開始フラグのクリア
    search_adachi(&wall, &search, &g_direction, &l_direction,
                  &current_x.contents, &current_y.contents,
                  &current_dir.contents, maze_row_size, maze_col_size, maze_wall,
                  maze_wall_search, new_goal, 1U);

    fornt_wall_calibrate();
    turn_conclk_180();
    target_distance_m_set(-0.5);
    /* for code generation */
  }

  if (run_mode == 1) {
    /* 探索情報をもとに等高線MAPを生成 */
    make_map_fustrun(&b_goal_size, &g_direction, &wall, &search, maze_row_size,
                     maze_col_size, maze_goal, maze_wall, maze_wall_search,
                     contour_map, &search_flag);

    /* 最短距離走行 */
    fust_run(&g_direction, &current_x, &current_y, &current_dir, &b_goal_size,
             &wall, &l_direction, maze_wall, contour_map, maze_goal, search_flag);
  }
}

/*
 * File trailer for maze_solve.c
 *
 * [EOF]
 */
