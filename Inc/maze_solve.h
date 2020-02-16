/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: maze_solve.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 18-Nov-2019 23:53:15
 */

#ifndef MAZE_SOLVE_H
#define MAZE_SOLVE_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "matlab_code_gen_types.h"

/* Type Definitions */
#ifndef typedef_coder_internal_ref
#define typedef_coder_internal_ref

typedef struct {
  unsigned char contents;
} coder_internal_ref;

#endif                                 /*typedef_coder_internal_ref*/

/*extern var*/
extern int wall_sensor_front;
extern int wall_sensor_right;
extern int wall_sensor_left;
extern coder_internal_ref current_x;
extern coder_internal_ref current_y;



/* Function Declarations */
extern void maze_solve(unsigned char maze_wall[1024], unsigned char
  maze_wall_search[1024], unsigned char maze_row_size, unsigned char
  maze_col_size, const unsigned char maze_goal[18], unsigned char goal_size,
  unsigned char run_mode);

#endif

/*
 * File trailer for maze_solve.h
 *
 * [EOF]
 */
