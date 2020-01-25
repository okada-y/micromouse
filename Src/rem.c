/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: rem.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 18-Nov-2019 23:53:15
 */

/* Include Files */
#include "maze_init.h"
#include "maze_solve.h"
#include "rem.h"

/* Function Definitions */

/*
 * Arguments    : const unsigned short x_data[]
 *                const int x_size[1]
 *                unsigned short r_data[]
 *                int r_size[1]
 * Return Type  : void
 */
void b_rem(const unsigned short x_data[], const int x_size[1], unsigned short
           r_data[], int r_size[1])
{
  int nx;
  int k;
  r_size[0] = (short)x_size[0];
  nx = (short)x_size[0];
  for (k = 0; k < nx; k++) {
    r_data[k] = (unsigned short)(x_data[k] % 32);
  }
}

/*
 * File trailer for rem.c
 *
 * [EOF]
 */
