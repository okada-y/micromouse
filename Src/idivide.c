/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: idivide.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 18-Nov-2019 23:53:15
 */

/* Include Files */
#include <math.h>
#include "maze_init.h"
#include "maze_solve.h"
#include "idivide.h"

/* Function Declarations */
static double rt_roundd_snf(double u);

/* Function Definitions */

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : const unsigned short a_data[]
 *                const int a_size[1]
 *                unsigned short c_data[]
 *                int c_size[1]
 * Return Type  : void
 */
void idivide(const unsigned short a_data[], const int a_size[1], unsigned short
             c_data[], int c_size[1])
{
  int nx;
  int k;
  static double x_data[1024];
  nx = a_size[0];
  for (k = 0; k < nx; k++) {
    x_data[k] = (double)a_data[k] / 32.0;
  }

  nx = a_size[0];
  for (k = 0; k < nx; k++) {
    x_data[k] = trunc(x_data[k]);
  }

  c_size[0] = a_size[0];
  nx = a_size[0];
  for (k = 0; k < nx; k++) {
    c_data[k] = (unsigned short)rt_roundd_snf(x_data[k]);
  }
}

/*
 * File trailer for idivide.c
 *
 * [EOF]
 */
