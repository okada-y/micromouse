/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: matlab_code_gen_rtwutil.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 18-Nov-2019 21:12:53
 */

/* Include Files */
#include <math.h>
#include "matlab_code_gen_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : double u
 * Return Type  : double
 */
double rt_roundd_snf(double u)
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
 * File trailer for matlab_code_gen_rtwutil.c
 *
 * [EOF]
 */
