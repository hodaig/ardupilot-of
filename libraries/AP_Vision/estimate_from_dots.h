/*
 * File: estimate_from_dots.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 15-May-2017 17:21:22
 */

#ifndef ESTIMATE_FROM_DOTS_H
#define ESTIMATE_FROM_DOTS_H


//#include "rt_defines.h"
//#include "rt_nonfinite.h"
//#include "rtwtypes.h"
//#include "estimate_from_dots_types.h"

/* Function Declarations */
void estimate_from_dots(float p1x0, float p2x0, float p3x0, float
  p4x0, float p1y0, float p2y0, float p3y0, float p4y0, int iterations,
  float *Px, float *Py, float *Pz, float *pitch, float *roll, float *yaw,
  float *wy);

void loc_to_points(float Px, float Py, float Pz, float pitch, float
  roll, float yaw, float wy, float *p1x, float *p2x, float *p3x, float
  *p4x, float *p1y, float *p2y, float *p3y, float *p4y);

#endif

/*
 * File trailer for estimate_from_dots.h
 *
 * [EOF]
 */
