/*
 * File: estimate_from_dots.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 15-May-2017 17:21:22
 */

/* Include Files */
//#include "rt_nonfinite.h"
#include "estimate_from_dots.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/* Function Declarations */
static float rt_atan2d_snf(float u0, float u1);

#if 0
/* Function Definitions */

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2d_snf(float u0, float u1)
{
  float y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

#else
static float rt_atan2d_snf(float u0, float u1)
{
    return atan2(u0, u1);
}
#endif

/*
 * initialization
 * Arguments    : float p1x0
 *                float p2x0
 *                float p3x0
 *                float p4x0
 *                float p1y0
 *                float p2y0
 *                float p3y0
 *                float p4y0
 *                float iterations
 *                float *Px
 *                float *Py
 *                float *Pz
 *                float *pitch
 *                float *roll
 *                float *yaw
 *                float *wy
 * Return Type  : void
 */
void estimate_from_dots(float p1x0, float p2x0, float p3x0, float p4x0,
  float p1y0, float p2y0, float p3y0, float p4y0, int iterations, float *
  Px, float *Py, float *Pz, float *pitch, float *roll, float *yaw, float
  *wy)
{
  float mtop0;
  float mbot0;
  float mright0;
  float mleft0;
  float sumx0;
  float sumy0;
  float a;
  float b_a;
  float c_a;
  float d_a;
  float Vd0;
  float Hd0;
  float e_a;
  float f_a;
  float g_a;
  float h_a;
  float w2h0;
  int b_index;
  float R[9];
  float dv0[9];
  int i0;
  static const signed char iv0[3] = { 0, 0, 1 };

  static const signed char iv1[3] = { 0, 1, 0 };

  float dv1[9];
  float dv2[9];
  int i1;
  static const signed char iv2[3] = { 1, 0, 0 };

  int i2;
  float b_Px[3];
  float b_R[3];
  float c_R[9];
  float dv3[4];
  float G[12];
  float dv4[4];
  float dv5[4];
  float dv6[4];
  float p1x;
  float p1[3];
  float p2x;
  float p2[3];
  float p3x;
  float p3[3];
  float p4x;
  float p4[3];
  float p1y;
  float p2y;
  float p3y;
  float p4y;
  float i_a;
  float j_a;
  float k_a;
  float l_a;
  float m_a;
  float n_a;
  float o_a;
  float p_a;
  *pitch = 0.0;
  *roll = 0.0;
  *yaw = 0.0;
  *Px = -4.0;
  *Py = 0.0;
  *Pz = 0.0;
  *wy = 1.0;

  /* %%%%%%%%%%%%%%%%% */
  /*  starting point % */
  /* %%%%%%%%%%%%%%%%% */
  mtop0 = rt_atan2d_snf(p2y0 - p1y0, p2x0 - p1x0);
  mbot0 = rt_atan2d_snf(p3y0 - p4y0, p3x0 - p4x0);
  mright0 = rt_atan2d_snf(p3x0 - p2x0, -p3y0 + p2y0);
  mleft0 = rt_atan2d_snf(-p1x0 + p4x0, p1y0 - p4y0);
  sumx0 = ((p1x0 + p2x0) + p3x0) + p4x0;
  sumy0 = ((p1y0 + p2y0) + p3y0) + p4y0;
  a = p2x0 - p3x0;
  b_a = p2y0 - p3y0;
  c_a = p1x0 - p4x0;
  d_a = p1y0 - p4y0;
  Vd0 = sqrt(a * a + b_a * b_a) / sqrt(c_a * c_a + d_a * d_a);
  a = p1x0 - p2x0;
  b_a = p1y0 - p2y0;
  c_a = p3x0 - p4x0;
  d_a = p3y0 - p4y0;
  Hd0 = sqrt(a * a + b_a * b_a) / sqrt(c_a * c_a + d_a * d_a);
  a = p2x0 - p3x0;
  b_a = p2y0 - p3y0;
  c_a = p1x0 - p4x0;
  d_a = p1y0 - p4y0;
  e_a = p1x0 - p2x0;
  f_a = p1y0 - p2y0;
  g_a = p3x0 - p4x0;
  h_a = p3y0 - p4y0;
  w2h0 = (sqrt(a * a + b_a * b_a) + sqrt(c_a * c_a + d_a * d_a)) / (sqrt(e_a *
    e_a + f_a * f_a) + sqrt(g_a * g_a + h_a * h_a));

  /* %%%%%%%%%%%%%%%% */
  /*  start iterate % */
  /* %%%%%%%%%%%%%%%% */
  for (b_index = 0; b_index < iterations; b_index++) {
    R[0] = cosf(*yaw);
    R[3] = -sinf(*yaw);
    R[6] = 0.0;
    R[1] = sinf(*yaw);
    R[4] = cosf(*yaw);
    R[7] = 0.0;
    dv0[0] = cosf(*pitch);
    dv0[3] = 0.0;
    dv0[6] = sinf(*pitch);
    for (i0 = 0; i0 < 3; i0++) {
      R[2 + 3 * i0] = iv0[i0];
      dv0[1 + 3 * i0] = iv1[i0];
    }

    dv0[2] = -sinf(*pitch);
    dv0[5] = 0.0;
    dv0[8] = cosf(*pitch);
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        dv1[i0 + 3 * i1] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          dv1[i0 + 3 * i1] += R[i0 + 3 * i2] * dv0[i2 + 3 * i1];
        }
      }

      dv2[3 * i0] = iv2[i0];
    }

    dv2[1] = 0.0;
    dv2[4] = cosf(*roll);
    dv2[7] = -sinf(*roll);
    dv2[2] = 0.0;
    dv2[5] = sinf(*roll);
    dv2[8] = cosf(*roll);
    b_Px[0] = *Px;
    b_Px[1] = *Py;
    b_Px[2] = *Pz;
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        c_R[i0 + 3 * i1] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          c_R[i0 + 3 * i1] += dv1[i0 + 3 * i2] * dv2[i2 + 3 * i1];
        }

        R[i1 + 3 * i0] = -c_R[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      b_R[i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        G[i1 + 3 * i0] = c_R[i0 + 3 * i1];
        b_R[i0] += R[i0 + 3 * i1] * b_Px[i1];
      }

      G[9 + i0] = b_R[i0];
    }

    /* wy=1.5; */
    dv3[0] = 0.0;
    dv3[1] = -*wy;
    dv3[2] = -1.0;
    dv3[3] = 1.0;
    dv4[0] = 0.0;
    dv4[1] = *wy;
    dv4[2] = -1.0;
    dv4[3] = 1.0;
    dv5[0] = 0.0;
    dv5[1] = *wy;
    dv5[2] = 1.0;
    dv5[3] = 1.0;
    dv6[0] = 0.0;
    dv6[1] = -*wy;
    dv6[2] = 1.0;
    dv6[3] = 1.0;
    for (i0 = 0; i0 < 3; i0++) {
      p1[i0] = 0.0;
      p2[i0] = 0.0;
      p3[i0] = 0.0;
      p4[i0] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        p1[i0] += G[i0 + 3 * i1] * dv3[i1];
        p2[i0] += G[i0 + 3 * i1] * dv4[i1];
        p3[i0] += G[i0 + 3 * i1] * dv5[i1];
        p4[i0] += G[i0 + 3 * i1] * dv6[i1];
      }
    }

    p1x = p1[1] / p1[0];
    p2x = p2[1] / p2[0];
    p3x = p3[1] / p3[0];
    p4x = p4[1] / p4[0];
    p1y = -p1[2] / p1[0];
    p2y = -p2[2] / p2[0];
    p3y = -p3[2] / p3[0];
    p4y = -p4[2] / p4[0];

    /*  calc the errors */
    a = p2x - p3x;
    b_a = p2y - p3y;
    c_a = p1x - p4x;
    d_a = p1y - p4y;
    e_a = p1x - p2x;
    f_a = p1y - p2y;
    g_a = p3x - p4x;
    h_a = p3y - p4y;
    i_a = p2x - p3x;
    j_a = p2y - p3y;
    k_a = p1x - p4x;
    l_a = p1y - p4y;
    m_a = p1x - p2x;
    n_a = p1y - p2y;
    o_a = p3x - p4x;
    p_a = p3y - p4y;

    /*  regulate */
    float generalGain = 1.0f;
    /*  roll */
    *roll -= generalGain * 0.2 * ((((rt_atan2d_snf(p2y - p1y, p2x - p1x) + rt_atan2d_snf(p3y -
      p4y, p3x - p4x)) + rt_atan2d_snf(-p1x + p4x, p1y - p4y)) + rt_atan2d_snf
                     (p3x - p2x, -p3y + p2y)) - (((mtop0 + mbot0) + mleft0) +
      mright0));

    /*  pich */
    *pitch += generalGain * 0.4 * (sqrt(e_a * e_a + f_a * f_a) / sqrt(g_a * g_a + h_a * h_a) -
                     Hd0);

    /*  yaw */
    *yaw += generalGain * 0.2 * (sqrt(a * a + b_a * b_a) / sqrt(c_a * c_a + d_a * d_a) - Vd0);

    /*  y */
    *Py += generalGain * 0.1 * ((((p1x + p2x) + p3x) + p4x) - sumx0);

    /*  z */
    *Pz -= generalGain * 0.2 * ((((p1y + p2y) + p3y) + p4y) - sumy0);

    /*  wy */
    *wy += generalGain * 0.2 * ((sqrt(i_a * i_a + j_a * j_a) + sqrt(k_a * k_a + l_a * l_a)) /
                  (sqrt(m_a * m_a + n_a * n_a) + sqrt(o_a * o_a + p_a * p_a)) -
                  w2h0);

    /*  x - Hodai addition */
    *Px -= generalGain * 0.2 * ((((((((p1x0 - p2x0) - (p1x - p2x)) + (p4x0 - p3x0)) - (p4x -
      p3x)) + (p4y0 - p1y0)) - (p4y - p1y)) + (p3y0 - p2y0)) - (p3y - p2y));
  }
}

/*
 * Arguments    : float Px
 *                float Py
 *                float Pz
 *                float pitch
 *                float roll
 *                float yaw
 *                float wy
 *                float *p1x
 *                float *p2x
 *                float *p3x
 *                float *p4x
 *                float *p1y
 *                float *p2y
 *                float *p3y
 *                float *p4y
 * Return Type  : void
 */
void loc_to_points(float Px, float Py, float Pz, float pitch, float roll,
                   float yaw, float wy, float *p1x, float *p2x, float *p3x,
                   float *p4x, float *p1y, float *p2y, float *p3y, float *p4y)
{
  float R[9];
  float dv0[9];
  int i0;
  static const signed char iv0[3] = { 0, 0, 1 };

  static const signed char iv1[3] = { 0, 1, 0 };

  float dv1[9];
  float dv2[9];
  int i1;
  static const signed char iv2[3] = { 1, 0, 0 };

  int i2;
  float b_Px[3];
  float b_R[3];
  float c_R[9];
  float dv3[4];
  float G[12];
  float dv4[4];
  float dv5[4];
  float dv6[4];
  float p1[3];
  float p2[3];
  float p3[3];
  float p4[3];
  R[0] = cosf(yaw);
  R[3] = -sinf(yaw);
  R[6] = 0.0;
  R[1] = sinf(yaw);
  R[4] = cosf(yaw);
  R[7] = 0.0;
  dv0[0] = cosf(pitch);
  dv0[3] = 0.0;
  dv0[6] = sinf(pitch);
  for (i0 = 0; i0 < 3; i0++) {
    R[2 + 3 * i0] = iv0[i0];
    dv0[1 + 3 * i0] = iv1[i0];
  }

  dv0[2] = -sinf(pitch);
  dv0[5] = 0.0;
  dv0[8] = cosf(pitch);
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      dv1[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i0 + 3 * i1] += R[i0 + 3 * i2] * dv0[i2 + 3 * i1];
      }
    }

    dv2[3 * i0] = iv2[i0];
  }

  dv2[1] = 0.0;
  dv2[4] = cosf(roll);
  dv2[7] = -sinf(roll);
  dv2[2] = 0.0;
  dv2[5] = sinf(roll);
  dv2[8] = cosf(roll);
  b_Px[0] = Px;
  b_Px[1] = Py;
  b_Px[2] = Pz;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      c_R[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        c_R[i0 + 3 * i1] += dv1[i0 + 3 * i2] * dv2[i2 + 3 * i1];
      }

      R[i1 + 3 * i0] = -c_R[i0 + 3 * i1];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    b_R[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      G[i1 + 3 * i0] = c_R[i0 + 3 * i1];
      b_R[i0] += R[i0 + 3 * i1] * b_Px[i1];
    }

    G[9 + i0] = b_R[i0];
  }

  /* wy=1.5; */
  dv3[0] = 0.0;
  dv3[1] = -wy;
  dv3[2] = -1.0;
  dv3[3] = 1.0;
  dv4[0] = 0.0;
  dv4[1] = wy;
  dv4[2] = -1.0;
  dv4[3] = 1.0;
  dv5[0] = 0.0;
  dv5[1] = wy;
  dv5[2] = 1.0;
  dv5[3] = 1.0;
  dv6[0] = 0.0;
  dv6[1] = -wy;
  dv6[2] = 1.0;
  dv6[3] = 1.0;
  for (i0 = 0; i0 < 3; i0++) {
    p1[i0] = 0.0;
    p2[i0] = 0.0;
    p3[i0] = 0.0;
    p4[i0] = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      p1[i0] += G[i0 + 3 * i1] * dv3[i1];
      p2[i0] += G[i0 + 3 * i1] * dv4[i1];
      p3[i0] += G[i0 + 3 * i1] * dv5[i1];
      p4[i0] += G[i0 + 3 * i1] * dv6[i1];
    }
  }

  *p1x = p1[1] / p1[0];
  *p2x = p2[1] / p2[0];
  *p3x = p3[1] / p3[0];
  *p4x = p4[1] / p4[0];
  *p1y = -p1[2] / p1[0];
  *p2y = -p2[2] / p2[0];
  *p3y = -p3[2] / p3[0];
  *p4y = -p4[2] / p4[0];
}


/*
 * File trailer for estimate_from_dots.c
 *
 * [EOF]
 */
