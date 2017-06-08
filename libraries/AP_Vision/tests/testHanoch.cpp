/*
 * test.cpp
 *
 *  Created on: May 16, 2017
 *      Author: hodai
 */

#include <stdio.h> // printf
#include <stdlib.h>     /* srand, rand */
#include "../estimate_from_dots.h"
#include "TimeStatistics.h"

struct time_stat_t estimation;

void printDots(float p1x, float p2x, float p3x, float
        p4x, float p1y, float p2y, float p3y, float p4y){

    printf("x: %5.3f , %5.3f , %5.3f , %5.3f \n", p1x, p2x, p3x, p4x);
    printf("y: %5.3f , %5.3f , %5.3f , %5.3f \n\n", p1y, p2y, p3y, p4y);

}

float deg_to_rad(float deg){
    return deg*0.0174532925f;
}

#define RANDOM_DETWEEN_F(min, max)\
    ((min) + ((float)rand()) * (((max)-(min))/RAND_MAX) )

#define ABS(v) (((v) < 0) ? -(v) : (v) )

void randomTest(int iters){
    STAT_INIT(estimation, "estimation");
    int numberOfTests = 100;

    float Px = -4;
    float Py = -1;
    float Pz = 1;
    float pitch = deg_to_rad(20);
    float roll = deg_to_rad(20);
    float yaw = deg_to_rad(20);
    float wy = 1.2;


    float p1x;
    float p2x;
    float p3x;
    float p4x;
    float p1y;
    float p2y;
    float p3y;
    float p4y;

    /* outputs */
    float out_Px,  out_Py,  out_Pz,  out_pitch,  out_roll,  out_yaw, out_wy;

    /* errors */
    float err_Px=0,  err_Py=0,  err_Pz=0,  err_pitch=0,  err_roll=0,  err_yaw=0, err_wy=0;

    /* initialize random seed: */
    srand (time(NULL));

    for (int i=0 ; i<100 ; i++){

        /* generate values */
        Px = RANDOM_DETWEEN_F(-10, -2);
        Py = RANDOM_DETWEEN_F(-1.5, 1.5);
        Pz = RANDOM_DETWEEN_F(-1.5, 1.5);
        pitch = RANDOM_DETWEEN_F(deg_to_rad(-45), deg_to_rad(45));
        roll = RANDOM_DETWEEN_F(deg_to_rad(-45), deg_to_rad(45));
        yaw = RANDOM_DETWEEN_F(deg_to_rad(-45), deg_to_rad(45));
        wy = RANDOM_DETWEEN_F(0.8, 2.0);


        // simulate the dots
        loc_to_points( Px,  Py,  Pz,  pitch,
                roll,  yaw,  wy, &p1x, &p2x, &p3x,
                &p4x, &p1y, &p2y, &p3y, &p4y);
/*
        printDots(p1x, p2x, p3x,
                p4x, p1y, p2y, p3y, p4y);
*/
        // estimate the loc & ori

        STATS_START(estimation);
        estimate_from_dots( p1x,  p2x,  p3x,
                p4x,  p1y,  p2y,  p3y,  p4y,  iters,
                &out_Px,  &out_Py,  &out_Pz,  &out_pitch,  &out_roll,  &out_yaw,
                &out_wy);
        STATS_END(estimation);

        loc_to_points( Px,  Py,  Pz,  pitch,
                roll,  yaw,  wy, &p1x, &p2x, &p3x,
                &p4x, &p1y, &p2y, &p3y, &p4y);

        printf("outputs:\n");
        printDots(p1x, p2x, p3x,
                p4x, p1y, p2y, p3y, p4y);

/*
        printf("real: Px=%5.3f ,  Py=%5.3f ,  Pz=%5.3f ,  pitch=%5.3f ,  roll=%5.3f ,  yaw=%5.3f , wy=%5.3f\n",
                Px,  Py,  Pz,  pitch,  roll,  yaw, wy);
        printf("out:  Px=%5.3f ,  Py=%5.3f ,  Pz=%5.3f ,  pitch=%5.3f ,  roll=%5.3f ,  yaw=%5.3f , wy=%5.3f\n",
                out_Px,  out_Py,  out_Pz,  out_pitch,  out_roll,  out_yaw, out_wy);
  */
        err_Px += ABS(out_Px - Px);
        err_Py += ABS(out_Py - Py);
        err_Pz += ABS(out_Pz - Pz);
        err_pitch += ABS(out_pitch - pitch);
        err_roll += ABS(out_roll - roll);
        err_yaw += ABS(out_yaw - yaw);
        err_wy += ABS(out_wy - wy);

    }

    err_Px /= (float)numberOfTests;
    err_Py /= (float)numberOfTests;
    err_Pz /= (float)numberOfTests;
    err_pitch /= (float)numberOfTests;
    err_roll /= (float)numberOfTests;
    err_yaw /= (float)numberOfTests;
    err_wy /= (float)numberOfTests;

    printf("errors (%d itr): Px=%5.3f ,  Py=%5.3f ,  Pz=%5.3f ,  pitch=%5.3f ,  roll=%5.3f ,  yaw=%5.3f , wy=%5.3f\n",
            iters, err_Px,  err_Py,  err_Pz,  err_pitch,  err_roll,  err_yaw, err_wy);
    STATS_SHOW(estimation);
}

int main(int argc, char **argv) {
    printf("hanoch algo test\n");

    randomTest(40);
    randomTest(60);
    randomTest(80);
    randomTest(100);

}
