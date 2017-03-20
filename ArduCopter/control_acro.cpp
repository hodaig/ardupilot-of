/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"

#define RADIAN_TO_DEG(r) ((r)*90.0/ M_PI_2)

static uint32_t lastUpdate;
static uint32_t lastStats;
static uint32_t lastPrint;
static float sumX[2];
static float sumY[2];

/* controller variables */
static float alt_I;

static float angle_yaw_initial_rad;


//static float get_yaw_error(float yaw_angle, float desier_yaw_rate){
static float get_yaw_error(float yaw_angle, bool update_angle){
    if (update_angle){
        angle_yaw_initial_rad = yaw_angle;
    }

    float yaw_err = angle_yaw_initial_rad - yaw_angle;
    if (yaw_err > M_PI) yaw_err = yaw_err - M_2PI;
    if (yaw_err < -1.0*M_PI) yaw_err = M_2PI + yaw_err;
    return yaw_err;
}


/*
 * Init and run calls for acro flight mode
 */

// acro_init - initialise acro controller
bool Copter::acro_init(bool ignore_checks)
{
   // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
   if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
       return false;
   }
   // set target altitude to zero for reporting
   pos_control.set_alt_target(0);

   lastStats=0;
   lastPrint=0;
   lastUpdate=0;
   sumX[0] = 0;
   sumX[1] = 0;
   sumY[0] = 0;
   sumY[1] = 0;

   alt_I = 0.0;
   angle_yaw_initial_rad = 0.0;

   return true;
}

// acro_run - runs the acro controller
// should be called at 100hz or more
void Copter::acro_run()
{
    float pilot_roll, pilot_pitch, pilot_yaw;
    float pilot_throttle_scaled, pilot_throttle;
    uint32_t optFlow_interval_ms = 100; // apply opticflow at 10Hz
    uint32_t now = AP_HAL::millis();

    bool need_print = false;
    if (lastPrint + 300 < now){
        lastPrint = now;
        need_print = true;
    }

#if 0
    if (opticalFlowPi.last_update() + optFlow_interval_ms < now){
        opticalFlowPi.update();
    }

    if (lastStats + 5000 < now){
        lastStats = now;
        opticalFlowPi.showStatistics();
    }

    /* export optFlow data */
    if ((opticalFlowPi.last_update(0) > lastUpdate ||
             opticalFlowPi.last_update(1) > lastUpdate)){
        lastUpdate = MAX(opticalFlowPi.last_update(0), opticalFlowPi.last_update(1));

        sumX[0] += opticalFlowPi.getVel(0).x;
        sumY[0] += opticalFlowPi.getVel(0).y;
        sumX[1] += opticalFlowPi.getVel(1).x;
        sumY[1] += opticalFlowPi.getVel(1).y;

        //printf("%10d ;", lastUpdate); // now
        printf("%4d ; %6.2f ; %6.2f;", opticalFlowPi.quality(0), opticalFlowPi.getVel(0).x, opticalFlowPi.getVel(0).y);
        printf("%8.2f ; %8.2f;",sumX[0], sumY[0]);

        printf("%4d ; %6.2f ; %6.2f ;", opticalFlowPi.quality(1), opticalFlowPi.getVel(1).x, opticalFlowPi.getVel(1).y);
        printf("%8.2f ; %8.2f\n",sumX[1], sumY[1]);
    }
#endif

    static float cur_alt = 0.0f;

#define NATNET 1
#if NATNET

    natNet.update();
    if (natNet.healthy()){
        cur_alt = natNet.getLocation().y;
        //roll_angle =  natNet.get_euler_roll();
        //pitch_angle = natNet.get_euler_pitch();
        //yaw_angle =   natNet.get_euler_yaw();
    }
#else
    //cur_alt = (rangefinder_state.alt_cm_filt.get()/100.0) - 0.3;
    cur_alt = (rangefinder.distance_cm()/100.0f) - 0.3f;
    //cur_alt = cur_alt*0.5f + (((float)rangefinder_state.alt_cm/100.0f) - 0.3f)*0.5f;
#endif

    // x->roll axis (acc:back gyro:rigth)
    // y->pich axis (acc:left gyro:back)
    // z->yaw axis (acc:up gyro:rigth)
    const Vector3f insAccel = ins.get_accel(); // vector of current accelerations in m/s/s
    const Vector3f insGyro = ins.get_gyro();   // vector of rotational rates in radians/sec

    const Vector3f compass_field = compass.get_field(); // current field as a Vector3f in milligauss
    const float angle_yaw_rad = atan2f(compass_field.y, compass_field.x);

    /* from atitude_control code */
    /*****************************/
    Quaternion attitude_vehicle_quat;

    attitude_vehicle_quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned());
    float roll_angle =  attitude_vehicle_quat.get_euler_roll();
    float pitch_angle = attitude_vehicle_quat.get_euler_pitch();
    float yaw_angle =   attitude_vehicle_quat.get_euler_yaw();

#if 1
    if (need_print){
        printf("cur_alt %6.3f\n", cur_alt);
        //printf("axis_angles_norm = [%6.3f , %6.3f, %6.3f]\n", roll_angle, pitch_angle, yaw_angle);
    }
    // just for ignoring compilation errors
    if (need_print & 0){
        printf("insAccel = [%6.3f , %6.3f , %6.3f]  insGyro = [%6.3f , %6.3f , %6.3f]\n", insAccel.x, insAccel.y, insAccel.z, insGyro.x, insGyro.y, insGyro.z);
        printf("compass_field = [%6.3f , %6.3f , %6.3f] angle_to_north = %6.3f\n", compass_field.x, compass_field.y, compass_field.z, angle_yaw_rad);
        printf("attitude_vehicle_quat = [%6.3f , %6.3f , %6.3f, %6.3f]\n", attitude_vehicle_quat.q1, attitude_vehicle_quat.q2, attitude_vehicle_quat.q3, attitude_vehicle_quat.q4);
    }
#endif


#define SONAR_ALT 1
#define RC_STILE 0

#if RC_STILE == 0 // direct
    // get pilot's desired throttle
    pilot_throttle = get_pilot_desired_throttle(channel_throttle->get_control_in());  // [0.0 , 1.0]
    //pilot_throttle_scaled = channel_throttle->get_control_in(); // [0 , 2000] (int)

    pilot_roll = channel_roll->get_control_in(); // [-4500 , +4500] (int)
    pilot_pitch = channel_pitch->get_control_in(); // [-4500 , +4500] (int)
    pilot_yaw = channel_yaw->get_control_in(); // [-4500 , +4500] (int)

    pilot_roll = pilot_roll / 5000.0; // [-1 , +1] (float)
    pilot_pitch = pilot_pitch / 5000.0; // [-1 , +1] (float)
    pilot_yaw = pilot_yaw / 5000.0; // [-1 , +1] (float)

    if(need_print & 0){
        printf("throttle = %8.3f, roll = %8.3f, pitch = %8.3f yaw = %8.3f\n", pilot_throttle, pilot_roll, pilot_pitch, pilot_yaw);
    }

#elif RC_STILE == 1 // acro
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());  //TODO [0.0 , 1.0]
    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), pilot_roll, pilot_pitch, pilot_yaw);
#elif RC_STILE == 2 // stabilize
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), pilot_roll, pilot_pitch, aparm.angle_max);
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());  //TODO [0.0 , 1.0]

    // get pilot's desired yaw rate
    pilot_yaw = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
#endif

#if 0
#define OPTFLOW_DIFF_TRASHOLD 0.01f
    float pitch_opt_cmd = 0;

    if (opticalFlowPi.healthy() && opticalFlowPi.quality(0) > 20 && opticalFlowPi.quality(1) > 20){
        float delta_speed_morm = opticalFlowPi.getVel(0).length() - opticalFlowPi.getVel(1).length();

        float delta_speed_morm_scaled = delta_speed_morm/50.0f;
        delta_speed_morm_scaled = constrain_float(delta_speed_morm_scaled, -0.5f, 0.5f); // [-0.5, 0.5]

        // clear little noises
        if (delta_speed_morm_scaled > -OPTFLOW_DIFF_TRASHOLD && delta_speed_morm_scaled < OPTFLOW_DIFF_TRASHOLD){
            delta_speed_morm_scaled = 0;
        } else if (delta_speed_morm_scaled < 0){
            delta_speed_morm_scaled += OPTFLOW_DIFF_TRASHOLD;
        } else if (delta_speed_morm_scaled > 0){
            delta_speed_morm_scaled -= OPTFLOW_DIFF_TRASHOLD;
        }


        pitch_opt_cmd = delta_speed_morm_scaled; // P term

        if(need_print){
            printf("q0= %d q1= %d , delta_speed_morm_scaled = %6.3f delta_speed_morm =  %6.3f\n",
                    opticalFlowPi.quality(0), opticalFlowPi.quality(1),
                    delta_speed_morm_scaled,
                    delta_speed_morm);
        }
    }
#endif

    /************************/
    /***** controller *******/
    /************************/

#define KP_P 0.10f
#define KP_Q 0.12f
#define KP_R 0.5f

    static float last_alt, vertical_vel_filt;
    static uint32_t last_alt_time_ms;

    float pilot_scaled;
    float error;
    float PTerm;
    float error_phi;

    /* roll */
    pilot_scaled = pilot_roll/9.0/KP_P;  // scale RC input
    error_phi = (pilot_scaled - roll_angle)*3.0f;
    error = error_phi - insGyro.x;
    PTerm = error * KP_P;
    float roll_cmd = constrain_float(PTerm, -1.0f, +1.0f);

    /* pitch */
    pilot_scaled = pilot_pitch/9.0/KP_Q;  // scale RC input
    error_phi = (pilot_scaled - pitch_angle)*3.0f;
    error = error_phi - insGyro.y;
    PTerm = error * KP_Q;
    float pitch_cmd = constrain_float(PTerm, -1.0f, +1.0f);

    /* yaw */
    pilot_scaled = pilot_yaw/3/KP_R;  // scale RC input
    error = pilot_scaled - insGyro.z;
    PTerm = error * KP_R;
    float yaw_cmd = constrain_float(PTerm, -1.0f, +1.0f);

    pilot_scaled = pilot_yaw/1.0;  // scale RC input
    //error = pilot_scaled - insGyro.z;
    error = get_yaw_error(yaw_angle, (pilot_scaled != 0)) + pilot_scaled;
    PTerm = error * KP_R;
    yaw_cmd = constrain_float(yaw_cmd + PTerm, -1.0f, +1.0f);

    /* throttle */
    float delta_time = (float)(now - last_alt_time_ms) / 1000.0f;
    float delta_alt = cur_alt - last_alt;
    float vertical_vel = delta_alt / delta_time;
    last_alt_time_ms = now;
    last_alt = cur_alt;

    vertical_vel_filt = vertical_vel_filt*0.9f + vertical_vel*0.1f;

    pilot_scaled = pilot_throttle*1.5;
    error = (pilot_scaled - cur_alt)*0.2f;
    float throtle_cmd = constrain_float( (0.3 + error - vertical_vel_filt*0.1f) , -1.0f, +1.0f);

    if(need_print){
        printf("vertical_vel_filt = %8.3ff\n", vertical_vel_filt);
    }


    /* failsafe */
#define MAX_ANGLE_ALLOW 0.8
    if ((roll_angle < -MAX_ANGLE_ALLOW || roll_angle > MAX_ANGLE_ALLOW) ||
            (pitch_angle < -MAX_ANGLE_ALLOW || pitch_angle > MAX_ANGLE_ALLOW)){
        motors.armed(false);
        motors.output();
    }

    /* arm check */
    /*************/
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        //motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        //attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        motors.set_roll(0.0);
        motors.set_pitch(0.0);
        motors.set_yaw(0.0);
        motors.set_throttle(0.0);

        alt_I = 0;
        angle_yaw_initial_rad = yaw_angle;

        return;
    }

    // clear landing flag
    set_land_complete(false);


    /*************************/
    /***** motors output *****/
    /*************************/
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    motors.set_throttle(throtle_cmd);
    //motors.set_throttle(pilot_throttle);
    motors.set_roll(roll_cmd);
    motors.set_pitch(pitch_cmd);
    motors.set_yaw(yaw_cmd);

    if (need_print){
        printf("pilot_throttle = %8.3f, alt_filt=%8.3f, alt_healthy=%d\n",
                pilot_throttle,
                cur_alt,
                rangefinder_state.alt_healthy);
    }
}

#if 0
#define KP_P 0.12f
#define KP_Q 0.14f
#define KP_R 0.33f

    /* PID controller */
    float pilot_scaled;
    float error;
    float PTerm;

    // roll
    pilot_scaled = pilot_roll/3.0/KP_P;  // scale RC input
    error = pilot_scaled - insGyro.x;
    PTerm = error * KP_P;
    float roll_cmd = constrain_float(PTerm, -1.0f, +1.0f);

    // pitch
    pilot_scaled = pilot_pitch/3.0/KP_Q;  // scale RC input
    error = pilot_scaled - insGyro.y;
    PTerm = error * KP_Q;
    float pitch_cmd = constrain_float(PTerm, -1.0f, +1.0f);

    //yaw
    pilot_scaled = pilot_yaw/3/KP_R;  // scale RC input
    error = pilot_scaled - insGyro.z;
    PTerm = error * KP_R;
    float yaw_cmd = constrain_float(PTerm, -1.0f, +1.0f);
#endif
