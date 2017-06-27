/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"
#include "MyLogger.h"

#define RADIAN_TO_DEG(r) ((r)*(90.0/ M_PI_2))
#define RCIN(n) (g.rc_##n.get_radio_in())

static uint32_t lastUpdate;
static uint32_t lastStats;
static uint32_t lastPrint;
static float sumX[2];
static float sumY[2];

/* controller variables */

static float angle_yaw_initial_rad;

static MyLogger myLogger("ArduCopter/logOut.csv");


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

    angle_yaw_initial_rad = 0.0;

#define log_NN_pich 0
    myLogger.printVal(log_NN_pich, "natnet pitch");
#define log_NN_roll 1
    myLogger.printVal(log_NN_roll, "natnet roll");
#define log_NN_yaw 2
    myLogger.printVal(log_NN_yaw, "natnet yaw");

#define log_NN_X 3
    myLogger.printVal(log_NN_X, "natnet x");
#define log_NN_Z 4
    myLogger.printVal(log_NN_Z, "natnet z");
#define log_NN_alt 5
    myLogger.printVal(log_NN_alt, "natnet alt");

#define log_cmd_pitch 6
    myLogger.printVal(log_cmd_pitch, "cmd pitch");
#define log_cmd_roll 7
    myLogger.printVal(log_cmd_roll, "cmd roll");
#define log_cmd_yaw 8
    myLogger.printVal(log_cmd_yaw, "cmd yaw");
#define log_cmd_throtle 9
    myLogger.printVal(log_cmd_throtle, "cmd throtle");



#define log_VISION_delay 12
    myLogger.printVal(log_VISION_delay, "vision delay");
#define log_VISION_res 13
    myLogger.printVal(log_VISION_res, "vision res (*240)");

#define log_VISION_status 14
    myLogger.printVal(log_VISION_status, "vision status");
#define log_VISION_dots 15
    myLogger.printVal(log_VISION_dots, "num of dots found");
#define log_VISION_mass 16
    myLogger.printVal(log_VISION_mass, "mass");


#define log_VISION_p1x_cam 17
    myLogger.printVal(log_VISION_p1x_cam, "p1x vid");
#define log_VISION_p1y_cam 18
    myLogger.printVal(log_VISION_p1y_cam, "p1y vid");
#define log_VISION_p1x_est 19
   // myLogger.printVal(log_VISION_p1x_est, "p1x est");
#define log_VISION_p1y_est 20
   // myLogger.printVal(log_VISION_p1y_est, "p1y est");

#define log_VISION_p2x_cam 21
    myLogger.printVal(log_VISION_p2x_cam, "p2x vid");
#define log_VISION_p2y_cam 22
    myLogger.printVal(log_VISION_p2y_cam, "p2y vid");
#define log_VISION_p2x_est 23
  //  myLogger.printVal(log_VISION_p2x_est, "p2x est");
#define log_VISION_p2y_est 24
   // myLogger.printVal(log_VISION_p2y_est, "p2y est");

#define log_VISION_p3x_cam 25
    myLogger.printVal(log_VISION_p3x_cam, "p3x vid");
#define log_VISION_p3y_cam 26
    myLogger.printVal(log_VISION_p3y_cam, "p3y vid");
#define log_VISION_p3x_est 27
   // myLogger.printVal(log_VISION_p3x_est, "p3x est");
#define log_VISION_p3y_est 28
   // myLogger.printVal(log_VISION_p3y_est, "p3y est");

#define log_VISION_p4x_cam 29
    myLogger.printVal(log_VISION_p4x_cam, "p4x vid");
#define log_VISION_p4y_cam 30
    myLogger.printVal(log_VISION_p4y_cam, "p4y vid");
#define log_VISION_p4x_est 31
    //myLogger.printVal(log_VISION_p4x_est, "p4x est");
#define log_VISION_p4y_est 32
    //myLogger.printVal(log_VISION_p4y_est, "p4y est");

#define log_VISION_Sx 33
    myLogger.printVal(log_VISION_Sx, "vision Sx");
#define log_VISION_Sy 34
    myLogger.printVal(log_VISION_Sy, "vision Sy");
#define log_VISION_Vd 35
    myLogger.printVal(log_VISION_Vd, "vision Vd");
#define log_VISION_Vd_filt 36
    myLogger.printVal(log_VISION_Vd_filt, "vision Vd-filt");

#define log_vel_x 37
    myLogger.printVal(log_vel_x, "vel x");
#define log_vel_x_filt 38
    myLogger.printVal(log_vel_x_filt, "vel x-filt");
#define log_vel_y 39
    myLogger.printVal(log_vel_y, "vel y");
#define log_vel_y_filt 40
    myLogger.printVal(log_vel_y_filt, "vel y-filt");

#define log_window_size 41
    myLogger.printVal(log_window_size, "window_size");


#define log_rangefinder 42
    //myLogger.printVal(log_rangefinder, "rangefinder");

#define log_VISION_X 43
    //myLogger.printVal(log_VISION_X, "vision x");

#define log_VISION_pich 44
    //myLogger.printVal(log_VISION_pich, "vision pitch");
#define log_VISION_roll 45
    //myLogger.printVal(log_VISION_roll, "vision roll");
#define log_VISION_yaw 46
    //myLogger.printVal(log_VISION_yaw, "vision yaw");

#define log_VISION_angle_0 47
    myLogger.printVal(log_VISION_angle_0, "VISION_angle_0");
#define log_VISION_angle_1 48
    myLogger.printVal(log_VISION_angle_1, "VISION_angle_1");
#define log_VISION_angle_2 49
    myLogger.printVal(log_VISION_angle_2, "VISION_angle_2");
#define log_VISION_angle_3 50
    myLogger.printVal(log_VISION_angle_3, "VISION_angle_3");


    return true;
}

#define CONVERGENCE_TEST 1

// acro_run - runs the acro controller
// should be called at 100hz or more
void Copter::acro_run()
{
    float pilot_roll, pilot_pitch, pilot_yaw;
    float pilot_throttle;
    //uint32_t now = AP_HAL::millis();
    uint32_t now_us = AP_HAL::micros();

    static bool loging;
    if (RCIN(6) > 1400){
        if (!loging){
            loging = true;
            printf("start logging sw: %d\n", RCIN(6));
        }
        myLogger.flashLine();
    } else {
        if (loging){
            loging = false;
            printf("stop logging sw: %d\n", RCIN(6));
            printf("resolution = %d X %d\n", vision.getHeight(), vision.getWidth());
            vision.showStatistics();
        }
    }

    bool need_print = false;
    if (300*1000 < (now_us - lastPrint) ||
            (now_us < lastPrint) ){ // now_us 32bit ovweflow
        lastPrint = now_us;
        need_print = true;
    }


    // Dynamic resolution
    static int resMode;
#if CONVERGENCE_TEST
    int resModeNew =  (RCIN(8) > 1500 ? 3 : 0); // AUX2  - only 240P or 960P
#else
    int resModeNew = (RCIN(7) > 1500 ? 1 : 0) // AUX1
                   + (RCIN(8) > 1500 ? 1<<1 : 0); // AUX2
#endif

    myLogger.putVal(log_VISION_res, resMode);

    if (resMode != resModeNew){
        resMode = resModeNew;
        switch (resMode) {
        case 0: // DD
            if (!vision.setResolution(240, 320)){ // factor of 240X320
                printf("vision.setResolution(240, 320) fail!\n");
            }
            break;
        case 1: // UD
            if (!vision.setResolution(480, 640)){ // factor of 240X320
                printf("vision.setResolution(480, 640) fail!\n");
            }
            break;
        case 2: // DU
            if (!vision.setResolution(720, 960)){ // factor of 240X320
                printf("vision.setResolution(720, 960) fail!\n");
            }
            break;
        case 3: // UU
            if (!vision.setResolution(960, 1280)){ // factor of 240X320
                printf("vision.setResolution(960, 1280) fail!\n");
            }
            break;

        default:
            break;
        }
        printf("res= %d X %d\n", vision.getHeight(), vision.getWidth());
        vision.initStatistics();
    }




    /*******************************/
    /******* sensors update ********/
    /*******************************/

    static float cur_alt = 0.0f; // y axis
    static float cur_x = 0.0f;
    static float cur_z = 0.0f;
    static float yaw_angle = 0.0f;

#define NATNET 1
#define NATNET_POS_HOLD 1
#if NATNET
    natNet.update();
    if (natNet.healthy()){
        cur_alt = natNet.getLocation().y;
        cur_x = natNet.getLocation().x;
        cur_z = natNet.getLocation().z;
        //roll_angle =  natNet.get_euler_roll();
        //pitch_angle = natNet.get_euler_pitch();
        yaw_angle =   natNet.get_euler_yaw();
        myLogger.putVal(log_NN_pich, natNet.get_euler_pitch());
        myLogger.putVal(log_NN_roll, natNet.get_euler_roll());
        myLogger.putVal(log_NN_yaw, natNet.get_euler_yaw());
        myLogger.putVal(log_NN_X, cur_x);
        myLogger.putVal(log_NN_Z, cur_z);
        myLogger.putVal(log_NN_alt, cur_alt);
    }
#else
    //cur_alt = (rangefinder_state.alt_cm_filt.get()/100.0) - 0.3;
    cur_alt = (rangefinder.distance_cm()/100.0f) - 0.3f;
    //cur_alt = cur_alt*0.5f + (((float)rangefinder_state.alt_cm/100.0f) - 0.3f)*0.5f;
#endif
    static float rangefinder_filt = 0;
    rangefinder_filt = rangefinder_filt*0.99 + (rangefinder.distance_cm()/100.0f)*0.01;
    myLogger.putVal(log_rangefinder, (rangefinder.distance_cm()/100.0f));


    // x->roll axis (acc:back gyro:rigth)
    // y->pich axis (acc:left gyro:back)
    // z->yaw axis (acc:up gyro:rigth)
    //const Vector3f insAccel = ins.get_accel(); // vector of current accelerations in m/s/s
    const Vector3f insGyro = ins.get_gyro();   // vector of rotational rates in radians/sec

    Quaternion attitude_vehicle_quat;
    attitude_vehicle_quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned());
    float roll_angle =  attitude_vehicle_quat.get_euler_roll();
    float pitch_angle = attitude_vehicle_quat.get_euler_pitch();
    //float yaw_angle =   attitude_vehicle_quat.get_euler_yaw();

#if 0
    if (need_print){
        printf("switches: 5:%d, 6:%d, 7:%d, 8:%d, 9:%d\n", RCIN(5), RCIN(6), RCIN(7), RCIN(8), RCIN(9));
        //printf("cur_alt %6.3f\n", cur_alt);
        printf("natnet       x:%6.3f y:%6.3f , z:%6.3f HEL:%d\n", natNet.getLocation().x, natNet.getLocation().y, natNet.getLocation().z, natNet.healthy());
        //printf("natnet       roll:%6.3f pitch:%6.3f , yaw:%6.3f HEL:%d\n", natNet.get_euler_roll(), natNet.get_euler_pitch(), natNet.get_euler_yaw(), natNet.healthy());
        //printf("att          roll:%6.3f pitch:%6.3f , yaw:%6.3f\n", roll_angle, pitch_angle, yaw_angle);

    }
    // just for ignoring compilation errors
    if (need_print & 0){
        printf("insGyro = [%6.3f , %6.3f , %6.3f]\n", insGyro.x, insGyro.y, insGyro.z);
        //printf("insAccel = [%6.3f , %6.3f , %6.3f]  insGyro = [%6.3f , %6.3f , %6.3f]\n", insAccel.x, insAccel.y, insAccel.z, insGyro.x, insGyro.y, insGyro.z);
    }

#endif

    /*******************************/
    /******* vision update *********/
    /*******************************/
    bool newVisionData = false;

#define VISION_CONTROL 1
#define VISION 1
#if VISION

    static int pointCount = 0;
    static ip_point p[6];

    if (vision.update()){
        if (vision.healthy()){
            // new data!
            newVisionData = true;
            pointCount = vision.getBestPoints(p, 6); // TODO - for testing
            vision.reorderPoints(p, pointCount);
            if (0 < pointCount){
                myLogger.putVal(log_VISION_mass, p[0].mass);
            }
            myLogger.putVal(log_VISION_status, 1);
#  if 0
            if (vision.estimatePosition()){
                // Success
                myLogger.putVal(log_VISION_pich, vision.getPitch());
                myLogger.putVal(log_VISION_roll, vision.getRoll());
                myLogger.putVal(log_VISION_yaw, vision.getYaw());
                myLogger.putVal(log_VISION_X, vision.getLocX());

                myLogger.putVal(log_VISION_status, 2);
            } else {
            }
#  endif

        } else {
            // vision not healthy
            myLogger.putVal(log_VISION_status, -1);
        }
        //myLogger.putVal(log_VISION_dots, vision.getPointsCount());
        myLogger.putVal(log_VISION_dots, pointCount);

        myLogger.putVal(log_VISION_p1x_cam, vision._pointsVision[0].x);
        myLogger.putVal(log_VISION_p1y_cam, vision._pointsVision[0].y);
        myLogger.putVal(log_VISION_p1x_est, vision._pointsEstimated[0].x);
        myLogger.putVal(log_VISION_p1y_est, vision._pointsEstimated[0].y);

        myLogger.putVal(log_VISION_p2x_cam, vision._pointsVision[1].x);
        myLogger.putVal(log_VISION_p2y_cam, vision._pointsVision[1].y);
        myLogger.putVal(log_VISION_p2x_est, vision._pointsEstimated[1].x);
        myLogger.putVal(log_VISION_p2y_est, vision._pointsEstimated[1].y);

        myLogger.putVal(log_VISION_p3x_cam, vision._pointsVision[2].x);
        myLogger.putVal(log_VISION_p3y_cam, vision._pointsVision[2].y);
        myLogger.putVal(log_VISION_p3x_est, vision._pointsEstimated[2].x);
        myLogger.putVal(log_VISION_p3y_est, vision._pointsEstimated[2].y);

        myLogger.putVal(log_VISION_p4x_cam, vision._pointsVision[3].x);
        myLogger.putVal(log_VISION_p4y_cam, vision._pointsVision[3].y);
        myLogger.putVal(log_VISION_p4x_est, vision._pointsEstimated[3].x);
        myLogger.putVal(log_VISION_p4y_est, vision._pointsEstimated[3].y);

    }

#  if 1
    if (need_print && vision.healthy()){
        printf("vision: [");
        for (int i = 0 ; i<pointCount ; i++){
            printf(" (%5.4f,%5.4f)", p[i].x, p[i].y);
        }
        printf(" ]\n");
/*
        static int t=0;
        if(t++%100 == 0){
            vision.showStatistics();
        }
        */
    }
#  endif
#  if 0
    if (need_print && vision.helthyPosition()){
        printf("vision: pitch=%4.3f , roll=%4.3f , yaw=%4.3f\n"
                , vision.getPitch(), vision.getRoll(), vision.getYaw());
        if (natNet.healthy()){
            printf("natnet: pitch=%4.3f , roll=%4.3f , yaw=%4.3f\n"
                    , natNet.get_euler_pitch(), natNet.get_euler_roll(), natNet.get_euler_yaw());
        }
    }
#  endif
#endif // vision

    /**************************/
    /******* pilot RC *********/
    /**************************/
#define RC_STILE 0
    // get pilot's desired throttle
    pilot_throttle = get_pilot_desired_throttle(channel_throttle->get_control_in());  // [0.0 , 1.0]

    pilot_roll = channel_roll->get_control_in(); // [-4500 , +4500] (int)
    pilot_pitch = channel_pitch->get_control_in(); // [-4500 , +4500] (int)
    pilot_yaw = channel_yaw->get_control_in(); // [-4500 , +4500] (int)

    pilot_roll = pilot_roll / 5000.0; // [-1 , +1] (float)
    pilot_pitch = pilot_pitch / 5000.0; // [-1 , +1] (float)
    pilot_yaw = pilot_yaw / 5000.0; // [-1 , +1] (float)

#if 0
    if(need_print && 0){
        printf("throttle = %8.3f, roll = %8.3f, pitch = %8.3f yaw = %8.3f\n", pilot_throttle, pilot_roll, pilot_pitch, pilot_yaw);
        printf("location x = %8.3f, y = %8.3f z = %8.3f\n", cur_x, cur_alt, cur_z);
    }
#endif

    /************************/
    /***** controller *******/
    /************************/

    // high level controller
#define WINDOW_SIZE_TARGET 1.8570
#define WINDOW_SIZE_MUL_FACTOR 0.67
    static float centerOfMass_x=0.0f; // 'S_x' from hanoch paper
    static float centerOfMass_y=0.0f; // 'S_y' from hanoch paper
    static float visionDistance =0.0f; // Average size of line in the window
    static float vertical_diff=0.0f;  // 'V_d' from hanoch paper
    static float vertical_diff_filt=0.0f;  // 'V_d' from hanoch paper

    //static float dotsAngle[4];

    static int lastPointCount = 0;
#if VISION
    if(newVisionData){
        if (4 == pointCount){
            //lastPointCount = pointCount;

            centerOfMass_x = (p[0].x + p[1].x + p[2].x + p[3].x ) / 4.0;

            centerOfMass_y = (p[0].y + p[1].y + p[2].y + p[3].y ) / 4.0;

            visionDistance = (p[3].y - p[0].y) + (p[2].y - p[1].y)
                             + (p[0].x - p[1].x) + (p[3].x - p[2].x);
            visionDistance *= WINDOW_SIZE_MUL_FACTOR;
            visionDistance += WINDOW_SIZE_TARGET;

            float temp_Vd =  -(p[0].y - p[3].y - (p[1].y - p[2].y)) /
                    (p[0].y - p[3].y + (p[1].y - p[2].y));
            if (0.3 < (temp_Vd - vertical_diff_filt) ||
                    -0.3 > (temp_Vd - vertical_diff_filt)){
                temp_Vd = vertical_diff_filt;
            }

            // add distance factor (linear aproximation)
            if (visionDistance < -1){
                temp_Vd = 0;
            } else {
                temp_Vd = temp_Vd * (1+visionDistance) * 0.65;
            }

            vertical_diff = temp_Vd;

            myLogger.putVal(log_VISION_Vd, vertical_diff);
            switch (resMode) {
            case 0: // DD (240, 320)
                vertical_diff_filt = vertical_diff_filt*0.95f + vertical_diff * 0.05f;
                break;
            case 1: // UD (480, 640)
                vertical_diff_filt = vertical_diff_filt*0.95f + vertical_diff * 0.05f;
                break;
            case 2: // DU (720, 960)
                vertical_diff_filt = vertical_diff_filt*0.92f + vertical_diff * 0.08f;
                break;
            case 3: // UU (960, 1280)
                vertical_diff_filt = vertical_diff_filt*0.85f + vertical_diff * 0.15f;
                break;

            default:
                vertical_diff_filt = 0;
                break;
            }

            // dots angle

        } else if (3 == pointCount){
            // keep last values
#if 0
        } else if (2 == pointCount && 2 <= lastPointCount){
            if(centerOfMass_y < -0.4 &&  // if was low
                    p[0].y < 0 && p[1].y < 0){
                // too low
                centerOfMass_y = -0.8f;

            } else if(centerOfMass_y > +0.4 && // if was high
                    p[0].y > 0 && p[1].y > 0){
                // too high
                centerOfMass_y = +0.8f;
            }

            if(centerOfMass_x < -0.4 &&  // if was right
                    p[0].x < 0 && p[1].x < 0){
                // too right
                centerOfMass_x = -0.8f;

            } else if(centerOfMass_x > +0.4 && // if was left
                    p[0].x > 0 && p[1].x > 0){
                // too left
                centerOfMass_x = +0.8f;
            }
            lastPointCount = pointCount;
#endif
        } else if (2 == pointCount && 2 == lastPointCount){
            if( p[0].y > 0.2 && p[1].y > 0.2 &&                                 // if is low
                    (0.2 > (p[0].y - p[1].y)) && (-0.2 < (p[0].y - p[1].y)) ){  // if is horizontal line
                // too low
                centerOfMass_x = (p[0].x + p[1].x) / 2.0f;
                //centerOfMass_y = 0.8;
                /*
                visionDistance = (p[0].x - p[1].x)*WINDOW_SIZE_MUL_FACTOR;
                if (0.0 > visionDistance){
                    visionDistance = -visionDistance;
                }
                visionDistance -= WINDOW_SIZE_TARGET;
                */
                visionDistance = visionDistance*0.7;

            }

        } else {

            if(2 > pointCount && 2 > lastPointCount){
                centerOfMass_x = 0.0;
                centerOfMass_y = 0.0;
                visionDistance = 0;
            } else{
                centerOfMass_x *= 0.95;
                centerOfMass_y *= 0.95;
            }
            lastPointCount = 0;
        }
        lastPointCount = pointCount;
    } // newVisionData

    myLogger.putVal(log_window_size, visionDistance);
    myLogger.putVal(log_VISION_Sx, centerOfMass_x);
    myLogger.putVal(log_VISION_Sy, centerOfMass_y);
    myLogger.putVal(log_VISION_Vd_filt, vertical_diff_filt);

#endif

// low level controller
#define KP_P 0.10f
#define KP_Q 0.12f
//#define KP_R 0.8f
#define KP_R 1.4f
#if NATNET
    static float last_alt, vertical_vel_filt;
    #if NATNET_POS_HOLD
        static float last_x, x_vel_filt;
        static float last_z, z_vel_filt;
    #endif
#endif

    static uint32_t last_time_vision_us;
    float delta_time_vision_sec = 0;
    if (newVisionData){
        if (now_us < last_time_vision_us){ // handle 32bit overflow (every 1:10 Hours)
            last_time_vision_us = 0xFFFF - last_time_vision_us;
            delta_time_vision_sec = (float)(now_us + last_time_vision_us) / 1000.0f/1000.0f; // uS -> sec
        } else {
            delta_time_vision_sec = (float)(now_us - last_time_vision_us) / 1000.0f/1000.0f; // uS -> sec
        }
        last_time_vision_us = now_us;
        if (delta_time_vision_sec == 0){
            printf("zero!\n");
            delta_time_vision_sec = 0.0000005f;
        }

        myLogger.putVal(log_VISION_delay, delta_time_vision_sec);
    }

    static uint32_t last_time_us;
    float delta_time_sec;

    if (now_us < last_time_us){ // handle 32bit overflow (every 1:10 Hours)
        last_time_us = 0xFFFF - last_time_us;
        delta_time_sec = (float)(now_us + last_time_us) / 1000.0f/1000.0f; // uS -> sec
    } else {
        delta_time_sec = (float)(now_us - last_time_us) / 1000.0f/1000.0f; // uS -> sec
    }
    last_time_us = now_us;
    if (delta_time_sec == 0){
        printf("zero!\n");
        delta_time_sec = 0.0000005f;
    }


    /* roll */
    /********/
    float pilot_roll_scaled = pilot_roll; // * 1.1f;        // scale RC input


    #if NATNET && NATNET_POS_HOLD
        #define KP_X 0.25f
        #define KD_X 0.12f

        #if VISION && VISION_CONTROL

            #if CONVERGENCE_TEST
                if (RCIN(7) > 1500) { // AUX1  - only 240P or 960P
                    // move aside
                    if (!natNet.healthy()){
                        // use only camera
                        cur_x = vertical_diff_filt * 30.0f; // Override optitrack (*20)
                    }
                    cur_x +=0.7;
                } else {
                    // back to normal control
                    cur_x = vertical_diff_filt * 30.0f; // Override optitrack (*20)
                }
            #else
                cur_x = vertical_diff_filt * 30.0f; // Override optitrack (*20)
            #endif


            if (newVisionData) {
                float x_vel;
                if (0.4 < (last_x - cur_x) || -0.4 > (last_x - cur_x)){
                    last_x = cur_x;
                }
                if (4 == pointCount){       // if good data
                    float delta_x = cur_x - last_x;
                    x_vel = delta_x / delta_time_vision_sec;
                } else {
                    x_vel = 0.0f;
                }

                x_vel_filt = x_vel_filt*0.85f + x_vel*0.15f;
                //x_vel_filt = x_vel_filt*0.98f + x_vel*0.02f;
                last_x = cur_x;
                myLogger.putVal(log_vel_x, x_vel);
                //printf("delta_time_vision_sec = %f, x_vel= %f, last_x= %f\n", delta_time_vision_sec, cur_x, last_x);
            }
        #else
            float delta_x = cur_x - last_x;
            float x_vel = delta_x / delta_time_sec;
            last_x = cur_x;
            x_vel_filt = x_vel_filt*0.9f + x_vel*0.1f;
        #endif

        myLogger.putVal(log_vel_x_filt, x_vel_filt);

        pilot_roll_scaled += -KP_X*cur_x*0.8 -KD_X*x_vel_filt*2.5;
    #endif

    float error_angle_roll = (pilot_roll_scaled - 2*roll_angle)*3.0f;
    float error_vel_roll = error_angle_roll - insGyro.x;
    float roll_cmd = error_vel_roll * KP_P;               // the output!
    roll_cmd = constrain_float(roll_cmd, -1.0f, +1.0f);



    /* pitch */
    /*********/
    float pilot_pitch_scaled = pilot_pitch; // * 0.93f;        // scale RC input

    #if NATNET && NATNET_POS_HOLD
        #define KP_Z 0.27f
        #define KD_Z 0.12f

        #if VISION && VISION_CONTROL

            #if CONVERGENCE_TEST
                if (RCIN(7) > 1500) { // AUX1  - only 240P or 960P
                    // move aside
                    if (!natNet.healthy()){
                        // use only camera
                        cur_z = visionDistance; // Override optitrack (*20)
                    }
                    //use the natnet data
                } else {
                    // back to normal control
                    cur_z = visionDistance; // Override optitrack (*20)
                }
            #else
                cur_z = visionDistance; // Override optitrack (*20)
            #endif


            if (newVisionData) {
                float z_vel;
                if (4 == pointCount){       // if good data
                    float delta_Z = cur_z - last_z;
                    z_vel = delta_Z / delta_time_vision_sec;
                } else {
                    z_vel = 0.0f;
                }

                z_vel_filt = z_vel_filt*0.9f + z_vel*0.1f;
                //x_vel_filt = x_vel_filt*0.98f + x_vel*0.02f;
                last_z = cur_z;
                //myLogger.putVal(log_vel_z, z_vel);
                //printf("delta_time_vision_sec = %f, x_vel= %f, last_x= %f\n", delta_time_vision_sec, cur_x, last_x);
            }
        #else

            float delta_Z = cur_z - last_z;
            float z_vel = delta_Z / delta_time_sec;
            last_z = cur_z;
            z_vel_filt = z_vel_filt*0.9f + z_vel*0.1f;

        #endif


        pilot_pitch_scaled += -KP_Z*cur_z -KD_Z*z_vel_filt;
    #endif

    float error_angle_pitch = (pilot_pitch_scaled - 2*pitch_angle)*3.0f;
    float error_vel_pitch = error_angle_pitch - insGyro.y;
    float pitch_cmd = error_vel_pitch * KP_Q;               // the output!
    pitch_cmd = constrain_float(pitch_cmd, -1.0f, +1.0f);



    /* yaw */
    /*******/
    float pilot_yaw_scaled = pilot_yaw * 1.6f;           // scale RC input

    #if VISION && VISION_CONTROL
        if (vision.healthy() && 4 == pointCount){
            yaw_angle = centerOfMass_x;
        } else {
            yaw_angle = centerOfMass_x;
        }
    #endif

    float yaw_scaled = yaw_angle * 1.8f;   // was 1.6
    float error_vel_yaw = pilot_yaw_scaled + yaw_scaled - insGyro.z;
    float yaw_cmd  = error_vel_yaw * KP_R;               // the output!
    yaw_cmd = constrain_float(yaw_cmd, -1.0f, +1.0f);

    /* throttle */
    static float throtle_cmd = 0;
    static float throtle_I = 0;
    #if NATNET
        //if (natNet.initialized()) {
            cur_alt = -centerOfMass_y;
            if (newVisionData){
                float vertical_vel;
                if (4 == pointCount){
                    float delta_alt = cur_alt - last_alt;
                    vertical_vel = delta_alt / delta_time_vision_sec;
                } else {
                    vertical_vel = 0;
                }
                last_alt = cur_alt;
                vertical_vel_filt = vertical_vel_filt*0.8f + vertical_vel*0.2f;
                myLogger.putVal(log_vel_y_filt, vertical_vel_filt);
                myLogger.putVal(log_vel_y, vertical_vel);
            }

            if (4 != pointCount){
                vertical_vel_filt = vertical_vel_filt*0.95f;
            }

            //float pilot_throtle_scaled = pilot_throttle*1.8;
            float error_throtle = (0.0f - cur_alt)*0.2f;
            //float hover_throtle = 0.3f; // full battery: 0.3;

            // Safety first - constrain_float() convert 'nan' to 0.5
            //throtle_cmd = (hover_throtle + error_throtle - vertical_vel_filt*0.1f);

            if (motors.armed() && !ap.throttle_zero && 4 == pointCount){
                throtle_I += error_throtle * 0.0001;
            }
            #define throtle_I_LINIT 0.15
            if (throtle_I > throtle_I_LINIT){
                throtle_I = throtle_I_LINIT;
            } else if (throtle_I < -throtle_I_LINIT){
                throtle_I = -throtle_I_LINIT;
            }

            throtle_cmd = ((pilot_throttle + throtle_I) + error_throtle - vertical_vel_filt*0.1f);
            if(isnan(throtle_cmd)){
                throtle_cmd = 0;
            } else {
                throtle_cmd = constrain_float( throtle_cmd , 0.0f, +1.0f);
            }
        //} else {
        //    throtle_cmd = pilot_throttle;
       // }
    #else
        throtle_cmd = pilot_throttle;
    #endif


        myLogger.putVal(log_cmd_pitch, pitch_cmd);
        myLogger.putVal(log_cmd_roll, roll_cmd);
        myLogger.putVal(log_cmd_yaw, yaw_cmd);
        myLogger.putVal(log_cmd_throtle, throtle_cmd);

    /*******************/
    /**** failsafe *****/
    /*******************/
#define MAX_ANGLE_ALLOW 0.8
    if ((roll_angle < -MAX_ANGLE_ALLOW || roll_angle > MAX_ANGLE_ALLOW) ||
            (pitch_angle < -MAX_ANGLE_ALLOW || pitch_angle > MAX_ANGLE_ALLOW)){
        motors.armed(false);
        motors.output();
    }

    /*********************/
    /***** arm check *****/
    /*********************/

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        //motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        //attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        motors.set_roll(0.0);
        motors.set_pitch(0.0);
        motors.set_yaw(0.0);
        motors.set_throttle(0.0);

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

#if 0
    if (need_print){
        printf("pilot_throttle = %8.3f, alt_filt=%8.3f, alt_healthy=%d\n",
                pilot_throttle,
                cur_alt,
                rangefinder_state.alt_healthy);
    }
#endif
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

#if 0
    uint32_t optFlow_interval_ms = 100; // apply opticflow at 10Hz

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
