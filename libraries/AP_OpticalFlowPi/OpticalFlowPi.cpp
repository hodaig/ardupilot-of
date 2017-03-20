/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "OpticalFlowPi.h"

extern const AP_HAL::HAL& hal;

//#include <stdio.h>
//#include <cv.h>
//#include <highgui.h>
//#include <math.h>
#include <opencv2/video/tracking.hpp>

//#include <ctime>
//#include <iostream>
//#include <raspicam/raspicam_cv.h>
using namespace std;

void STATS_SHOW(struct of_stat_t& stat){
    printf("*** stat - %s: avg=%d, max=%d\n", stat.name, stat.avg, stat.max_ms);
}

void STAT_INIT(struct of_stat_t& stat, char* name){
        stat.initialized = true;
        stat.iterations = 0;
        stat.max_ms = 0;
        stat.total_ms = 0;
        stat.avg = 0;
        stat.name = name;
}

void STATS_START(struct of_stat_t& stat){
    if (!stat.initialized){
        STAT_INIT(stat, "noName");
    }
    stat.begin = AP_HAL::millis();
}
void STATS_END(struct of_stat_t& stat){
    int dif = AP_HAL::millis() - stat.begin;
    if(dif < 0){
        return;
    }
    if (stat.begin != 0){
        stat.iterations++;
        stat.total_ms += (uint32_t)dif;
        stat.avg = stat.total_ms / stat.iterations;
        if (stat.max_ms < (uint32_t)dif){
            stat.max_ms = (uint32_t)dif;
        }
    }
    stat.begin = 0;
}

int prepareInVid(raspicam::RaspiCam_Cv* Camera){
    uint8_t res;
    uint8_t attempts = 3;

    //set camera params
    Camera->set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    Camera->set( CV_CAP_PROP_FRAME_HEIGHT, 240 );
    Camera->set( CV_CAP_PROP_FRAME_WIDTH, 320 );
    Camera->set( CV_CAP_PROP_FPS, 90 );
    Camera->set( CV_CAP_PROP_BRIGHTNESS, 70);
    //Open camera
    printf("Opening Camera...\n");
    while (attempts-- > 0 && !(res=Camera->open())) {
        Camera->release();
    }

    if (res){
        printf("done opening the camera attempts=%d \n", attempts);
    } else {
        printf("Error opening the camera\n");
        return -1;
    }

    //wait a while until camera stabilizes
    //usleep(3*1000000);

    return 0;
}

int endInVid(raspicam::RaspiCam_Cv* Camera){
    printf("Stop camera...\n");
    Camera->release();
    return 0;
}

void grabFrame(raspicam::RaspiCam_Cv* Camera, cv::Mat& mat){
    cv::Mat src;
    Camera->grab();
    Camera->retrieve ( src);
    cv::resize(src, mat, cv::Size(160, 120), 0, 0, cv::INTER_CUBIC); // 320x240 to 80x60
}

int OpticalFlowPi::updateAvgVelocity(int number_of_features, opticflowDada_t* ofData) {
    int res = 0;

    /* calculate the average velocity */
    static const float MAX_ERR_ALLOW = 5;
    double totalX = 0;
    double totalY = 0;
    uint16_t quality = 0; // based on number of features and their error level
    int number_of_good_features = 0;
    for (int i = 0; i < number_of_features; i++) {
        /* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
        if (ofData->status[i] == 0) {
            continue;
        } else if (MAX_ERR_ALLOW < ofData->err[i]) {
            continue;
        }

        /*
         totalX1 += frame1_features[i].x;
         totalY1 += frame1_features[i].y;
         totalX2 += frame2_features[i].x;
         totalY2 += frame2_features[i].y;
         */
        totalX += (ofData->corners1[i].x - ofData->corners2[i].x) * (MAX_ERR_ALLOW - ofData->err[i]);
        totalY += (ofData->corners1[i].y - ofData->corners2[i].y) * (MAX_ERR_ALLOW - ofData->err[i]);
        number_of_good_features+= (MAX_ERR_ALLOW - ofData->err[i]);
        quality += (MAX_ERR_ALLOW - ofData->err[i]);
        //printf("err=%f\n", err[i]);
    }
    ofData->numberOfGoodFeatures = number_of_good_features;
    ofData->quality = quality;
    if (number_of_good_features <= 0) {
        printf("ERROR - no good features \n");
        res = -1;
    }
    float avg_X = totalX / number_of_good_features;
    float avg_Y = totalY / number_of_good_features;
    if (-100000.0 < avg_X && -100000.0 < avg_Y && avg_X < 100000.0 && avg_Y < 100000.0) {
        ofData->avgSpeed.x = avg_X;
        ofData->avgSpeed.y = avg_Y;
        ofData->angle = atan2(ofData->avgSpeed.y, ofData->avgSpeed.x);
    } else {
        printf("ERROR - opticflow value too high\n");
        res = -1;
    }

    if (0 == res){
        ofData->valid = true;
    } else {
        ofData->valid = false;
        ofData->quality = 0;
    }

    return res;
}

int OpticalFlowPi::runOptFlow(opticflowCamera_t* camData, opticflowDada_t* ofData, uint8_t state){

    /* Preparation: BEFORE the function call this variable is the array size
     * (or the maximum number of features to find).  AFTER the function call
     * this variable is the number of features actually found.
     */
    /* I'm hardcoding this at 400.  But you should make this a #define so that you can
     * change the number of features you use for an accuracy/speed tradeoff analysis.
     */
    int number_of_features = 50;

    if (state == 1){
        STATS_START(camData->stat_detectCorners);
#if 0
        if( grayFrame1.size() != grayFrame2.size() ||
                grayFrame1.type() != grayFrame2.type() ){
            printf("Error - grayFrame1.size() != grayFrame2.size() || grayFrame1.type() != grayFrame2.type()\n");
            //return -1;
        }
#endif

        /*
    if( grayFrame1.checkVector(2, CV_32F, true) < 0 ||
            grayFrame2.checkVector(2, CV_32F, true) < 0 ){
        printf("Error - grayFrame1.checkVector(2, CV_32F, true) < 0 || grayFrame2.checkVector(2, CV_32F, true) < 0\n");
        return -1;
    }
         */

        try {
            /* Actually run the Shi and Tomasi algorithm!!
             * "frame1_1C" is the input image.
             * "eig_image" and "temp_image" are just workspace for the algorithm.
             * The first ".01" specifies the minimum quality of the features (based on the eigenvalues).
             * The second ".01" specifies the minimum Euclidean distance between features.
             * "NULL" means use the entire input image.  You could point to a part of the image.
             * WHEN THE ALGORITHM RETURNS:
             * "frame1_features" will contain the feature points.
             * "number_of_features" will be set to a value <= 400 indicating the number of feature points found.
             */
            //cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .01, .01, NULL);
            cv::goodFeaturesToTrack(*(camData->grayFirst_p), ofData->corners1, number_of_features, .01, .01);
        }catch(...) {
            printf("Error - exeption in 'goodFeaturesToTrack'\n");
            STATS_END(camData->stat_detectCorners);
            return -1;
        }
        STATS_END(camData->stat_detectCorners);

    } else if(state == 2){
        STATS_START(camData->stat_calcOf);
#if 0
        if( ((cv::OutputArray)(ofData->corners1)).getMat().checkVector(2, CV_32F, true) < 0){
            printf("Error - ((cv::OutputArray)(ofData->corners1)).getMat().checkVector(2, CV_32F, true) < 0\n");
            //return -1;
        }
#endif
        try {
            /* Actually run Pyramidal Lucas Kanade Optical Flow!!
             * "frame1_1C" is the first frame with the known features.
             * "frame2_1C" is the second frame where we want to find the first frame's features.
             * "pyramid1" and "pyramid2" are workspace for the algorithm.
             * "frame1_features" are the features from the first frame.
             * "frame2_features" is the (outputted) locations of those features in the second frame.
             * "number_of_features" is the number of features in the frame1_features array.
             * "optical_flow_window" is the size of the window to use to avoid the aperture problem.
             * "5" is the maximum number of pyramids to use.  0 would be just one level.
             * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
             * "optical_flow_feature_error" is as described above (error in the flow for this feature).
             * "optical_flow_termination_criteria" is as described above (how long the algorithm should look).
             * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
             */
            //cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, optical_flow_window, 10, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );
            cv::calcOpticalFlowPyrLK(*(camData->grayFirst_p), *(camData->graySecond_p), ofData->corners1, ofData->corners2, ofData->status, ofData->err);  //TODO - c++
        }catch(...) {
            printf("Error - exeption in 'calcOpticalFlowPyrLK'\n");
            STATS_END(camData->stat_calcOf);
            return -1;
        }
        STATS_END(camData->stat_calcOf);

    } else if(state >= 3){
        /* calculate the average velocity */
        int res = updateAvgVelocity(number_of_features, ofData);
        return res;
    }

    return 1; // state didn't finished
}

#if 0
int runOpticflow(raspicam::RaspiCam_Cv* camera) {
    int res;
    long current_frame = 0;
    int nCount=100, iteration=0;
    cv::Mat mat, grayFrame1, grayFrame2;

    while(iteration++ < nCount)
    {

        struct opticflowDada_t ofData;

        //time ( &timer_begin );

        grabFrame(camera, mat);
        cv::cvtColor(mat, grayFrame1, CV_BGR2GRAY);

        /* Get the second frame of video.  Same principles as the first. */
        grabFrame(camera, mat);
        cv::cvtColor(mat, grayFrame2, CV_BGR2GRAY);


        /* Preparation: BEFORE the function call this variable is the array size
         * (or the maximum number of features to find).  AFTER the function call
         * this variable is the number of features actually found.
         */
        int number_of_features;

        /* I'm hardcoding this at 400.  But you should make this a #define so that you can
         * change the number of features you use for an accuracy/speed tradeoff analysis.
         */
        number_of_features = 100;


        if (0 != (res=runOptFlow(grayFrame1, grayFrame2, number_of_features, &ofData))){
            return res;
        }

        /* take statistics */
        /*
        time ( &timer_end );
        secondsElapsed = difftime ( timer_end,timer_begin );
        if (secondsMax < secondsElapsed){
            secondsMax = secondsElapsed;
        }
        secondsAvg += (secondsElapsed - secondsAvg)/iteration;
        */

        //printf("frame %d:  angle:%f dx:%f dy:%f  stats(sec): cur:%f avg:%f max:%f \n", current_frame, ofData.angle, ofData.avgY, ofData.avgX,
          //      secondsElapsed, secondsAvg, secondsMax);
        printf("frame %d:  angle:%f dx:%f dy:%f \n", current_frame, ofData.angle, ofData.avgY, ofData.avgX);
        current_frame++;

    }

    return res;
}
#endif

// default constructor
OpticalFlowPi::OpticalFlowPi(AP_AHRS_NavEKF &ahrs)
    : _cameraSelector(0)
{
        printf("starting OpticalFlowPi::OpticalFlowPi\n");
        // healthy flag will be overwritten on update
        _flags.healthy = false;
        _flags.gpioError = false;
        _flags.cameraError = false;
        _optFlowData[0].valid = false;
        _optFlowData[1].valid = false;
        _optFlowData[0].last_update_ms = 0;
        _optFlowData[1].last_update_ms = 0;

}

void OpticalFlowPi::init(void)
{

    printf("init\n");
    _selector_update_ms = AP_HAL::millis();

    // for now pin number will be hard coded
    printf("OpticalFlowPi::OpticalFlowPi config out pin\n");
    _selectorPin = hal.gpio->channel(17);
    if (_selectorPin == NULL) {
        // failed to allocate a ADC channel? This shouldn't happen
        printf("Error - OpticalFlowPi::OpticalFlowPi config out pin\n");
        _flags.gpioError = true;
        return;
    }
    printf("OpticalFlowPi::OpticalFlowPi done config out pin\n");
    _selectorPin->mode(HAL_GPIO_OUTPUT);
    printf("OpticalFlowPi::OpticalFlowPi done pin mode out\n");
    _selectorPin->write(_cameraSelector);
    printf("OpticalFlowPi::OpticalFlowPi done pin write\n");

    //if (0 != (res=prepareInVid(&(_cam1.Camera)))){
    if (0 != prepareInVid(&_camera)){
        _flags.cameraError = true;
        printf("Error: fail to init camera\n");
        //return res;
    }


    _camData[0].grayFirst_p = &(_camData[0].grayFrame1);
    _camData[0].graySecond_p = &(_camData[0].grayFrame2);
    _camData[1].grayFirst_p = &(_camData[1].grayFrame1);
    _camData[1].graySecond_p = &(_camData[1].grayFrame2);
    grabFrame(&_camera, *(_camData[0].grayFirst_p));
    grabFrame(&_camera, *(_camData[1].grayFirst_p));
    grabFrame(&_camera, *(_camData[0].graySecond_p));
    grabFrame(&_camera, *(_camData[1].graySecond_p));


    STAT_INIT(_camData[0].stat_optFlow, "stat_optFlow0");
    STAT_INIT(_camData[0].stat_grab, "stat_grab0");
    STAT_INIT(_camData[0].stat_detectCorners, "stat_detectCorners0");
    STAT_INIT(_camData[0].stat_calcOf, "stat_calcOf0");
    STAT_INIT(_camData[1].stat_optFlow, "stat_optFlow1");
    STAT_INIT(_camData[1].stat_grab, "stat_grab1");
    STAT_INIT(_camData[1].stat_detectCorners, "stat_detectCorners1");
    STAT_INIT(_camData[1].stat_calcOf, "stat_calcOf1");

    _flags.healthy = true;
    //endInVid(&Camera);
}

void OpticalFlowPi::nextFrame(struct opticflowCamera_t* camData){
    STATS_START(camData->stat_grab);

    // flip frames
    cv::Mat* tempFrame = camData->grayFirst_p;
    camData->grayFirst_p = camData->graySecond_p;
    camData->graySecond_p = tempFrame;

    grabFrame(&_camera, *(camData->graySecond_p));
    //grabFrame(&_camera, data->mat);
    //cv::cvtColor(data->mat, *(data->graySecond_p), CV_BGR2GRAY);

    STATS_END(camData->stat_grab);
}

void OpticalFlowPi::switchCamera(){
    uint32_t now = AP_HAL::millis();
    _cameraSelector = (_cameraSelector + 1) % 2;
    _selectorPin->write(_cameraSelector);
    _selector_update_ms = now;
}

void OpticalFlowPi::showStatistics(){
    uint f;
    for (uint cam=0; cam < 2 ; cam++){
        //STATS_SHOW(_camData[cam].stat_optFlow);
        STATS_SHOW(_camData[cam].stat_grab);
        STATS_SHOW(_camData[cam].stat_detectCorners);
        STATS_SHOW(_camData[cam].stat_calcOf);
    }
}

void OpticalFlowPi::update(void)
{
    int res = 1;
    //uint32_t timeElapsed_ms;

    //cv::Mat mat, grayFrame1, grayFrame2;
    opticflowCamera_t* camData = &_camData[_cameraSelector];
    opticflowDada_t* ofData = &_optFlowData[_cameraSelector];

    uint32_t startMillis = AP_HAL::millis();
    // in the testing 20ms after pin change is optimal
    if ( (startMillis - _selector_update_ms) < 20){
        // Error - not enough time to camera switch finish
        _stateMachine = 0;
        return;
    }


    //time ( &timer_begin );
    if (_stateMachine == 0){
        ofData->deltaTime_ms = startMillis - ofData->last_update_ms;
        ofData->last_update_ms = startMillis;

        nextFrame(camData);

    } else {

        res = runOptFlow(camData, ofData, _stateMachine);


    }

#if 0
    /* take statistics */
    uint32_t now;
    now = AP_HAL::millis();
    if (now > startMillis){
        timeElapsed_ms = now - startMillis;
        data->totalTime_ms += timeElapsed_ms;
        data->iterations++;
        if (data->maxTime_ms < timeElapsed_ms){
            data->maxTime_ms = timeElapsed_ms;
        }
    }
#endif

    _stateMachine++;
    if (res <= 0){
        // switch to next camera
        switchCamera();
        _stateMachine = 0;
    }
}


