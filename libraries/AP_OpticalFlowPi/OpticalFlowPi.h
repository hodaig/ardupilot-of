/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

/*
 *       OpticalFlowPi.h - OpticalFlowPi Base Class for Ardupilot
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
//#include <cv.h>
//#include <highgui.h>
//#include <math.h>

//#include <ctime>
//#include <iostream>
#include <raspicam/raspicam_cv.h>

class AP_AHRS_NavEKF;

struct of_stat_t{
    bool initialized = false;

    char* name;
    uint32_t iterations;
    uint32_t total_us;
    uint32_t max_us;
    uint32_t avg;

    /* temp */
    uint32_t begin;
};

struct opticflowDada_t{
    std::vector<cv::Point2f> corners1;
    std::vector<cv::Point2f> corners2;
    std::vector<uchar> status;
    std::vector<float> err;

    /* results */
    uint16_t numberOfGoodFeatures;
    uint16_t quality;

    Vector2f avgSpeed;
    double angle;

    uint32_t last_update_ms;  // millis() time of last optic flow update
    uint32_t deltaTime_ms;    // time from previous to current calculation

    uint8_t valid;              // true if data is valid

};

struct opticflowCamera_t{
    cv::Mat mat;
    cv::Mat grayFrame1;
    cv::Mat grayFrame2;
    cv::Mat* grayFirst_p;       // pointer to grayFrame1 or grayFrame2
    cv::Mat* graySecond_p;      // pointer to grayFrame1 or grayFrame2

    /* statistics */
    struct of_stat_t stat_optFlow;
    struct of_stat_t stat_grab;
    struct of_stat_t stat_detectCorners;
    struct of_stat_t stat_calcOf;

};

class OpticalFlowPi
{

public:
    // constructor
    OpticalFlowPi(AP_AHRS_NavEKF& ahrs);

    // init - initialise sensor
    void init(void);

    // enabled - returns true if optical flow is enabled
    //bool enabled() const { return _enabled; }

    // healthy - return true if the sensor is healthy
    bool healthy() const { return (healthy(0) && healthy(1)); }

    bool healthy(uint camSelector) const { return (_optFlowData[camSelector%2].valid && !_flags.cameraError && !_flags.gpioError); }

    // read latest values from sensor and fill in x,y and totals.
    void update(void);

    // quality - returns the surface quality as a measure from 0 ~ 255
    uint16_t quality(uint camSelector) const { return _optFlowData[camSelector%2].quality; }

    // raw - returns the raw movement from the sensor
    //const Vector2f& flowRate() const { return _state.flowRate; }

    // velocity - returns the velocity in m/s
    //const Vector2f& bodyRate() const { return _state.bodyRate; }

    const Vector2f& getVel(uint camSelector) { return _optFlowData[camSelector%2].avgSpeed; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update() const { return MAX(last_update(0) , last_update(1));}
    uint32_t last_update(uint cameraSelector) const { return _optFlowData[cameraSelector%2].last_update_ms; }

    // time (ms) between last iteration and current iteration
    uint32_t getDeltaTime_ms(uint cameraSelector) const { return _optFlowData[cameraSelector%2].deltaTime_ms; }

    void showStatistics();

private:
    void nextFrame(struct opticflowCamera_t* camData);
    void switchCamera();

    int runOptFlow(opticflowCamera_t* camData, opticflowDada_t* ofData, uint8_t state);
    int updateAvgVelocity(int number_of_features, opticflowDada_t* ofData);

private:

    struct AP_OpticalFlowPi_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
        uint8_t gpioError   : 1;    // true if there is gpio error
        uint8_t cameraError : 1;    // true if there is camera error
    } _flags;

    // state filled in by backend
    //struct OpticalFlowPi_state _state;


    raspicam::RaspiCam_Cv _camera;
    struct opticflowCamera_t _camData[2];
    struct opticflowDada_t _optFlowData[2];

    uint8_t _cameraSelector; // 0 or 1
    AP_HAL::DigitalSource* _selectorPin;

    //uint32_t _last_update_ms[2];        // millis() time of last opric flow update
    uint32_t _selector_update_ms;           // millis() time of last GPIO selector pin update

    uint8_t _stateMachine;
};

