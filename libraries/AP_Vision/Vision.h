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
 *       Vision.h - Vision Base Class for Ardupilot
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>

#include "RaspiCamDriver.h"


struct ip_point{
    float x;
    float y;
    int mass; // pixel count
    int radius;
    int quality; // TODO - not used
};

struct ip_data{
    std::vector<struct ip_point> points;
    int nPoints; // number of points found
};

class Vision
{
private:

    struct AP_OpticalFlowPi_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
        uint8_t cameraError : 1;    // true if there is camera error
    } _flags;

    struct ip_data _data;

    uint32_t _last_update_ms;
    RaspiCamDriver _camDriv;

    int _height;
    int _width;

    cv::Mat _mat;

public:
    // constructor
    Vision();

    // init - initialise sensor (factor of 240X320)
    // 480 640
    // 960 1280
    void init(int height=480, int width=640);
#if 0
    // make runtime errors
    void setResolution(int height, int width);
#endif
    // healthy - return true if the sensor is healthy
    bool healthy() const { return _flags.healthy && !_flags.cameraError; }

    /* return true if new data arrived */
    virtual bool update(void);

    // quality - returns the surface quality as a measure from 0 ~ 255
    //uint16_t quality(uint camSelector) const { return _optFlowData[camSelector%2].quality; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update(uint cameraSelector) const { return _last_update_ms; }

    // time (ms) between last iteration and current iteration
    //uint32_t getDeltaTime_ms(uint cameraSelector) const { return _optFlowData[cameraSelector%2].deltaTime_ms; }

    int getBestPoints(struct ip_point* points, int count, bool regulate=true);

    int getPointsCount() { return _data.nPoints; }

    void showStatistics();

    // for testing
    cv::Mat* getImage() { return &_mat; }

private:

    int dfs(cv::Mat& mat, int x, int y, uchar trashold, int level=0);

    void findDots(cv::Mat& mat, struct ip_data& ipData);

};

