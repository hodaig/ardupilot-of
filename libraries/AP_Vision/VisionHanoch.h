/*
 * VisionHanoch.h
 *
 * extends the vision capabilities by estimating the position of
 * the frame from the window prespective
 *
 *  Created on: May 16, 2017
 *      Author: hodai
 */

#ifndef LIBRARIES_AP_VISION_VISIONHANOCH_H_
#define LIBRARIES_AP_VISION_VISIONHANOCH_H_

#include "Vision.h"

class VisionHanoch: public Vision {
private:
    float _pitch;
    float _roll;
    float _yaw;

    float _x;
    float _y;
    float _z;

    bool _updatedPos;
    bool _helthyPos;

public:
    // for statistics & debugging
    struct ip_point _pointsVision[4];
    struct ip_point _pointsEstimated[4];

public:
    VisionHanoch();
    virtual ~VisionHanoch();

    bool estimatePosition();

    bool helthyPosition();

    /* overide */
    /* return true if new data arrived */
    virtual bool update(void);

    /* geters */
    float getPitch();
    float getRoll();
    float getYaw();

    float getLocX();
    float getLocY();
    float getLocZ();

    void reorderPoints(struct ip_point* points);

private:

    void pointSwap(ip_point* p1, ip_point* p2);

};

#endif /* LIBRARIES_AP_VISION_VISIONHANOCH_H_ */
