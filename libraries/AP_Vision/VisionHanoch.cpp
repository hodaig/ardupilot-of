/*
 * VisionHanoch.cpp
 *
 *  Created on: May 16, 2017
 *      Author: hodai
 */

#include "VisionHanoch.h"
#include "estimate_from_dots.h"

#define FOV_FACTOR 2.34f // 2.7

VisionHanoch::VisionHanoch() : Vision(), _pitch(0), _roll(0), _yaw(0),
_x(0), _y(0), _z(0), _updatedPos(false), _helthyPos(false){
    // TODO Auto-generated constructor stub

}

VisionHanoch::~VisionHanoch() {
    // TODO Auto-generated destructor stub
}

float VisionHanoch::getPitch(){ return _pitch; }
float VisionHanoch::getRoll(){ return _roll; }
float VisionHanoch::getYaw(){ return _yaw; }

float VisionHanoch::getLocX(){ return _x; }
float VisionHanoch::getLocY(){ return _y; }
float VisionHanoch::getLocZ(){ return _z; }

bool VisionHanoch::estimatePosition(){
    float wy; // not used for now
    int iterations = 100;

    if (_updatedPos){
        return true;
    }
    _updatedPos = true;

    // get the points
    //struct ip_point points[4];
    if (4 != getBestPoints(_pointsVision, 4, true) ){
        return false;
    }

    reorderPoints(_pointsVision);
    //memcpy(&_pointsVision, &points, sizeof(points));

    // now estimate the position
    estimate_from_dots(_pointsVision[0].x, _pointsVision[1].x, _pointsVision[2].x, _pointsVision[3].x
            , _pointsVision[0].y, _pointsVision[1].y, _pointsVision[2].y, _pointsVision[3].y
            , iterations, &_x, &_y, &_z, &_pitch, &_roll, &_yaw, &wy);

    loc_to_points(_x,_y,_z,_pitch, _roll, _yaw, wy,
            &_pointsEstimated[0].x, &_pointsEstimated[1].x, &_pointsEstimated[2].x, &_pointsEstimated[3].x,
            &_pointsEstimated[0].y, &_pointsEstimated[1].y, &_pointsEstimated[2].y, &_pointsEstimated[3].y);


    /* NatNet adaptations */
    _yaw = -_yaw;
    // take in count the camera field of view
    _yaw   *= FOV_FACTOR;
    _pitch *= FOV_FACTOR;

    /* quality check */
    if (1.0 < _roll || _roll < -1.0 ||
            1.0 < _pitch || _pitch < -1.0 ||
            1.0 < _yaw || _yaw < -1.0 ){
        // overshooting
        _helthyPos = false;
        return false;
    } else {
        // looks good
        _helthyPos = true;
        return true;
    }


}

bool VisionHanoch::helthyPosition() {
    return (healthy() && _updatedPos && _helthyPos);
}

/* overide */
bool VisionHanoch::update(void){
    if (Vision::update()){
        _updatedPos = false;

        getBestPoints(_pointsVision, 4, true);
        reorderPoints(_pointsVision);

        return true;
    } else {
        return false;
    }
}


void VisionHanoch::reorderPoints(struct ip_point* points){
    /*
     * Y axis direction corection:
     * vision -> positive is down
     * hanoch algo -> positive is up
     */
    for (int i = 0; i < 4; ++i) {
        points[i].y = -points[i].y;
    }

    if (points[0].x > points[1].x){
        pointSwap(&points[0], &points[1]);
    }
    if (points[3].x > points[2].x){
        pointSwap(&points[3], &points[2]);
    }
}

void VisionHanoch::pointSwap(ip_point* p1, ip_point* p2){
    ip_point ptemp;
    memcpy(&ptemp, p1, sizeof(ip_point));
    memcpy(p1, p2, sizeof(ip_point));
    memcpy(p2, &ptemp, sizeof(ip_point));
}
