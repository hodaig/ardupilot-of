/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "NatNet_client.h"
#include "NatNet_impl.h"

#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace std;

const char* NatNet_client::_paramsServer = 0;
const char* NatNet_client::_paramsLocal = 0;

// default constructor
NatNet_client::NatNet_client()
{
    printf("starting NatNet::NatNet\n");

    // healthy flag will be overwritten on update
    _flags.healthy = false;
    _flags.ready = false;


}

void NatNet_client::init()
{
    if ((_paramsServer == 0) || (_paramsLocal == 0)){
        printf("init NatNet fail - please define host and local addresses\n");
        _flags.ready = false;
        return;
    }

    printf("init NatNet with server: %s, local: %s\n", _paramsServer, _paramsLocal);
    _impl = new NatNet_impl();
    _impl->init(_paramsServer, _paramsLocal);
    printf("init NatNet Done.\n");

    _flags.ready = true;
}

void NatNet_client::update(void)
{
    if (!_flags.ready){
        _flags.healthy = false;
        //printf("NatNet_client::update: !_flags.ready");
        return;
    }

    if(_impl->grab()){
        if(!_impl->getLocation(_location) ||
                !_impl->getOrientation(&_pitch, &_roll, &_yaw) ){
            _flags.healthy = false;
        }
    } else {
        _flags.healthy = false;
    }
    _flags.healthy = true;
}

const Vector3f& NatNet_client::getLocation(){
    return _location;
}

const Quaternion& NatNet_client::getOrientation(){
    Quaternion o;
    o.from_euler(_roll, _pitch,_yaw);
    return o;
#if 0
           // If Motive is streaming Z-up, convert to this renderer's Y-up coordinate system
           if (upAxis==2)
           {
               // convert position
               ConvertRHSPosZupToYUp(x, y, z);
               // convert orientation
               ConvertRHSRotZUpToYUp(q.x, q.y, q.z, q.w);
           }
#endif
}

float NatNet_client::get_euler_roll(){
    return _roll;
}
float NatNet_client::get_euler_pitch(){
    return _pitch;
}
float NatNet_client::get_euler_yaw(){
    return _yaw;
}

