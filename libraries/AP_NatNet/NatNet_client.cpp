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
    if (_paramsServer == 0) _paramsServer = "";
    if (_paramsLocal == 0) _paramsLocal = "";
    printf("init NatNet with server: %s, local: %s\n", _paramsServer, _paramsLocal);
    _impl = new NatNet_impl();
    _impl->init(_paramsServer, _paramsLocal);

    _flags.ready = true;
}

void NatNet_client::update(void)
{
    if (!_flags.ready){
        _flags.healthy = false;
        printf("NatNet_client::update: !_flags.ready");
        return;
    }

    if(_impl->grab()){
        if(!_impl->getLocation(_location) ||
                !_impl->getOrientation(_orientation) ){
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
    return _orientation;
}

float NatNet_client::get_euler_roll(){
    return _orientation.get_euler_roll();
}
float NatNet_client::get_euler_pitch(){
    return _orientation.get_euler_pitch();
}
float NatNet_client::get_euler_yaw(){
    return _orientation.get_euler_yaw();
}

