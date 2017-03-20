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

#include <AP_Math/AP_Math.h>
//#include "NatNet_impl.h"
class NatNet_impl;

class NatNet_client
{

public:
    static const char* _paramsServer;
    static const char* _paramsLocal;

    // constructor
    NatNet_client();

    void init();

    bool healthy() const { return _flags.healthy; }

    // read latest values and store them (grab)
    void update(void);

    const Vector3f& getLocation();

    const Quaternion& getOrientation();

    /*
     * TODO - RigidBody orientation
     * If Motive is streaming Z-up, convert to this renderer's Y-up coordinate system
     *
     * based on NatNet-SDK example: /NatNetSDK/Samples/SampleClient3D/
     */

    float get_euler_roll();
    float get_euler_pitch();
    float get_euler_yaw();

private:

    NatNet_impl* _impl;

    struct AP_NatNet_Flags {
        uint8_t healthy     : 1;    // true if healthy
        uint8_t ready       : 1;    // true if initialized
    } _flags;

    Vector3f   _location;
    Quaternion _orientation;

    // TODO
    //uint32_t _lastUpdate;

};

