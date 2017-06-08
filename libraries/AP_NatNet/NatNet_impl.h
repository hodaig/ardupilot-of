/*
 * NatNet_impl.h
 *
 *  Created on: Mar 13, 2017
 *      Author: hodai
 */

#ifndef LIBRARIES_AP_NATNET_NATNET_IMPL_H_
#define LIBRARIES_AP_NATNET_NATNET_IMPL_H_

#include <AP_Math/AP_Math.h>

class FrameListener;
class CommandListener;
class MocapFrame;


class NatNet_impl {
private:
    // Version number of the NatNet protocol, as reported by the server.
    unsigned char _natNetMajor;
    unsigned char _natNetMinor;

    // Sockets
    int _sdCommand;
    int _sdData;

    FrameListener*   _frameListener;
    CommandListener* _commandListener;

    bool _runing;

public:

    struct raw{
        MocapFrame* data;
        struct timespec timestamp;
        bool valid;
    } _raw;

    virtual ~NatNet_impl();
    NatNet_impl();

    bool init(const char* serverAddress, const char* localAddress);

    void terminate();

    bool grab();

    bool getLocation( Vector3f& Vout);

    bool getOrientation( Quaternion& Qout);

    bool getOrientation(float* pitch, float* roll, float* yaw);

    bool isValid();

};

#endif /* LIBRARIES_AP_NATNET_NATNET_IMPL_H_ */
