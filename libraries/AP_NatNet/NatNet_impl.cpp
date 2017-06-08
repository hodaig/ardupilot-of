/*
 * NatNet_impl.cpp
 *
 *  Created on: Mar 13, 2017
 *      Author: hodai
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

#include "NatNet_impl.h"

NatNet_impl::~NatNet_impl() {
    terminate();
}

NatNet_impl::NatNet_impl(): _runing(false), _natNetMajor(0), _natNetMinor(0){
    _raw.valid = false;
    _raw.data = new MocapFrame();
}


bool NatNet_impl::grab(){
   bool valid;

   if (!_runing){
       _raw.valid = false;
       return false;
   }

   std::pair<MocapFrame, struct timespec> frame(_frameListener->pop(&valid));

   if( !valid ||
           frame.first.latency() == 0 ||
           frame.first.rigidBodies().size() <= 0 ||
           frame.first.unIdMarkers().size() <= 0 ||
           frame.first.markerSets().size()  <= 0) {
       _raw.valid = false;
       return false;
   }

   _raw.data = new MocapFrame(frame.first);
   _raw.timestamp = frame.second;
   _raw.valid = true;

   return true;
}

bool NatNet_impl::getLocation( Vector3f& Vout){
    if (!isValid()){
        return false;
    }

    if (_raw.data->rigidBodies().empty()){
        return false;
    }

    Point3f locPoint = _raw.data->rigidBodies().front().location();
    Vout(locPoint.x, locPoint.y, locPoint.z);

    return true;
}

void GetEulers_hanoch(float qx, float qy, float qz, float qw,
        float* radYaw, float* radPitch, float* radRoll)
{
    //float &heading = *angle1;
    //float &attitude = *angle2;
    //float &bank = *angle3;

    //float test = -qx * qy * qz * qw;
    //float radYaw, radPitch, radRoll;


    //radYaw = (float) atan2(2.0f * (qy * qw - qx * qz),1.0f - 2.0f * (qy * qy - qz * qz));
    //radPitch = (float) asin(2.0f * qx * qy + 2.0f * qz * qw);
    //radRoll = (float) atan2(2.0f * qx * qw - 2.0f * qy * qz, 1.0f - 2.0f * qx * qx - 2.0f * qz * qz);
    *radYaw = (float) atan2(2.0f * (qy * qw - qx * qz),1.0f - 2.0f * (qy * qy - qz * qz));
    *radPitch = (float) asin(2.0f * qx * qy + 2.0f * qz * qw);
    *radRoll = (float) atan2(2.0f * qx * qw - 2.0f * qy * qz, 1.0f - 2.0f * qx * qx - 2.0f * qz * qz);

    //heading = 57*(radYaw);
    //attitude = 57*(radPitch);
    //bank = 57*(radRoll);
}

bool NatNet_impl::getOrientation( Quaternion& Qout){
    if (!isValid()){
        return false;
    }

    if (_raw.data->rigidBodies().empty()){
        return false;
    }

    Quaternion4f oriNatnet = _raw.data->rigidBodies().front().orientation();

    float pitch, roll, yaw;
    GetEulers_hanoch(oriNatnet.qx, oriNatnet.qy, oriNatnet.qz, oriNatnet.qw,
            &yaw, &pitch, &roll);

    Qout.from_euler(roll, pitch, yaw);

    return true;
}
bool NatNet_impl::getOrientation(float* pitch, float* roll, float* yaw){
    if (!isValid()){
        return false;
    }

    if (_raw.data->rigidBodies().empty()){
        return false;
    }

    float dummi;
    if (NULL == pitch){
        pitch = &dummi;
    }
    if (NULL == roll){
        roll = &dummi;
    }
    if (NULL == yaw){
        yaw = &dummi;
    }

    Quaternion4f oriNatnet = _raw.data->rigidBodies().front().orientation();

    GetEulers_hanoch(oriNatnet.qx, oriNatnet.qy, oriNatnet.qz, oriNatnet.qw,
            yaw, pitch, roll);

    return true;
}

bool NatNet_impl::isValid() {
    return _runing && _raw.valid;
}


bool NatNet_impl::init(const char* serverAddress, const char* localAddress){


    uint32_t server_inetAddr = inet_addr(serverAddress);
    struct sockaddr_in serverCommands; // Use this socket address to send commands to the server.
    serverCommands = NatNet::createAddress(server_inetAddr, NatNet::commandPort);

    // Create sockets
    _sdCommand = NatNet::createCommandSocket( inet_addr(localAddress) );
    _sdData = NatNet::createDataSocket( inet_addr(localAddress) );

   // Start the CommandListener in a new thread.
   _commandListener = new CommandListener(_sdCommand);
   _commandListener->start();

   // Send a ping packet to the server so that it sends us the NatNet version
   // in its response to commandListener.
   NatNetPacket ping = NatNetPacket::pingPacket();
   ping.send(_sdCommand, serverCommands);

   // Wait here for ping response to give us the NatNet version.
   _commandListener->getNatNetVersion(_natNetMajor, _natNetMinor);

   // Start up a FrameListener in a new thread.
   _frameListener = new FrameListener(_sdData, _natNetMajor, _natNetMinor);
   _frameListener->start();

   _runing = true;
   return true;
}

void NatNet_impl::terminate()
{

   _runing = false;

   // Wait for threads to finish.
   _frameListener->stop();
   _commandListener->stop();
   _frameListener->join();
   _commandListener->join();

   // Epilogue
   close(_sdData);
   close(_sdCommand);
}
