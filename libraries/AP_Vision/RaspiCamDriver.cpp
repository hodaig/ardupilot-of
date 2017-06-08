/*
 * RaspiCamDriver.cpp
 *
 *  Created on: Mar 21, 2017
 *      Author: hodai
 */

#include "RaspiCamDriver.h"
#include <stdio.h> // printf
#include <unistd.h> // usleep

#ifdef RPI_CV
#  include <raspicam/raspicam_cv.h>
#else
#  include <raspicam/raspicam.h>
#endif
#include <string.h> // memcpy()

using namespace raspicam;

static RaspiCamDriver* _this = 0;

RaspiCamDriver::RaspiCamDriver() :
#ifdef RPI_CV
                    _cam(new raspicam::RaspiCam_Cv()),
                    _frame(),
#else
                    _cam(new raspicam::RaspiCam()),
                    _frame(0),
#endif
                    _isCapturing(false),
                    _hasNewFrame(false),
                    _frameIsUsed(false){
    _this = this;
}

RaspiCamDriver::~RaspiCamDriver() {
    // TODO Auto-generated destructor stub
}

#if 0
#ifdef RPI_CV

bool RaspiCamDriver::prepareInVid(raspicam::RaspiCam_Cv* Camera){
    //set camera params
    Camera->set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    Camera->set( CV_CAP_PROP_BRIGHTNESS, 50 );
    Camera->set( CV_CAP_PROP_EXPOSURE, 2 );
    //Camera->set( CV_CAP_PROP_FRAME_HEIGHT, 240 ); // 240 480
    //Camera->set( CV_CAP_PROP_FRAME_WIDTH, 320 );  // 320 640
    //Open camera
    printf("Opening Camera...\n");
    if (!Camera->open()) {
        printf("Error opening the camera\n");
        return false;
    }

    //wait a while until camera stabilizes
    //cout<<"Sleeping for 3 secs"<<endl;
    //usleep(3*1000000);

    return true;
}
#else
bool RaspiCamDriver::prepareInVid(raspicam::RaspiCam* Camera){
    //set camera params
    Camera->setFormat( RASPICAM_FORMAT_GRAY );
    Camera->setHeight( 240 );
    Camera->setWidth( 320 );
    //Open camera
    printf("Opening Camera...\n");
    if (!Camera->open( false )) {
        printf("Error opening the camera\n");
        return false;
    }

    //wait a while until camera stabilizes
    //cout<<"Sleeping for 3 secs"<<endl;
    //usleep(3*1000000);

    if ( !_cam->startCapture()){
        _cam->setUserCallback(0);
        _cam->release();
        delete _cam;
        _cam = 0;
        return false;
    }

    return true;
}
#endif
#endif // 0

bool RaspiCamDriver::startCapture(int fps, int h, int w){
    if (_isCapturing || !_cam){
        return false;
    }

    //set camera params
    _cam->set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    _cam->set( CV_CAP_PROP_BRIGHTNESS, 50 );
    _cam->set( CV_CAP_PROP_EXPOSURE, 2 );
    _cam->set( CV_CAP_PROP_FPS, fps );
    _cam->set( CV_CAP_PROP_FRAME_HEIGHT, h ); // 240 480
    _cam->set( CV_CAP_PROP_FRAME_WIDTH, w );  // 320 640

    printf("RaspiCamDriver - Opening Camera...\n");
    if (!_cam->open()) {
        printf("RaspiCamDriver - Error opening the camera\n");
        return false;
    }

    _hasNewFrame = false;
    _cam->setUserCallback(userCallback);

    //wait a while until camera stabilizes
    //cout<<"Sleeping for 3 secs"<<endl;
    //usleep(3*1000000);

    _isCapturing = true;

    return true;
}
#ifdef RPI_CV
bool RaspiCamDriver::retrive(cv::Mat& dest ,bool chackForNewFrame){
#else
bool RaspiCamDriver::retrive(unsigned char* dest ,bool chackForNewFrame){
#endif
    if ( chackForNewFrame && !_hasNewFrame){
        return false;
    }
    if (_frameIsUsed) {
        return false;
    } else {

        _frameIsUsed = true; // lock
#ifdef RPI_CV
        //dest = _frame.clone();
        // manual copy is faster
        dest.create ( _frame.rows, _frame.cols, CV_8UC1);
        memcpy(dest.data, _frame.data, dest.cols*dest.rows);
#else
        memcpy ( _frame, dest ,getFrameSize() );
#endif
        _hasNewFrame = false;
        _frameIsUsed = false; // release

        return true;
    }
}

static volatile bool inCallback = false;
void RaspiCamDriver::userCallback() {
    inCallback = true;

    if (!_this || !_this->_isCapturing ){
        inCallback = false;
        return;
    }

    for(int i=0 ; i<50 && _this->_frameIsUsed ; i++){
        // Wait
        usleep(10);
    }
    if ( !_this->_frameIsUsed ) {
        _this->_frameIsUsed = true;
    } else {
        inCallback = false;
        return;
    }
#ifdef RPI_CV
    _this->_cam->retrieve(_this->_frame);
#else
    _this->_cam->retrieve(_this->_frame, RASPICAM_FORMAT_IGNORE);
#endif

    _this->_hasNewFrame = true;
    _this->_frameIsUsed = false;
    inCallback = false;
}

void RaspiCamDriver::setResolution(int height, int width){
    printf("RaspiCamDriver::setResolution start\n");

    // TODO - bad mutex
    bool isCapturing = _isCapturing;
    _isCapturing = false;
    while (inCallback){
        // Wait
        usleep(10);
    }

    _cam->set( CV_CAP_PROP_FRAME_HEIGHT, height ); // 240 480
    _cam->set( CV_CAP_PROP_FRAME_WIDTH, width );  // 320 640

    if (!_cam->open()) {
        printf("RaspiCamDriver - Error opening the camera\n");
        return false;
    }
    _isCapturing = isCapturing;

    printf("RaspiCamDriver::setResolution end\n");
}

size_t RaspiCamDriver::getFrameSize(){
#ifdef RPI_CV
    return _frame.size().area();
#else
    return _cam->getWidth() * _cam->getHeight();
#endif
}
