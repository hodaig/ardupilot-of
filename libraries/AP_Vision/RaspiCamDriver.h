/*
 * RaspiCamDriver.h
 *
 *  Created on: Mar 21, 2017
 *      Author: hodai
 */

#ifndef RASPICAMDRIVER_H_
#define RASPICAMDRIVER_H_

#define RPI_CV

#ifdef RPI_CV
#  include <opencv2/core/core.hpp>
#endif

namespace raspicam {
class RaspiCam;
class RaspiCam_Cv;
}
class RaspiCamDriver {
private:
#ifdef RPI_CV
    raspicam::RaspiCam_Cv* _cam;
    cv::Mat _frame;
#else
    raspicam::RaspiCam* _cam;
    unsigned char* _frame;
#endif
    volatile bool _isCapturing;
    volatile bool _hasNewFrame;

    volatile bool _frameIsUsed;

public:
    RaspiCamDriver();
    virtual ~RaspiCamDriver();

    bool startCapture(int fps=90, int h=480, int w=640);

#ifdef RPI_CV
    bool retrive(cv::Mat& dest, bool chackForNewFrame = true);
#else
    bool retrive(unsigned char* dest, bool chackForNewFrame = true);
#endif

    bool setResolution(int height, int width);
    size_t getFrameSize();

private:

    bool prepareInVid(raspicam::RaspiCam_Cv* Camera);

    static void userCallback(void* arg);

};

#endif /* RASPICAMDRIVER_H_ */
