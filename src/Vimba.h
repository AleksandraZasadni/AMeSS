#ifndef ROB4_VIMBA_H
#define ROB4_VIMBA_H

#include "VimbaCPP/Include/VimbaCPP.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define CAMERA_LOAD_SETTINGS_DELAY 1000 //ms
#define CAMERA_ACQUISITION_TIMEOUT 300 //ms
#define CAMERA_ACQUISITION_ATTEMPTS 5
#define CAMERA_REACQUISITION_DELAY 100 //ms

class Vimba {
public:
    Vimba();

    ~Vimba();

    uint8_t init();

    void shutdown();

    void loadSettings(const std::string &path, const uint8_t &cameraIndex = 0);

    cv::Mat acquireImage(const uint8_t &cameraIndex = 0);

private:
    AVT::VmbAPI::VimbaSystem &vimbaSys = AVT::VmbAPI::VimbaSystem::GetInstance();
    AVT::VmbAPI::CameraPtrVector cameras;
};

extern Vimba mako;

#endif //ROB4_VIMBA_H