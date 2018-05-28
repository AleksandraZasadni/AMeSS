#include "Vimba.h"

Vimba::Vimba() = default;

Vimba::~Vimba() = default;


uint8_t Vimba::init() {
    if (vimbaSys.Startup() != VmbErrorSuccess || vimbaSys.GetCameras(cameras) != VmbErrorSuccess) {
        std::cout << "ERROR: Vimba start-up FAILED!\n";
        return 0;
    }

    uint8_t connectedCameraCounter = 0;
    for (auto &camera : cameras) {
        if (camera->Open(VmbAccessModeFull) != VmbErrorSuccess) {
            std::cout << "ERROR: Camera_" << static_cast<uint16_t>(connectedCameraCounter) << " FAILED to open!\n";
        } else {
            std::string cameraName;
            if (camera->Camera::GetName(cameraName) == VmbErrorSuccess) {
                std::cout << "Info: Camera_" << static_cast<uint16_t>(connectedCameraCounter) << " \"" << cameraName
                          << "\" opened.\n";
            } else {
                cameraName = "UNKNOWN CAMERA NAME";
                std::cout << "WARNING: Could NOT detect name of Camera_"
                          << static_cast<uint16_t>(connectedCameraCounter) << "!\n";
            }
            AVT::VmbAPI::FramePtr cameraFrame;
            if (camera->Camera::AcquireSingleImage(cameraFrame, CAMERA_ACQUISITION_TIMEOUT) != VmbErrorSuccess) {
                std::cout << "ERROR: FAILED to acquire the first frame from Camera_"
                          << static_cast<uint16_t>(connectedCameraCounter) << "!\n";
            } else {
                connectedCameraCounter++;
            }
        }
    }
    return connectedCameraCounter;
}


void Vimba::shutdown() {
    for (auto &camera : cameras) {
        camera->Close();
    }
    vimbaSys.Shutdown();
}


void Vimba::loadSettings(const std::string &path, const uint8_t &cameraIndex) {
    if (cameras[cameraIndex]->Camera::LoadCameraSettings(path) == VmbErrorSuccess) {
        std::cout << "Info: Settings for Camera_" << static_cast<uint16_t>(cameraIndex) << " from file \""
                  << path << "\" loaded.\n";
    } else {
        std::cout << "WARNING: FAILED to load settings for Camera_" << static_cast<uint16_t>(cameraIndex)
                  << " from file \""
                  << path << "\"!\n";
    }
    cv::waitKey(CAMERA_LOAD_SETTINGS_DELAY);
}


cv::Mat Vimba::acquireImage(const uint8_t &cameraIndex) {
    AVT::VmbAPI::FramePtr cameraFrame;
    for (int i = 0; i < CAMERA_ACQUISITION_ATTEMPTS; ++i) {
        if (cameras[cameraIndex]->Camera::AcquireSingleImage(cameraFrame, CAMERA_ACQUISITION_TIMEOUT) ==
            VmbErrorSuccess) {
            VmbFrameStatusType frameStatus;
            if (cameraFrame->Frame::GetReceiveStatus(frameStatus) == VmbErrorSuccess) {
                VmbUchar_t *cameraImage = nullptr;
                if (cameraFrame->GetImage(cameraImage) == VmbErrorSuccess) {
                    VmbUint32_t frameHeight, frameWidth;
                    if (cameraFrame->GetHeight(frameHeight) != VmbErrorSuccess) {
                        std::cout << "ERROR: FAILED to acquire height from Camera_"
                                  << static_cast<uint16_t>(cameraIndex)
                                  << "!\n";
                        cv::waitKey(CAMERA_REACQUISITION_DELAY);
                        continue;
                    }
                    if (cameraFrame->GetWidth(frameWidth) != VmbErrorSuccess) {
                        std::cout << "ERROR: FAILED to acquire width from Camera_" << static_cast<uint16_t>(cameraIndex)
                                  << "!\n";
                        cv::waitKey(CAMERA_REACQUISITION_DELAY);
                        continue;
                    }
                    return cv::Mat(frameHeight, frameWidth, CV_8UC1, cameraImage);
                } else {
                    std::cout << "ERROR: FAILED to acquire image data of frame from Camera_"
                              << static_cast<uint16_t>(cameraIndex) << "!\n";
                }
            } else {
                std::cout << "ERROR: Frame from Camera_" << static_cast<uint16_t>(cameraIndex)
                          << " NOT received properly!\n";
            }
        } else {
            std::cout << "ERROR: FAILED to acquire frame from Camera_" << static_cast<uint16_t>(cameraIndex) << "!\n";
        }
        cv::waitKey(CAMERA_REACQUISITION_DELAY);
    }
    return cv::Mat(0, 0, CV_8U); //Image could not be acquired, returning Mat of size (0, 0)
}


Vimba mako; // NOLINT