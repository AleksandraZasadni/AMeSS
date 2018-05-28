///Flags
//#define __UR_CONNECTED //CAREFUL, UR might move if defined!
//#define __CALIBRATION
//#define __LASER_DEBUG
//#define __CALIBRATION_PROCEDURE_DEBUG

#define HOMOGRAPHY_11 (-0.0343751880449925)
#define HOMOGRAPHY_12 0.9795642828439
#define HOMOGRAPHY_13 (-34.33328579248314)
#define HOMOGRAPHY_21 0.8623221913633431
#define HOMOGRAPHY_22 0.089846664820702
#define HOMOGRAPHY_23 (-164.9783349472563)
#define HOMOGRAPHY_31 (-2.189529268557359e-06)
#define HOMOGRAPHY_32 7.808275702727731e-05
#define HOMOGRAPHY_33 1

///Dependancies
//Classes
#include "src/Vimba.h"
#include "src/URSocket.h"
//C++ libraries
#include <iostream>
#include <cmath>
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


///Namespaces
using namespace cv;
using namespace std;

///Global Variables
//"UR" from URSocket
//"mako" from Vimba
int16_t imageCropOffsetX = 0;

///Methods
void getProjectedImage(const Mat &homography, Mat &output);
#define REACQUIRE_IMAGE_DELAY 5000 // us
#define CROP_OFFSET (-75) //px

inline void show(const std::string &name, const Mat &input);
#define CAMERA_DEFAULT_SETTINGS_PATH "/home/jesper/Desktop/rob4/src/cameraOperatingSettings.xml"

void calibration(const Mat &referenceImageGreen);
#define CAMERA_CALIBRATION_SETTINGS_PATH "/home/jesper/Desktop/rob4/src/cameraCalibrationSettings.xml"
#define CALIBRATION_BEGINNING_X     (-0.27) //m
#define CALIBRATION_BEGINNING_Y     (+0.14) //m
#define CALIBRATION_END_X           (-0.46) //m
#define CALIBRATION_END_Y           (-0.15) //m
#define CALIBRATION_Z               (+0.25) //m
#define CALIBRATION_STEPS_X         20
#define CALIBRATION_STEPS_Y         30
#define CALIBRATION_STEP_TRAVEL_TIME_MULTIPLIER 30 //s per m
#define PROJECTION_PIXELS_PER_METRE 5000.0 //px per m
#define PROJECTION_RESOLUTION_X (abs(CALIBRATION_END_X - CALIBRATION_BEGINNING_X) * PROJECTION_PIXELS_PER_METRE)
#define PROJECTION_RESOLUTION_Y (abs(CALIBRATION_END_Y - CALIBRATION_BEGINNING_Y) * PROJECTION_PIXELS_PER_METRE)

Point2f getLaserCoordinates(const Mat &referenceImageGreen);
#define LASER_THRESHOLD 128
#define MEDIAN_FILTER_KERNEL_SIZE 5

void mapMotion(const Point2f &imageCoordinates, const float &Z, const float &t);
#define WORKSPACE_SURFACE_Z (+0.12) //m
#define DEFAULT_UR_X (CALIBRATION_BEGINNING_X) //m
#define DEFAULT_UR_Y (CALIBRATION_BEGINNING_Y) //m
#define WORKSPACE_OFFSET_Z (+0.13) //m
#define DEFAULT_UR_Z (WORKSPACE_SURFACE_Z + WORKSPACE_OFFSET_Z) //m
#define DEFAULT_MOVE_TRAVEL_TIME 5 //s
#define MOVEMENT_SLOPE_X ((CALIBRATION_END_X - CALIBRATION_BEGINNING_X) / PROJECTION_RESOLUTION_X)
#define MOVEMENT_SLOPE_Y ((CALIBRATION_END_Y - CALIBRATION_BEGINNING_Y) / PROJECTION_RESOLUTION_Y)

inline void moveAbovePixel(const Point2f &planarCoordinates);
#define MOVE_ABOUT_PIXEL_TRAVEL_TIME_MULTIPLIER 7.5 //s per m
#define MOVE_ABOUT_PIXEL_WAIT_MULTIPLIER 1025000 //1000000 = 100% of the motion time

void pick(const Point2f &planarCoordinates);
#define MINIMUM_DISTANCE 200
#define MAXIMUM_CORRECTION_DISTANCE 200 //px
#define PICKING_TRAVEL_TIME_MULTIPLIER 5 //s per m
#define PICKING_TRAVEL_WAIT_MULTIPLIER 1010000 //1000000 = 100% of the motion time
#define PICKING_WAIT_DELAY 100000 //us

void placeInBin(const uint8_t &category);
#define NOZZLE_LIMIT 5
#define BIN_MOTION_TRAVEL_TIME_MULTIPLIER 7.5 //s per m
#define BIN_MOTION_TRAVEL_TIME_WAIT_MULTIPLIER 1010000 //1000000 = 100% of the motion time
#define BIN_WAIT 2000000 //us
#define __SHAKE_OFF
#define SHAKE_OFF_DISTANCE (-0.05) //m
#define SHAKE_OFF_TRAVEL_TIME_DOWN 0.25 //s
#define SHAKE_OFF_INTERRUPT_IN 165000 //us
#define SHAKE_DECELERATION_TIME 100000 //us
#define SHAKE_OFF_TRAVEL_TIME_UP 0.4 //s

#define BIN_Z           (+0.175) //m
#define LARVAE_X        (-0.250) //m
#define LARVAE_Y        (+0.230) //m
#define PUPAE_X         (-0.185) //m
#define PUPAE_Y         (+0.230) //m
#define BEETLES_X       (-0.315) //m
#define BEETLES_Y       (+0.230) //m
#define DEAD_X          (-0.280) //m
#define DEAD_Y          (+0.285) //m
#define UNIDENTIFIED_X  (-0.215) //m
#define UNIDENTIFIED_Y  (+0.285) //m

void segmentation(const Mat &input, Mat &output);
#define SEGMENTATION_THRESHOLD 225

vector<vector<Point2f>> classifier(const Mat &input);
#define DEPTH_OF_DEFECT 800
#define AREA_LOWER_LIMIT 700
#define AREA_UPPER_LIMIT_CLUSTER 4000
#define AREA_UPPER_LIMIT_ALL 7500
#define RESULT_RATIO_LIMIT 100

#define NUMBER_OF_CLASSES 6
#define NUMBER_OF_CATEGORIES 5
#define BEETLE 0
#define LARVAE 1
#define PUPA 2
#define DEAD 3
#define UNCLASSIFIED 4

#define NUMBER_OF_FEATURES 11
#define FEATURE_AREA 0
#define FEATURE_PERIMETER 1
#define FEATURE_WIDTH 2
#define FEATURE_HEIGHT 3
#define FEATURE_CIRCULARITY 4
#define FEATURE_ASPECT_RATIO 5
#define FEATURE_ROUND 6
#define FEATURE_SOLIDITY 7
#define FEATURE_DEFECTS 8
#define FEATURE_ELONGATION 9
#define FEATURE_INTENSITY 10

#define BEE_PRIOR 0.63
#define MW_PRIOR 0.1125
#define PUPA_PRIOR 0.1275
#define DBE_PRIOR 0.07
#define DMW_PRIOR 0.0375
#define DPU_PRIOR 0.0225

struct blobStruct {
    vector<Point> contour;
    float feature[NUMBER_OF_FEATURES];
    double gaussResult[NUMBER_OF_CLASSES] = {BEE_PRIOR, MW_PRIOR, PUPA_PRIOR, DBE_PRIOR, DMW_PRIOR, DPU_PRIOR};
    Point2f centreOfMass;
    Rect bounding_Rect;
    uint8_t result, resultSecond;
};

const double featuresMean[NUMBER_OF_CLASSES][NUMBER_OF_FEATURES] = {{1926.0109739369, 377.2918230453, 72.670781893,  71.7064471879, 0.1795508908, 1.7030454321, 0.4104875665, 0.5672829369, 6.9286694102, 4.4719853909,  144.1626337449},
                                                             {1792.355495251,  272.3863704206, 82.2035278155, 74.5685210312, 0.3061818833, 3.7208097286, 0.2199700328, 0.7025708318, 1.1818181818, 23.2560776934, 178.4325223881},
                                                             {1292.9002016129, 180.2025221774, 53.0282258065, 50.7258064516, 0.5030809113, 2.1533389113, 0.3801946593, 0.8227250746, 0.8951612903, 5.52487875,    201.1278729839},
                                                             {1543.5080321285, 186.1658574297, 52.4698795181, 52.7670682731, 0.5707170663, 2.2755375904, 0.4223849297, 0.9082419116, 1.4277108434, 5.2987523896,  115.4579092369},
                                                             {1135.1981327801, 199.6928589212, 56.5746887967, 62.9439834025, 0.3616604232, 4.3407971784, 0.1907029133, 0.8043849585, 0.6556016598, 23.384143361,  136.9529547718},
                                                             {1146.2873051225, 159.7379799555, 46.5679287305, 46.3118040089, 0.5686636771, 2.133281069,  0.4244101225, 0.8748659755, 0.7728285078, 4.9369283519,  141.3509420935}};
const double featuresSigma[NUMBER_OF_CLASSES][NUMBER_OF_FEATURES] = {{82338.1241925942, 4087.4403158409, 149.7183896351, 148.834038047,  0.0027110903, 0.0616299384, 0.00197083,   0.0040102344, 1.3108390238, 0.8546773156,   55.3615961473},
                                                             {130804.717201419, 1791.3469568803, 790.107976668,  866.7021599021, 0.0013580213, 4.3860464171, 0.0098121308, 0.0135663719, 0.3391798419, 239.7181669844, 123.2241244754},
                                                             {30151.8460807674, 233.8157701732,  153.7204138156, 151.9893124796, 0.0036615345, 0.2277002518, 0.003876811,  0.0049076453, 0.1586836103, 2.0926967088,   113.2980911414},
                                                             {39019.318848836,  538.3513908911,  145.6218273496, 133.1729978263, 0.0074641793, 0.0906967777, 0.002895332,  0.0030216199, 1.581282878,  0.8061677188,   103.4807405277},
                                                             {44422.5381046143, 560.1070701755,  452.1409796327, 475.2255458459, 0.0034429198, 1.9675553751, 0.0014071879, 0.0182116947, 0.3884196996, 57.1197130769,  179.8795411817},
                                                             {47030.6482983018, 371.1790452563,  120.4066477092, 141.9829183901, 0.0067557296, 0.2028734016, 0.0064105337, 0.0054931704, 0.2964922049, 2.3672300482,   233.0216895011}};

struct sortingOperatorAscendingPointX {
    bool operator()(Point2f point1, Point2f point2) {return (point1.x < point2.x); }
};



////////////
/// Main ///
////////////
int main() {
    //Check for workspace dimensions
    if (fmodf(static_cast<float>(PROJECTION_RESOLUTION_Y), 1.0) != 0.0 ||
        fmodf(static_cast<float>(PROJECTION_RESOLUTION_X), 1.0) != 0.0) {
        std::cout << "ERROR: The determined resolution of the projected image is a floating-point number ("
                  << static_cast<float>(PROJECTION_RESOLUTION_Y) << " x " << static_cast<float>(PROJECTION_RESOLUTION_X)
                  << ")!\n";
        return 0;
    }

    //Establish communication with camera
    if (mako.init() == 0) {
        std::cout << "ERROR: NO cameras were initialised!\n";
        mako.shutdown();
        UR.shutdown();
        return 0;
    }

    //Load settings
#if defined(__CALIBRATION) || defined(__LASER_DEBUG)
    mako.loadSettings(CAMERA_CALIBRATION_SETTINGS_PATH);
#else
    mako.loadSettings(CAMERA_DEFAULT_SETTINGS_PATH);
#endif

    //Acquire initial image
    Mat originalInitImageBayerRG = mako.acquireImage();
    if (originalInitImageBayerRG.rows == 0 || originalInitImageBayerRG.cols == 0) {
        std::cout << "ERROR: Initial image cannot be acquired properly!\n";
        mako.shutdown();
        UR.shutdown();
        return 0;
    }
    static Mat originalInitImage = Mat(originalInitImageBayerRG.cols, originalInitImageBayerRG.rows, CV_8UC3);
    cvtColor(originalInitImageBayerRG, originalInitImage, CV_BayerRG2RGB);

    //Establish communication with UR
#ifdef __UR_CONNECTED
    if (!UR.init()) {
        std::cout << "ERROR: UR is NOT responding!\n";
        mako.shutdown();
        UR.shutdown();
        return 0;
    }
#ifdef __CALIBRATION
    UR.moveJWait(static_cast<const float &>(DEFAULT_UR_X), static_cast<const float &>(DEFAULT_UR_Y),
                 static_cast<const float &>(CALIBRATION_Z), DEFAULT_MOVE_TRAVEL_TIME);
#else
    UR.moveJWait(static_cast<const float &>(DEFAULT_UR_X), static_cast<const float &>(DEFAULT_UR_Y),
                 static_cast<const float &>(DEFAULT_UR_Z), DEFAULT_MOVE_TRAVEL_TIME);
#endif

#endif

#ifdef __CALIBRATION
#ifndef __UR_CONNECTED
    std::cout << "ERROR: __UR_CONNECTED is NOT defined!\n";
    mako.shutdown();
    UR.shutdown();
    return 0;
#endif
    if (CALIBRATION_STEPS_X < 2 || CALIBRATION_STEPS_Y < 2) {
        std::cout << "ERROR: Not enough steps defined for the calibration procedure!\n";
        return 0;
    }
    Mat referenceImageGreen;
    extractChannel(originalInitImage, referenceImageGreen, 1);
    calibration(referenceImageGreen);
    mako.shutdown();
    UR.shutdown();
    return 0;
#endif

#ifdef __LASER_DEBUG
    while (true) {
        Mat referenceImageGreen;
        extractChannel(originalInitImage, referenceImageGreen, 1);
        Point2f presentLaserCoordinates = getLaserCoordinates(referenceImageGreen);
        std::cout << "Info: Laser found at " << presentLaserCoordinates << "\n";
        if (static_cast<char>(waitKey(25)) == static_cast<char>(113)) {
            mako.shutdown();
            UR.shutdown();
            return 0;
        }
    }
#endif

    float tempHomography[9]{static_cast<float>(HOMOGRAPHY_11), static_cast<float>(HOMOGRAPHY_12),
                            static_cast<float>(HOMOGRAPHY_13),
                            static_cast<float>(HOMOGRAPHY_21), static_cast<float>(HOMOGRAPHY_22),
                            static_cast<float>(HOMOGRAPHY_23),
                            static_cast<float>(HOMOGRAPHY_31), static_cast<float>(HOMOGRAPHY_32),
                            static_cast<float>(HOMOGRAPHY_33)};
    Mat perspectiveCorrectionHomography = Mat(3, 3, CV_32F, tempHomography);

    static Mat projectedImage;
    getProjectedImage(perspectiveCorrectionHomography, projectedImage);
    while (true) {
        bool pickedSomething = false;
        getProjectedImage(perspectiveCorrectionHomography, projectedImage);
        if (projectedImage.rows == 0 || projectedImage.cols == 0) {
            continue;
        }

        vector<vector<Point2f>> molitor = classifier(projectedImage);
        uint8_t pickCounter = 0;
        for (int8_t i = 0; i < NUMBER_OF_CATEGORIES; ++i) {
            uint8_t pickIndex = 0;
            while (!molitor.at(static_cast<uint8_t>(i)).empty()) {
                bool skip = false;
                for (uint8_t j = 0; j < NUMBER_OF_CATEGORIES; ++j) {
                    if (i == j) {
                        continue;
                    }
                    for (uint8_t k = 0; k < molitor.at(j).size(); ++k) {
                        if (sqrt(pow(molitor.at(static_cast<uint8_t>(i)).at(pickIndex).x - molitor.at(j).at(k).x, 2) +
                                 pow(molitor.at(static_cast<uint8_t>(i)).at(pickIndex).y - molitor.at(j).at(k).y, 2)) < MINIMUM_DISTANCE) {
                            skip = true;
                            pickIndex++;
                            break;
                        }
                    }
                    if (skip) {
                        break;
                    }
                }
                if (skip) {
                    if (molitor.at(static_cast<uint8_t>(i)).size() == pickIndex) {
                        break;
                    }
                    continue;
                }

                moveAbovePixel(molitor.at(static_cast<uint8_t>(i)).at(pickIndex));

                imageCropOffsetX += molitor.at(static_cast<uint8_t>(i)).at(pickIndex).x + CROP_OFFSET;
                if (imageCropOffsetX < 0) {
                    imageCropOffsetX = 0;
                }
                Point2f previouslyOnAMeSS = molitor.at(static_cast<uint8_t>(i)).at(pickIndex);

                pickIndex = 0;
                getProjectedImage(perspectiveCorrectionHomography, projectedImage);
                while (projectedImage.rows == 0 || projectedImage.cols == 0) {
                    getProjectedImage(perspectiveCorrectionHomography, projectedImage);
                    usleep(REACQUIRE_IMAGE_DELAY);
                }
                molitor = classifier(projectedImage);

                if (!molitor.at(static_cast<uint8_t>(i)).empty()) {
                    auto minDistance = static_cast<float>(sqrt(pow(previouslyOnAMeSS.x - molitor.at(
                            static_cast<uint8_t>(i)).at(0).x, 2) + pow(previouslyOnAMeSS.y - molitor.at(
                            static_cast<uint8_t>(i)).at(0).y, 2)));
                    auto updatedPoint = Point2f(molitor.at(static_cast<uint8_t>(i)).at(0));
                    if (molitor.at(static_cast<uint8_t>(i)).size() > 1) {
                        for (uint8_t j = 1; j < molitor.at(static_cast<uint8_t>(i)).size(); ++j) {
                            auto temp = static_cast<float>(sqrt(pow(previouslyOnAMeSS.x - molitor.at(
                                    static_cast<uint8_t>(i)).at(j).x, 2) + pow(previouslyOnAMeSS.y - molitor.at(
                                    static_cast<uint8_t>(i)).at(j).y, 2)));
                            if (temp < minDistance) {
                                minDistance = temp;
                                updatedPoint = Point2f(molitor.at(static_cast<uint8_t>(i)).at(j));
                            }
                        }
                    }
                    if (minDistance < MAXIMUM_CORRECTION_DISTANCE) {
                        pick(updatedPoint);
                        pickCounter++;
                        moveAbovePixel(updatedPoint);

                        if (pickCounter == NOZZLE_LIMIT) {
                            break;
                        }
                    }
                    getProjectedImage(perspectiveCorrectionHomography, projectedImage);
                    while (projectedImage.rows == 0 || projectedImage.cols == 0) {
                        getProjectedImage(perspectiveCorrectionHomography, projectedImage);
                        usleep(REACQUIRE_IMAGE_DELAY);
                    }
                    molitor = classifier(projectedImage);
                }
            }
            if (pickCounter > 0) {
                placeInBin(static_cast<const uint8_t &>(i));
                pickedSomething = true;
                i = -1;
                pickCounter = 0;
                imageCropOffsetX = 0;
            } else {
                imageCropOffsetX = 0;
                moveAbovePixel(Point2f(0, 0));
            }
            getProjectedImage(perspectiveCorrectionHomography, projectedImage);
            while (projectedImage.rows == 0 || projectedImage.cols == 0) {
                getProjectedImage(perspectiveCorrectionHomography, projectedImage);
                usleep(REACQUIRE_IMAGE_DELAY);
            }
            molitor = classifier(projectedImage);
        }

        if (!pickedSomething) {
            for (int8_t i = (NUMBER_OF_CATEGORIES - 1); i > -1; --i) {
                if (!molitor.at(static_cast<uint8_t >(i)).empty()) {
                    moveAbovePixel(molitor.at(static_cast<uint8_t >(i)).at(0));
                    pick(molitor.at(static_cast<uint8_t >(i)).at(0));
                    moveAbovePixel(molitor.at(static_cast<uint8_t >(i)).at(0));
                    placeInBin(UNCLASSIFIED);
                    getProjectedImage(perspectiveCorrectionHomography, projectedImage);
                    while (projectedImage.rows == 0 || projectedImage.cols == 0) {
                        getProjectedImage(perspectiveCorrectionHomography, projectedImage);
                        usleep(REACQUIRE_IMAGE_DELAY);
                    }
                    molitor = classifier(projectedImage);
                    break;
                }
            }
        }

        if (static_cast<char>(waitKey(1)) == static_cast<char>(113)) {
            break;
        }
    }

    mako.shutdown();
    UR.shutdown();

    return 0;
}



///////////////
/// Methods ///
///////////////

void getProjectedImage(const Mat &homography, Mat &output) {
    Mat originalImageBayerRG = mako.acquireImage();
    if (originalImageBayerRG.rows == 0 || originalImageBayerRG.cols == 0) {
        output = Mat(0, 0, CV_8U);
        return;
    }

    static Mat originalImage = Mat(originalImageBayerRG.cols, originalImageBayerRG.rows, CV_8UC3);
    cvtColor(originalImageBayerRG, originalImage, CV_BayerRG2RGB);

    static Mat croppedImage, projectedImage;
    warpPerspective(originalImage, projectedImage, homography,
                    Size(static_cast<int>(PROJECTION_RESOLUTION_X),
                         static_cast<int>(PROJECTION_RESOLUTION_Y)));


    projectedImage(Rect(imageCropOffsetX, 0, static_cast<int>(PROJECTION_RESOLUTION_X - imageCropOffsetX),
                        static_cast<int>(PROJECTION_RESOLUTION_Y))).copyTo(croppedImage);

    output = croppedImage;
}


inline void show(const std::string &name, const Mat &input) {
    namedWindow(name, CV_WINDOW_KEEPRATIO);
    moveWindow(name, 0, 0);
    resizeWindow(name, 1920, 1080 - 100);
    imshow(name, input);
    waitKey(5);
}

void calibration(const Mat &referenceImageGreen) {
    std::cout << "Info: Calibration Started!\n";

    Point2f stepSizeUR = Point2f(
            static_cast<float>((CALIBRATION_END_X - CALIBRATION_BEGINNING_X) / (CALIBRATION_STEPS_X - 1)),
            static_cast<float>((CALIBRATION_END_Y - CALIBRATION_BEGINNING_Y) / (CALIBRATION_STEPS_Y - 1)));
    Point2f stepSizeProjectedImage = Point2f(
            static_cast<float>(PROJECTION_RESOLUTION_X / float(CALIBRATION_STEPS_X - 1)),
            static_cast<float>(PROJECTION_RESOLUTION_Y / float(CALIBRATION_STEPS_Y - 1)));
    float stepSizeTime = CALIBRATION_STEP_TRAVEL_TIME_MULTIPLIER *
                         fabsf((stepSizeUR.x > stepSizeUR.y) ? stepSizeUR.x : stepSizeUR.y);

    std::vector<Point2f> coordinatesInOriginalImage, coordinatesInProjection;
    for (int stepY = 0; stepY < CALIBRATION_STEPS_Y; ++stepY) {
        for (int stepX = 0; stepX < CALIBRATION_STEPS_X; ++stepX) {
            UR.moveLWait(static_cast<const float &>(stepY % 2 == 0 ?
                                                    CALIBRATION_BEGINNING_X + stepX * stepSizeUR.x :
                                                    CALIBRATION_END_X - stepX * stepSizeUR.x),
                         static_cast<const float &>(CALIBRATION_BEGINNING_Y + stepY * stepSizeUR.y),
                         static_cast<const float &>(CALIBRATION_Z), stepSizeTime);
            Point2f presentLaserCoordinates = getLaserCoordinates(referenceImageGreen);
            if (presentLaserCoordinates.x != -1 && presentLaserCoordinates.y != -1) {
                coordinatesInOriginalImage.emplace_back(presentLaserCoordinates);
                coordinatesInProjection.emplace_back(stepY % 2 == 0 ? stepX * stepSizeProjectedImage.x :
                                                     PROJECTION_RESOLUTION_X - stepX * stepSizeProjectedImage.x,
                                                     stepY * stepSizeProjectedImage.y);
#ifdef __CALIBRATION_PROCEDURE_DEBUG
                std::cout << "Info: Step(X, Y) = (" << stepX << ", " << stepY << ") - UR "
                          << Point2f(static_cast<float>(stepY % 2 == 0 ?
                                                        CALIBRATION_BEGINNING_X + stepX * stepSizeUR.x :
                                                        CALIBRATION_END_X - stepX * stepSizeUR.x),
                                     static_cast<float>(CALIBRATION_BEGINNING_Y + stepY * stepSizeUR.y))
                          << ", Original " << presentLaserCoordinates
                          << ", Projected " << Point2f(
                        static_cast<float>(stepY % 2 == 0 ? stepX * stepSizeProjectedImage.x :
                                           PROJECTION_RESOLUTION_X - stepX * stepSizeProjectedImage.x),
                        stepY * stepSizeProjectedImage.y) << "\n";
#endif
            }
#ifdef __CALIBRATION_PROCEDURE_DEBUG
            else {
                std::cout << "Info: Step(X, Y) = (" << stepX << ", " << stepY << ") - Laser could NOT be detected\n";
            }
#endif
        }
    }

    std::cout << "Info: Calibration Finished: " << coordinatesInOriginalImage.size() << "/"
              << CALIBRATION_STEPS_X * CALIBRATION_STEPS_Y
              << " points detected!\n";
    if (coordinatesInOriginalImage.size() > CALIBRATION_STEPS_X * CALIBRATION_STEPS_Y / 2.0) {
        Mat foundHomography = findHomography(coordinatesInOriginalImage, coordinatesInProjection);
        std::cout << "Info: Calibration Successful! Insert the following values into the code!\nHomography Matrix:\n"
                  << foundHomography << "\n";
    } else {
        std::cout << "ERROR: Calibration Failed! Less than half of the points detected!\n";
    }
}

Point2f getLaserCoordinates(const Mat &referenceImageGreen) {
    Mat laserOriginalImageBayerRG = mako.acquireImage();
    if (laserOriginalImageBayerRG.rows == 0 || laserOriginalImageBayerRG.cols == 0) {
        return Point2f(-1, -1);
    }
    Mat laserOriginalImage = Mat(laserOriginalImageBayerRG.rows, laserOriginalImageBayerRG.cols, CV_8UC3);
    cvtColor(laserOriginalImageBayerRG, laserOriginalImage, CV_BayerRG2RGB);

    Mat laserOriginalImageGreen;
    extractChannel(laserOriginalImage, laserOriginalImageGreen, 1);

    Mat laserSubtractedImage, laserBinaryImage;
    subtract(laserOriginalImageGreen, referenceImageGreen, laserSubtractedImage);
    threshold(laserSubtractedImage, laserBinaryImage, LASER_THRESHOLD, 255, THRESH_BINARY);
    medianBlur(laserBinaryImage, laserBinaryImage, MEDIAN_FILTER_KERNEL_SIZE);

#ifdef __LASER_DEBUG
    show("Laser Subtracted", laserSubtractedImage);
    show("Laser Binary", laserBinaryImage);
    waitKey(5);
#endif

    Mat laserLabels, laserStats, laserCentroid;
    connectedComponentsWithStats(laserBinaryImage, laserLabels, laserStats, laserCentroid, 8, CV_16U);
    if (laserCentroid.rows != 2) {
        if (laserCentroid.rows > 2) {
            std::cout << "WARNING: More than one laser beam found!\n";
        } else {
            std::cout << "WARNING: No laser beam found!\n";
        }
        return Point2f(-1, -1);
    }

    return Point2f(static_cast<float>(laserCentroid.at<double>(1, 0)),
                   static_cast<float>(laserCentroid.at<double>(1, 1)));
}


inline void mapMotion(const Point2f &imageCoordinates, const float &Z, const float &t) {
    UR.moveL(static_cast<const float &>(CALIBRATION_BEGINNING_X + (imageCoordinates.x + imageCropOffsetX) * MOVEMENT_SLOPE_X),
             static_cast<const float &>(CALIBRATION_BEGINNING_Y + imageCoordinates.y * MOVEMENT_SLOPE_Y), Z, t);
}

inline void moveAbovePixel(const Point2f &planarCoordinates) {
    auto t = static_cast<float>(MOVE_ABOUT_PIXEL_TRAVEL_TIME_MULTIPLIER * sqrt(pow(CALIBRATION_BEGINNING_X + (planarCoordinates.x + imageCropOffsetX) * MOVEMENT_SLOPE_X - UR.getPresentX(), 2) +
                                                                               pow(CALIBRATION_BEGINNING_Y + planarCoordinates.y * MOVEMENT_SLOPE_Y - UR.getPresentY(), 2) +
                                                                               pow(DEFAULT_UR_Z - UR.getPresentZ(), 2)));
    mapMotion(Point2f(planarCoordinates.x, planarCoordinates.y), static_cast<const float &>(DEFAULT_UR_Z), t);
    usleep(static_cast<__useconds_t>(MOVE_ABOUT_PIXEL_WAIT_MULTIPLIER * t));
}

inline void pick(const Point2f &planarCoordinates) {
    auto t = static_cast<float>(PICKING_TRAVEL_TIME_MULTIPLIER * sqrt(pow(CALIBRATION_BEGINNING_X + (planarCoordinates.x + imageCropOffsetX) * MOVEMENT_SLOPE_X - UR.getPresentX(), 2) +
                                                                      pow(CALIBRATION_BEGINNING_Y + planarCoordinates.y * MOVEMENT_SLOPE_Y - UR.getPresentY(), 2) +
                                                                      pow(WORKSPACE_SURFACE_Z - UR.getPresentZ(), 2)));
    mapMotion(Point2f(planarCoordinates.x, planarCoordinates.y), static_cast<const float &>(WORKSPACE_SURFACE_Z), t);
    usleep(static_cast<__useconds_t>(PICKING_WAIT_DELAY + PICKING_TRAVEL_WAIT_MULTIPLIER * t));
}


inline void placeInBin(const uint8_t &category) {
    float t = 20;
    switch (category) {
        case UNCLASSIFIED:
            t = static_cast<float>(BIN_MOTION_TRAVEL_TIME_MULTIPLIER * sqrt(pow(UNIDENTIFIED_X - UR.getPresentX(), 2) +
                                                                            pow(UNIDENTIFIED_Y - UR.getPresentY(), 2) +
                                                                            pow(BIN_Z - UR.getPresentZ(), 2)));
            UR.moveL(static_cast<const float &>(UNIDENTIFIED_X), static_cast<const float &>(UNIDENTIFIED_Y), static_cast<const float &>(BIN_Z), t);
            break;
        case DEAD:
            t = static_cast<float>(BIN_MOTION_TRAVEL_TIME_MULTIPLIER * sqrt(pow(DEAD_X - UR.getPresentX(), 2) +
                                                                            pow(DEAD_Y - UR.getPresentY(), 2) +
                                                                            pow(BIN_Z - UR.getPresentZ(), 2)));
            UR.moveL(static_cast<const float &>(DEAD_X), static_cast<const float &>(DEAD_Y), static_cast<const float &>(BIN_Z), t);
            break;
        case LARVAE:
            t = static_cast<float>(BIN_MOTION_TRAVEL_TIME_MULTIPLIER * sqrt(pow(LARVAE_X - UR.getPresentX(), 2) +
                                                                            pow(LARVAE_Y - UR.getPresentY(), 2) +
                                                                            pow(BIN_Z - UR.getPresentZ(), 2)));
            UR.moveL(static_cast<const float &>(LARVAE_X), static_cast<const float &>(LARVAE_Y), static_cast<const float &>(BIN_Z), t);
            break;
        case PUPA:
            t = static_cast<float>(BIN_MOTION_TRAVEL_TIME_MULTIPLIER * sqrt(pow(PUPAE_X - UR.getPresentX(), 2) +
                                                                            pow(PUPAE_Y - UR.getPresentY(), 2) +
                                                                            pow(BIN_Z - UR.getPresentZ(), 2)));
            UR.moveL(static_cast<const float &>(PUPAE_X), static_cast<const float &>(PUPAE_Y), static_cast<const float &>(BIN_Z), t);
            break;
        case BEETLE:
            t = static_cast<float>(BIN_MOTION_TRAVEL_TIME_MULTIPLIER * sqrt(pow(BEETLES_X - UR.getPresentX(), 2) +
                                                                            pow(BEETLES_Y - UR.getPresentY(), 2) +
                                                                            pow(BIN_Z - UR.getPresentZ(), 2)));
            UR.moveL(static_cast<const float &>(BEETLES_X), static_cast<const float &>(BEETLES_Y), static_cast<const float &>(BIN_Z), t);
            break;
        default:
            break;
    }
    usleep(static_cast<__useconds_t>(BIN_MOTION_TRAVEL_TIME_WAIT_MULTIPLIER * t + BIN_WAIT));
#ifdef __SHAKE_OFF
    if (category == BEETLE) {
        UR.moveL(UR.getPresentX(), UR.getPresentY(), static_cast<const float &>(BIN_Z + SHAKE_OFF_DISTANCE),
                 SHAKE_OFF_TRAVEL_TIME_DOWN);
        usleep(SHAKE_OFF_INTERRUPT_IN);
        UR.command("stopl(10)"); //decelerate linearly with 10 m/s^2
        usleep(SHAKE_DECELERATION_TIME);
        UR.moveLWait(UR.getPresentX(), UR.getPresentY(), static_cast<const float &>(BIN_Z), SHAKE_OFF_TRAVEL_TIME_UP);
    }
#endif
}

void segmentation(const Mat &input, Mat &output) {
    Mat blueChannel;
    extractChannel(input, blueChannel, 0);
    threshold(blueChannel, output, SEGMENTATION_THRESHOLD, 255, THRESH_BINARY_INV);
};


vector<vector<Point2f>> classifier(const Mat &input) {
    vector<blobStruct> blob;
    Mat binaryImage;
    uint8_t numberOfGoodParticles = 0;

    segmentation(input, binaryImage);
    vector<vector<Point>> blobContours;
    findContours(binaryImage, blobContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (const auto &blobContour : blobContours) {
        float tempArea = static_cast<float>(contourArea(blobContour));
        if (tempArea < AREA_UPPER_LIMIT_ALL && tempArea > AREA_LOWER_LIMIT) {
            numberOfGoodParticles++;
            blob.resize(numberOfGoodParticles);
            blob[numberOfGoodParticles - 1].contour = blobContour;
            blob[numberOfGoodParticles - 1].feature[FEATURE_AREA] = tempArea;
        }
    }

    std::vector<std::vector<Point>> blobHulls(numberOfGoodParticles);
    vector<vector<int>> blobHullsIndices(numberOfGoodParticles);
    vector<Vec4i> defects(numberOfGoodParticles);

    vector<vector<Point2f>> output(NUMBER_OF_CATEGORIES);

    for (uint16_t i = 0; i < numberOfGoodParticles; ++i) {
        Moments blobMoments = moments(blob[i].contour);
        float a, b;
        a = static_cast<float>(blobMoments.mu20 + blobMoments.mu02);
        b = static_cast<float>(4 * pow(blobMoments.mu11, 2) + pow((blobMoments.mu20 - blobMoments.mu02), 2));
        blob[i].feature[FEATURE_ELONGATION] = (a + sqrt(b)) / (a - sqrt(b));
        blob[i].centreOfMass = Point2f(static_cast<int>(blobMoments.m10 / blobMoments.m00),
                                       static_cast<int>(blobMoments.m01 / blobMoments.m00));

        if (blob[i].feature[FEATURE_AREA] > AREA_UPPER_LIMIT_CLUSTER) {
            output.at(UNCLASSIFIED).emplace_back(blob[i].centreOfMass);
            continue;
        }

        uint16_t defectCounter = 0;

        convexHull(blob[i].contour, blobHulls[i]);

        convexHull(blob[i].contour, blobHullsIndices[i]);
        if (blobHullsIndices[i].size() > 3) {
            convexityDefects(blob[i].contour, blobHullsIndices[i], defects);
            for (auto &defect : defects) {
                if (defect[3] > DEPTH_OF_DEFECT) {
                    defectCounter++;
                }
            }
        }

        blob[i].feature[FEATURE_DEFECTS] = defectCounter;
        blob[i].feature[FEATURE_SOLIDITY] = static_cast<float>(blob[i].feature[FEATURE_AREA] /
                                                               (contourArea(blobHulls[i])));
        blob[i].bounding_Rect = boundingRect(blob[i].contour);

        blob[i].feature[FEATURE_HEIGHT] = blob[i].bounding_Rect.height;
        blob[i].feature[FEATURE_WIDTH] = blob[i].bounding_Rect.width;
        blob[i].feature[FEATURE_PERIMETER] = static_cast<float>(arcLength(blob[i].contour, true));

        float tempEllipseMajor, tempEllipseMinor;
        RotatedRect tempEllipse = fitEllipse(blob[i].contour);
        if (tempEllipse.size.width > tempEllipse.size.height) {
            tempEllipseMajor = tempEllipse.size.width;
            tempEllipseMinor = tempEllipse.size.height;
        } else {
            tempEllipseMajor = tempEllipse.size.height;
            tempEllipseMinor = tempEllipse.size.width;
        }

        blob[i].feature[FEATURE_ASPECT_RATIO] = tempEllipseMajor / tempEllipseMinor;
        blob[i].feature[FEATURE_CIRCULARITY] = static_cast<float>(4 * CV_PI * blob[i].feature[FEATURE_AREA] / (blob[i].feature[FEATURE_PERIMETER] * blob[i].feature[FEATURE_PERIMETER]));
        blob[i].feature[FEATURE_ROUND] = static_cast<float>(4 * blob[i].feature[FEATURE_AREA] / (CV_PI * tempEllipseMajor * tempEllipseMajor));

        Mat redChannel;
        extractChannel(input, redChannel, 2);

        float colourCounter = 0, tempColour = 0;
        for (int x = blob[i].bounding_Rect.x; x < (blob[i].bounding_Rect.x + blob[i].bounding_Rect.width); ++x) {
            for (int y = blob[i].bounding_Rect.y; y < (blob[i].bounding_Rect.y + blob[i].bounding_Rect.height); ++y) {
                if (redChannel.at<unsigned char>(y, x) < 255) {
                    tempColour = tempColour + redChannel.at<unsigned char>(y, x);
                    colourCounter++;
                }
            }
        }
        blob[i].feature[FEATURE_INTENSITY] = (tempColour / colourCounter);
    }

    for (auto &i : blob) {
        for (int k = 0; k < NUMBER_OF_CLASSES; ++k) {
            for (int j = 0; j < NUMBER_OF_FEATURES; ++j) {
                i.gaussResult[k] *= exp((-(pow((i.feature[j] - featuresMean[k][j]), 2))) / (2 * featuresSigma[k][j])) / (sqrt(2 * CV_PI * featuresSigma[k][j]));
            }
        }

        vector<double> tempVector(NUMBER_OF_CLASSES);

        for (auto m = 0; m < NUMBER_OF_CLASSES; ++m) {
            tempVector[m] = i.gaussResult[m];
        }

        sort(begin(tempVector), end(tempVector));

        for (uint8_t l = 0; l < NUMBER_OF_CLASSES; ++l) {
            if (i.gaussResult[l] == tempVector.at(NUMBER_OF_CLASSES - 1)) {
                i.result = l;
            } else if (i.gaussResult[l] == tempVector.at(NUMBER_OF_CLASSES - 2)) {
                i.resultSecond = l;
            }
        }

        double resultRatio = i.gaussResult[i.result] / i.gaussResult[i.resultSecond];

        if (resultRatio < RESULT_RATIO_LIMIT) {
            output.at(UNCLASSIFIED).emplace_back(i.centreOfMass);
        } else {
            if (i.result == BEETLE) {
                output.at(BEETLE).emplace_back(i.centreOfMass);
            } else if (i.result == LARVAE) {
                output.at(LARVAE).emplace_back(i.centreOfMass);
            } else if (i.result == PUPA) {
                output.at(PUPA).emplace_back(i.centreOfMass);
            } else {
                output.at(DEAD).emplace_back(i.centreOfMass);
            }
        }
    }

    sortingOperatorAscendingPointX sortingBasedOnCoordinateX;
    for (uint8_t l = 0; l < NUMBER_OF_CATEGORIES; ++l) {
        sort(output.at(l).begin(), output.at(l).end(), sortingBasedOnCoordinateX);
    }

    return output;
};