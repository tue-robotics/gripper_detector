#ifndef GRIPPER_DETECTOR_H
#define GRIPPER_DETECTOR_H

#include <rgbd/Client.h>

//#include "libauto_tracker/tracking.h"
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMbTracker.h>
#include <visp/vpDetectorBase.h>

// TODO: Fixed a build error by hacking in /usr/include/SbBasic.h: added #include <Inventor/C/errors/debugerror.h>

class gripper_detector
{
    rgbd::Client* kinect_client_;

public:
    gripper_detector();

    ros::NodeHandle nh_;
    vpCameraParameters cam_;
    bool debug_display_;
    vpImage<vpRGBa> I_; // Image used for debug display
    unsigned int iter_;
    bool tracking_;

    rgbd::Client client_;

    vpMbTracker* tracker_;
    vpDetectorBase *detector_;

    bool configure();
    void update();


};

#endif // GRIPPER_DETECTOR_H
