#ifndef GRIPPER_DETECTOR_H
#define GRIPPER_DETECTOR_H

#include <rgbd/Client.h>
#include <aruco/markerdetector.h>

// TODO: Fixed a build error by hacking in /usr/include/SbBasic.h: added #include <Inventor/C/errors/debugerror.h>

class gripper_detector
{
    rgbd::Client* kinect_client_;

public:
    gripper_detector();
    ~gripper_detector();

    ros::NodeHandle nh_;
    unsigned int iter_;
    bool tracking_;

    rgbd::Client client_;

    aruco::MarkerDetector detector_;

    bool configure();
    void update();


};

#endif // GRIPPER_DETECTOR_H
