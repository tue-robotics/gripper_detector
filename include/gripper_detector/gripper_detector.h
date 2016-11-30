#ifndef GRIPPER_DETECTOR_H
#define GRIPPER_DETECTOR_H

#include <rgbd/Client.h>
#include <aruco/markerdetector.h>
#include <gripper_detector/DetectGripper.h>

// TODO: Fixed a build error by hacking in /usr/include/SbBasic.h: added #include <Inventor/C/errors/debugerror.h>

class GripperDetector
{
    rgbd::Client* kinect_client_;

    ros::NodeHandle nh_;
    unsigned int max_iter_;
    std::string name_prefix_;
    std::map<int,std::string> marker_frames_;
    float marker_size_;

    rgbd::Client client_;

    aruco::MarkerDetector detector_;
    aruco::CameraParameters cam_;

    std::string base_frame_;

    ros::Publisher pose_pub_;

public:
    GripperDetector();
    ~GripperDetector();

    bool configure();
    bool detectMarkers(geometry_msgs::PoseStamped& gripper_pose, std::string& requested_arm);
    bool detectGripperCallback(gripper_detector::DetectGripper::Request  &req,
                      gripper_detector::DetectGripper::Response &res);
};

#endif // GRIPPER_DETECTOR_H
