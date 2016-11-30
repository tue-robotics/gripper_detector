#include "gripper_detector/gripper_detector.h"

// srvs
#include <gripper_detector/DetectGripper.h>

// rgbd
#include <rgbd/Client.h>
#include <rgbd/View.h>

// arUco
#include <aruco/aruco.h>

#include <rosconsole/macros_generated.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped arucoMarker2GeometryMsg(const aruco::Marker& marker)
{
    cv::Mat rot(3, 3, CV_32FC1);
    cv::Rodrigues(marker.Rvec, rot);
    cv::Mat tran = marker.Tvec;

    cv::Mat rotate_to_ros(3, 3, CV_32FC1);
    // -1 0 0
    //  0 0 1
    //  0 1 0
    rotate_to_ros.at<float>(0,0) = -1.0;
    rotate_to_ros.at<float>(0,1) = 0.0;
    rotate_to_ros.at<float>(0,2) = 0.0;
    rotate_to_ros.at<float>(1,0) = 0.0;
    rotate_to_ros.at<float>(1,1) = 0.0;
    rotate_to_ros.at<float>(1,2) = 1.0;
    rotate_to_ros.at<float>(2,0) = 0.0;
    rotate_to_ros.at<float>(2,1) = 1.0;
    rotate_to_ros.at<float>(2,2) = 0.0;
    rot = rot*rotate_to_ros.t();

    tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                         rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                         rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

    geometry_msgs::PoseStamped pose;

    tf::Quaternion q;
    tf_rot.getRotation(q);

    pose.pose.orientation.w = q.getW();
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();

    pose.pose.position.x = tran.at<float>(0,0);
    pose.pose.position.y = tran.at<float>(1,0);
    pose.pose.position.z = tran.at<float>(2,0);

    return pose;
}


GripperDetector::GripperDetector():
    nh_("~")
{
    client_.intialize("rgbd");
}

GripperDetector::~GripperDetector()
{
}

bool GripperDetector::configure()
{
    // TODO: make configurable
    base_frame_ = "/map/";
    name_prefix_ = "/amigo/gripper_marker_";
    marker_frames_[985] = "right";
    marker_frames_[177] = "left";
    marker_size_ = 0.042;
    max_iter_ = 1;

    // TODO: Read marker messages from config and relate them to their grippers
    // TODO: Use actual kinect camera parameters!

    double fx, fy, cx, cy, d1, d2;
    fx = 538.6725257330964;
    fy = 502.5794530135827;
    //    cx = 313.68782938;
    //    cy = 259.01834898;
    cx = 640;
    cy = 512;
    d1 = 0.18126525;
    d2 = -0.39866885;

    cv::Size size(1280,1024);

    cv::Mat C = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    cv::Mat D = (cv::Mat_<double>(5,1) << d1, d2, 0.0, 0.0, 0.0);

    cam_.setParams(C,D,size);

    if ( cam_.isValid() )
    {
        ROS_INFO_STREAM("Gripper Detector: Initialized!");
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Gripper Detector: Camera parameters not valid!");
        return false;
    }

    ros::NodeHandle n;
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("gripper_pose", 100);
}

bool GripperDetector::detectMarkers(geometry_msgs::PoseStamped& gripper_pose, std::string& requested_arm)
{
    unsigned int iter = 0;

    rgbd::Image image;

    while (iter < max_iter_)
    {
        if (client_.nextImage(image))
        {
            std::vector<aruco::Marker> markers;

            // Detect the AR markers!
            ROS_INFO_STREAM("Gripper Detector: detecting markers");
            detector_.detect(image.getRGBImage(), markers, cam_, marker_size_);
            ROS_INFO_STREAM("Gripper Detector: " << markers.size() << " markers detected");

            for ( std::vector<aruco::Marker>::iterator it = markers.begin(); it != markers.end(); ++it )
            {
                ROS_INFO_STREAM("Gripper Detector: Marker found with ID: " << it->id);

                // If the detected marker is not one of the gripper markers, continue
                if ( marker_frames_.find(it->id) == marker_frames_.end() )
                {
                    ROS_WARN_STREAM("Gripper Detector: Found an AR marker that is not a gripper");
                    continue;
                }

                // If the detected marker is not requested, continue
                if ( marker_frames_[it->id] != requested_arm )
                {
                    ROS_INFO_STREAM("Gripper Detector: Found a gripper that was not requested");
                    continue;
                }

                gripper_pose = arucoMarker2GeometryMsg(*it);

                gripper_pose.header.frame_id = image.getFrameId();
                gripper_pose.header.stamp = ros::Time(image.getTimestamp());

                ROS_INFO_STREAM("Gripper Detector: Gripper found");
                return true;
            }
        }
        else
        {
            ROS_WARN_STREAM("No rgbd image");
        }

        iter++;
    }

    ROS_WARN_STREAM("Gripper not found");
    return false;
}

bool GripperDetector::detectGripperCallback(gripper_detector::DetectGripper::Request  &req,
                                            gripper_detector::DetectGripper::Response &res)
{
    res.succeeded = detectMarkers(res.gripper_pose, req.arm);

    pose_pub_.publish(res.gripper_pose);

    return res.succeeded;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_detector");
    ros::NodeHandle n;

    GripperDetector gripperDetector;

    if ( !gripperDetector.configure() )
    {
        ROS_ERROR_STREAM("Something went wrong configuring the gripper detector!");
        return 1;
    }

//    ros::ServiceServer service = n.advertiseService("detect_gripper",
//                                                    &GripperDetector::detectGripperCallback,
//                                                    &gripperDetector);

//    ROS_INFO_STREAM("Ready to detect gripper.");
//    ros::spin();

    while ( !ros::isShuttingDown() )
    {
      geometry_msgs::PoseStamped pose_msg;
      std::string arm("right");
      gripperDetector.detectMarkers(pose_msg, arm);
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }

    return 0;
}
