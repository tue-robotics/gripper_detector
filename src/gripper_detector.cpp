#include "gripper_detector/gripper_detector.h"

// rgbd
#include <rgbd/Client.h>
#include <rgbd/View.h>

// arUco
#include <aruco/aruco.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


tf::Transform arucoMarker2Tf(const aruco::Marker& marker)
{
    cv::Mat rot(3, 3, CV_32FC1);
        cv::Rodrigues(marker.Rvec, rot);
        cv::Mat tran = marker.Tvec;

        cv::Mat rotate_to_ros(3, 3, CV_32FC1);
        // -1 0 0
        // 0 0 1
        // 0 1 0
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

        tf::Vector3 tf_orig(tran.at<float>(0,0), tran.at<float>(1,0), tran.at<float>(2,0));

        return tf::Transform(tf_rot, tf_orig);
}


gripper_detector::gripper_detector():
    nh_("~"),
    iter_(0),
    tracking_(false)
{
    client_.intialize("rgbd");
}

gripper_detector::~gripper_detector()
{
}

bool gripper_detector::configure()
{
    // TODO: make configurable
    base_frame_ = "/map/";

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

    rgbd::Image image;

    while(!client_.nextImage(image))
        cv::waitKey(10);

    cv::Size size(image.getRGBImage().cols,image.getRGBImage().rows);

    cv::Mat C = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    cv::Mat D = (cv::Mat_<double>(5,1) << d1, d2, 0.0, 0.0, 0.0);

    cam_.setParams(C,D,size);

    if ( cam_.isValid() )
    {
        std::cout << "Initialized!" << std::endl;
        return true;
    }
    else
    {
        std::cout << "Camera parameters not valid!" << std::endl;
        return false;
    }
}

void gripper_detector::update()
{
    rgbd::Image image;

    if (client_.nextImage(image))
    {
        cv::Mat showImage(image.getRGBImage());

        std::vector<aruco::Marker> markers;

        // Detect the AR markers!
        detector_.detect(image.getRGBImage(), markers, cam_, 0.048);

//        tf::Transform transform;

        for ( unsigned int i=0; i < markers.size(); i++ )
        {
//            transform = arucoMarker2Tf(markers[i]);
//            geometry_msgs::TransformStamped tf_stamped;
//            tf_stamped.child_frame_id(stdmarkers[i].id);
            std::cout << markers[i].id << std::endl;
//            broadcaster_.sendTransform();
            markers[i].draw(showImage,cv::Scalar(0,0,255),2);
        }
        cv::imshow("in",showImage);
        cv::waitKey(1);
    }
    else
        std::cout << "No rgbd image" << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_detector");

    std::cout << "Initializing gripper detector..." << std::endl;
    gripper_detector gripperdetector;

    aruco::MarkerDetector detector;

    if ( !gripperdetector.configure() )
    {
        std::cout << "Something went wrong configuring the gripper detector!" << std::endl;
        return 1;
    }

    ros::Rate r(10);
    while (ros::ok())
    {
        gripperdetector.update();
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
