#include "gripper_detector/gripper_detector.h"

// rgbd
#include <rgbd/Client.h>
#include <rgbd/View.h>

// arUco
#include <aruco/aruco.h>


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
    // Set if display is on or off and if features should be displayed

    // TODO: make configurable
    // TODO: set tracker model

    // TODO: Read marker messages from config and relate them to their grippers

    return true;
}

void gripper_detector::update()
{
    rgbd::Image image;

    if (client_.nextImage(image))
    {
        cv::Mat showImage(image.getRGBImage());
        aruco::CameraParameters cam;
        cam.readFromFile("/home/rokus/camera.yml");
        cam.CamSize.width = 1280;
        cam.CamSize.height = 1024;

//        double fx, fy, cx, cy;
//        fx = 538.6725257330964;
//        fy = 502.5794530135827;
//        cx = 313.68782938;
//        cy = 259.01834898;

        vector<aruco::Marker> markers;

        // Detect the AR markers!
        detector_.detect(image.getRGBImage(), markers, cam, 0.025);

        for ( unsigned int i=0; i < markers.size(); i++ )
        {
            std::cout << markers[i] << endl;
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

    ros::Rate r(30);
    while (ros::ok())
    {
        gripperdetector.update();
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
