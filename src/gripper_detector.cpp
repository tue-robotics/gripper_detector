#include "gripper_detector/gripper_detector.h"

#include <visp/vpDetectorQRCode.h>
#include <visp/vpImageIo.h>

// rgbd
#include <rgbd/Client.h>
#include <rgbd/View.h>

// visp includes
#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp_bridge/3dpose.h>
#include <visp/vpImageConvert.h>

// detectors
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>

gripper_detector::gripper_detector():
    nh_("~"),
    iter_(0),
    tracking_(false)
{
    client_.intialize("rgbd");
    detector_ = new vpDetectorQRCode;
    tracker_ = new vpMbEdgeKltTracker();
}

bool gripper_detector::configure()
{
    // set camera parameters
    tracker_->setCameraParameters(cam_);

    // Set if display is on or off and if features should be displayed
    tracker_->setDisplayFeatures(true);

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
        vpImage<unsigned char> vp_image_gray; // Create a gray level image container

        vpImageConvert::convert(image.getRGBImage(),vp_image_gray);

        if ( tracking_ )
        {
//            try
//            {
                tracker_->track(vp_image_gray);

                geometry_msgs::Pose pose;
                vpHomogeneousMatrix vp_pose = tracker_->getPose();
                pose = visp_bridge::toGeometryMsgsPose(vp_pose);

                tracker_->display(vp_image_gray,vp_pose,cam_,vpColor::darkRed);
                vpDisplay::flush(vp_image_gray);
//            }
//            catch ( std::exception e )
//            {
//                std::cout << "ERROR: " << e << std::endl;
//            }
        }
        else
        {
            // Marker is still to be detected, so do that
            if ( detector_->detect(vp_image_gray) )
            {
                // Go through the detected markers, and get the correct one
                for ( int i = 0; i < detector_->getNbObjects(); ++i )
                {
                    std::vector<vpImagePoint> bbox = detector_->getPolygon(i);
                    std::cout << "Point list of marker with message \"" << detector_->getMessage(i) << "\":" << std::endl;
                    for(std::vector<vpImagePoint>::const_iterator it = bbox.begin(); it != bbox.end(); ++it)
                        std::cout << "\t" << *it << std::endl;
                    std::cout << std::endl;
                    tracker_->init(vp_image_gray);
                    tracking_ = true;
                }
            }
        }

//            // Show depth image
//            if (image.getDepthImage().data)
//                cv::imshow("depth_original", image.getDepthImage() / 8);

//            // Show rgb image
//            if (image.getRGBImage().data)
//                cv::imshow("rgb_original", image.getRGBImage());

//            // Show combined image
//            if (image.getDepthImage().data && image.getRGBImage().data)
//            {
//                cv::Mat image_hsv;
//                cv::cvtColor(image.getRGBImage(), image_hsv, CV_BGR2HSV);

//                rgbd::View view(image, image_hsv.cols);

//                cv::Mat canvas_hsv(view.getHeight(), view.getWidth(), CV_8UC3, cv::Scalar(0, 0, 0));

//                for(int y = 0; y < view.getHeight(); ++y)
//                {
//                    for(int x = 0; x < view.getWidth(); ++x)
//                    {
//                        float d = view.getDepth(x, y);
//                        if (d == d)
//                        {
//                            cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(y, x);
//                            hsv[2] = 255 - (d / 8 * 255);
//                            canvas_hsv.at<cv::Vec3b>(y, x) = hsv;
//                        }
//                    }
//                }

//                cv::Mat canvas_bgr;
//                cv::cvtColor(canvas_hsv, canvas_bgr, CV_HSV2BGR);
//                cv::imshow("image + depth", canvas_bgr);
//            }

//            cv::waitKey(3);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_detector");

    gripper_detector gripperdetector;

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
