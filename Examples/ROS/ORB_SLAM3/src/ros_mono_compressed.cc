/**
* This file is part of ORB-SLAM3
*
* Modified to support CompressedImage messages
*/
// Downsample scale: 
static const double kDownsampleScale = 0.5;
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Compressed path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw/compressed", 5, &ImageGrabber::GrabCompressedImage,&igb);

    ros::spin();

    // Save full-frame camera trajectory (TUM format, timestamps in seconds)
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        if(image.empty())
        {
            ROS_ERROR("Failed to decode compressed image");
            return;
        }
        
        // Downsample to reduce load and improve tracking
        cv::Mat image_resized;
        cv::resize(image, image_resized, cv::Size(), kDownsampleScale, kDownsampleScale, cv::INTER_AREA);
        
        double timestamp = msg->header.stamp.toSec();
        mpSLAM->TrackMonocular(image_resized, timestamp);
        /*
        // Decode compressed image
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        
        if(image.empty())
        {
            ROS_ERROR("Failed to decode compressed image");
            return;
        }

        // Convert to grayscale if needed (ORB-SLAM3 works with grayscale)
        // cv::Mat gray;
        // cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        double timestamp = msg->header.stamp.toSec();
        mpSLAM->TrackMonocular(image, timestamp);
        */
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("OpenCV exception: %s", e.what());
        return;
    }
}


