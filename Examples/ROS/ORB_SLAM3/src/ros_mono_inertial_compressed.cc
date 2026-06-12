/**
* Mono-Inertial ORB-SLAM3 node with CompressedImage + IMU
* Based on ros_mono_inertial.cc, modified for compressed image input
* and 0.5x downsampling to match AMtown calibration.
*/
static const double kDownsampleScale = 0.5;

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb):
        mpSLAM(pSLAM), mpImuGb(pImuGb){}

    void GrabCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg);
    void SyncWithImu();

    queue<pair<double, cv::Mat>> imgBuf;
    std::mutex mBufMutex;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial_Compressed");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial_Compressed path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);

    ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img = n.subscribe("/camera/image_raw/compressed", 5, &ImageGrabber::GrabCompressedImage, &igb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
        if(image.empty())
        {
            ROS_ERROR("Failed to decode compressed image");
            return;
        }

        cv::Mat image_resized;
        cv::resize(image, image_resized, cv::Size(), kDownsampleScale, kDownsampleScale, cv::INTER_AREA);

        double timestamp = msg->header.stamp.toSec();

        mBufMutex.lock();
        if(!imgBuf.empty())
            imgBuf.pop();
        imgBuf.push(make_pair(timestamp, image_resized));
        mBufMutex.unlock();
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("OpenCV exception: %s", e.what());
    }
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        cv::Mat im;
        double tIm = 0;
        if (!imgBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            tIm = imgBuf.front().first;
            if(tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            {
                this->mBufMutex.lock();
                im = imgBuf.front().second.clone();
                imgBuf.pop();
                this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                   mpImuGb->imuBuf.front()->linear_acceleration.y,
                                   mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                   mpImuGb->imuBuf.front()->angular_velocity.y,
                                   mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            mpSLAM->TrackMonocular(im, tIm, vImuMeas);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}
