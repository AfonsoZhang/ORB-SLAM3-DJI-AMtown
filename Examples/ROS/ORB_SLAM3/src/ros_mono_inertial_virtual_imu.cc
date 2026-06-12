/**
 * Mono-Inertial ORB-SLAM3 with Virtual IMU
 *
 * The DJI camera is on a 3-axis stabilized gimbal, so the camera-body
 * extrinsic T_b_c changes over time. This node creates a "virtual IMU"
 * rigidly attached to the camera by rotating body-frame IMU measurements
 * into the camera frame using the drone attitude and gimbal angles.
 *
 * Key insight: accelerometer data is rotated at full IMU rate (400 Hz),
 * but camera angular velocity is computed at gimbal rate (50 Hz) to avoid
 * noise amplification from numerical differentiation at 400 Hz.
 *
 * With the virtual IMU, T_b_c1 can be set to identity (pure translation).
 *
 * Subscribed topics:
 *   /dji_osdk_ros/imu              - DJI body IMU (400 Hz, m/s^2)
 *   /dji_osdk_ros/attitude         - Drone attitude quaternion (~100 Hz)
 *   /dji_osdk_ros/gimbal_angle     - Gimbal angles in degrees (~50 Hz)
 *   /left_camera/image/compressed  - Compressed camera images (~10 Hz)
 */

static const double kDownsampleScale = 0.5;

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <deque>
#include <thread>
#include <mutex>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../../../include/System.h"
#include "../include/ImuTypes.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

// ---- Rotation helpers ----

// R_world_gimbal from DJI gimbal angles (degrees)
static Eigen::Matrix3d gimbalToRotMat(double pitch_deg, double roll_deg, double yaw_deg)
{
    double yaw   = yaw_deg   * M_PI / 180.0;
    double pitch = -pitch_deg * M_PI / 180.0;
    double roll_r = -roll_deg * M_PI / 180.0;

    Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(roll_r, Eigen::Vector3d::UnitX());
    return (Rz * Ry * Rx).toRotationMatrix();
}

// Fixed rotation: camera (OpenCV) frame in gimbal-zero (FLU) frame
static const Eigen::Matrix3d R_gim_cam = (Eigen::Matrix3d() <<
     0,  0,  1,
    -1,  0,  0,
     0, -1,  0).finished();

// ---- Timestamped data for interpolation ----

struct AttitudeStamped {
    double t;
    Eigen::Quaternion<double, Eigen::DontAlign> q;
};

struct GimbalStamped {
    double t;
    double pitch, roll, yaw;
};

// ---- Main grabber classes ----

class VirtualImuGrabber
{
public:
    void GrabBodyImu(const sensor_msgs::ImuConstPtr &msg);
    void GrabAttitude(const geometry_msgs::QuaternionStampedConstPtr &msg);
    void GrabGimbal(const geometry_msgs::Vector3StampedConstPtr &msg);

    struct BodyImuSample {
        double t;
        Eigen::Matrix<double, 3, 1, Eigen::DontAlign> acc;
        Eigen::Matrix<double, 3, 1, Eigen::DontAlign> gyr;
    };

    deque<BodyImuSample> imuBuf;
    deque<AttitudeStamped> attBuf;
    deque<GimbalStamped> gimBuf;
    std::mutex mBufMutex;

    bool getR_bc(double t, Eigen::Matrix3d &R_bc);

    bool hasValidRotation = false;
    Eigen::Matrix3d lastR_bc = Eigen::Matrix3d::Identity();

    // Camera angular velocity computed at gimbal rate (50 Hz),
    // held constant between gimbal updates
    Eigen::Vector3d omega_cam = Eigen::Vector3d::Zero();
    bool hasOmega = false;

private:
    bool interpAttitude(double t, Eigen::Quaternion<double, Eigen::DontAlign> &q_out);
    bool interpGimbal(double t, double &pitch, double &roll, double &yaw);

    // For computing camera angular velocity from consecutive gimbal updates
    Eigen::Matrix3d prevR_wc = Eigen::Matrix3d::Identity();
    double prevGimbalTime = -1;
    bool hasPrevGimbal = false;
};

bool VirtualImuGrabber::interpAttitude(double t, Eigen::Quaternion<double, Eigen::DontAlign> &q_out)
{
    if (attBuf.size() < 2) return false;
    if (t < attBuf.front().t || t > attBuf.back().t) return false;

    for (size_t i = 0; i + 1 < attBuf.size(); i++) {
        if (attBuf[i].t <= t && t <= attBuf[i+1].t) {
            double dt = attBuf[i+1].t - attBuf[i].t;
            double alpha = (dt > 1e-9) ? (t - attBuf[i].t) / dt : 0.0;
            q_out = attBuf[i].q.slerp(alpha, attBuf[i+1].q);
            return true;
        }
    }
    return false;
}

bool VirtualImuGrabber::interpGimbal(double t, double &pitch, double &roll, double &yaw)
{
    if (gimBuf.size() < 2) return false;
    if (t < gimBuf.front().t || t > gimBuf.back().t) return false;

    for (size_t i = 0; i + 1 < gimBuf.size(); i++) {
        if (gimBuf[i].t <= t && t <= gimBuf[i+1].t) {
            double dt = gimBuf[i+1].t - gimBuf[i].t;
            double alpha = (dt > 1e-9) ? (t - gimBuf[i].t) / dt : 0.0;
            pitch = gimBuf[i].pitch + alpha * (gimBuf[i+1].pitch - gimBuf[i].pitch);
            roll  = gimBuf[i].roll  + alpha * (gimBuf[i+1].roll  - gimBuf[i].roll);
            yaw   = gimBuf[i].yaw   + alpha * (gimBuf[i+1].yaw   - gimBuf[i].yaw);
            return true;
        }
    }
    return false;
}

bool VirtualImuGrabber::getR_bc(double t, Eigen::Matrix3d &R_bc)
{
    Eigen::Quaternion<double, Eigen::DontAlign> q_wb;
    double pitch, roll, yaw;

    if (interpAttitude(t, q_wb) && interpGimbal(t, pitch, roll, yaw)) {
        Eigen::Matrix3d R_wb = q_wb.toRotationMatrix();
        Eigen::Matrix3d R_wg = gimbalToRotMat(pitch, roll, yaw);
        Eigen::Matrix3d R_wc = R_wg * R_gim_cam;
        R_bc = R_wb.transpose() * R_wc;
        lastR_bc = R_bc;
        hasValidRotation = true;
        return true;
    }

    if (hasValidRotation) {
        R_bc = lastR_bc;
        return true;
    }

    return false;
}

void VirtualImuGrabber::GrabBodyImu(const sensor_msgs::ImuConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    BodyImuSample s;
    s.t = msg->header.stamp.toSec();
    s.acc = Eigen::Vector3d(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
    s.gyr = Eigen::Vector3d(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);
    imuBuf.push_back(s);
    while (imuBuf.size() > 5000) imuBuf.pop_front();
}

void VirtualImuGrabber::GrabAttitude(const geometry_msgs::QuaternionStampedConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    AttitudeStamped a;
    a.t = msg->header.stamp.toSec();
    a.q = Eigen::Quaternion<double, Eigen::DontAlign>(msg->quaternion.w, msg->quaternion.x,
                             msg->quaternion.y, msg->quaternion.z);
    attBuf.push_back(a);
    while (attBuf.size() > 2000) attBuf.pop_front();
}

void VirtualImuGrabber::GrabGimbal(const geometry_msgs::Vector3StampedConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    GimbalStamped g;
    g.t = msg->header.stamp.toSec();
    g.pitch = msg->vector.x;
    g.roll  = msg->vector.y;
    g.yaw   = msg->vector.z;
    gimBuf.push_back(g);
    while (gimBuf.size() > 1000) gimBuf.pop_front();

    // Compute camera angular velocity from consecutive gimbal measurements (50 Hz)
    // This avoids noisy numerical differentiation at 400 Hz IMU rate
    Eigen::Matrix3d R_wg = gimbalToRotMat(g.pitch, g.roll, g.yaw);
    Eigen::Matrix3d R_wc = R_wg * R_gim_cam;

    if (hasPrevGimbal && (g.t - prevGimbalTime) > 1e-9) {
        double dt = g.t - prevGimbalTime;
        // R_wc(t) = exp([ω_world]× * dt) * R_wc(t-dt)
        // => exp([ω_world]× * dt) = R_wc(t) * R_wc(t-dt)^T
        Eigen::Matrix3d dR = R_wc * prevR_wc.transpose();
        Eigen::Vector3d omega_world;
        omega_world.x() = (dR(2,1) - dR(1,2)) / (2.0 * dt);
        omega_world.y() = (dR(0,2) - dR(2,0)) / (2.0 * dt);
        omega_world.z() = (dR(1,0) - dR(0,1)) / (2.0 * dt);
        // Transform to camera frame
        Eigen::Matrix3d R_cw = R_wc.transpose();
        omega_cam = R_cw * omega_world;
        hasOmega = true;
    }

    prevR_wc = R_wc;
    prevGimbalTime = g.t;
    hasPrevGimbal = true;
}

// ---- Image grabber ----

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, VirtualImuGrabber *pVimu):
        mpSLAM(pSLAM), mpVimu(pVimu) {}

    void GrabCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg);
    void SyncWithImu();

    queue<pair<double, cv::Mat>> imgBuf;
    std::mutex mBufMutex;
    ORB_SLAM3::System* mpSLAM;
    VirtualImuGrabber *mpVimu;
};

void ImageGrabber::GrabCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
        if (image.empty()) { ROS_ERROR("Failed to decode image"); return; }

        cv::Mat image_resized;
        cv::resize(image, image_resized, cv::Size(), kDownsampleScale, kDownsampleScale, cv::INTER_AREA);

        std::lock_guard<std::mutex> lock(mBufMutex);
        if (!imgBuf.empty()) imgBuf.pop();
        imgBuf.push(make_pair(msg->header.stamp.toSec(), image_resized));
    } catch (cv::Exception& e) {
        ROS_ERROR("OpenCV exception: %s", e.what());
    }
}

void ImageGrabber::SyncWithImu()
{
    while (ros::ok())
    {
        cv::Mat im;
        double tIm = 0;

        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            if (imgBuf.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            tIm = imgBuf.front().first;
        }

        {
            std::lock_guard<std::mutex> lock(mpVimu->mBufMutex);
            if (mpVimu->imuBuf.empty() || mpVimu->imuBuf.back().t < tIm) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            im = imgBuf.front().second.clone();
            imgBuf.pop();
        }

        // Transform body IMU to camera-frame virtual IMU
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            std::lock_guard<std::mutex> lock(mpVimu->mBufMutex);
            while (!mpVimu->imuBuf.empty() && mpVimu->imuBuf.front().t <= tIm)
            {
                auto &s = mpVimu->imuBuf.front();

                Eigen::Matrix3d R_bc;
                if (mpVimu->getR_bc(s.t, R_bc)) {
                    Eigen::Matrix3d R_cb = R_bc.transpose();

                    // Accelerometer: rotate body acc to camera frame
                    Eigen::Vector3d acc_cam = R_cb * s.acc;

                    // Gyroscope: use camera angular velocity pre-computed
                    // at gimbal rate (50 Hz) — no noisy 400 Hz differentiation
                    Eigen::Vector3d gyr_cam = mpVimu->hasOmega ?
                        mpVimu->omega_cam : Eigen::Vector3d::Zero();

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                        cv::Point3f(acc_cam.x(), acc_cam.y(), acc_cam.z()),
                        cv::Point3f(gyr_cam.x(), gyr_cam.y(), gyr_cam.z()),
                        s.t));
                }

                mpVimu->imuBuf.pop_front();
            }
        }

        if (!vImuMeas.empty()) {
            static int diag_count = 0;
            if (diag_count++ % 100 == 0) {
                auto &m = vImuMeas.back();
                float ax = m.a.x(), ay = m.a.y(), az = m.a.z();
                float norm_a = std::sqrt(ax*ax + ay*ay + az*az);
                ROS_INFO("VirtualIMU [%d samples] acc=(%.2f,%.2f,%.2f) |a|=%.2f gyr=(%.4f,%.4f,%.4f)",
                    (int)vImuMeas.size(), ax, ay, az, norm_a,
                    m.w.x(), m.w.y(), m.w.z());
            }
            mpSLAM->TrackMonocular(im, tIm, vImuMeas);
        }
    }
}

// ---- Main ----

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial_VirtualIMU");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc < 3 || argc > 4) {
        cerr << "Usage: Mono_Inertial_VirtualIMU path_to_vocabulary path_to_settings [no_viewer]" << endl;
        return 1;
    }

    bool bUseViewer = true;
    if (argc == 4 && string(argv[3]) == "no_viewer")
        bUseViewer = false;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, bUseViewer);

    VirtualImuGrabber vimu;
    ImageGrabber igb(&SLAM, &vimu);

    ros::Subscriber sub_imu = n.subscribe("/dji_osdk_ros/imu", 1000,
                                          &VirtualImuGrabber::GrabBodyImu, &vimu);
    ros::Subscriber sub_att = n.subscribe("/dji_osdk_ros/attitude", 200,
                                          &VirtualImuGrabber::GrabAttitude, &vimu);
    ros::Subscriber sub_gim = n.subscribe("/dji_osdk_ros/gimbal_angle", 100,
                                          &VirtualImuGrabber::GrabGimbal, &vimu);
    ros::Subscriber sub_img = n.subscribe("/left_camera/image/compressed", 5,
                                          &ImageGrabber::GrabCompressedImage, &igb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}
