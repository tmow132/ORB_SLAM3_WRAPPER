#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "System.h"
#include "ImuTypes.h"
#include "Converter.h"
#include "SerializationUtils.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Atlas.h"
#include "Settings.h" // Include the correct path to your ImuTypes.h file

using namespace std::chrono_literals;

class ImuGrabber
{
public:
  ImuGrabber(){};
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
   ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bClahe)
      : mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe) {}

 

  void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
  void SyncWithImu();

  std::queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
  std::mutex mBufMutex;

  ORB_SLAM3::System *mpSLAM;
  ImuGrabber *mpImuGb;

  const bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

class MonoInertialNode : public rclcpp::Node
{
public:
    MonoInertialNode(const std::vector<std::string>& args)
    : Node("Mono_Inertial")
  {
    rclcpp::Logger logger = this->get_logger();


    bool bEqual = false;
    auto command_line_args = this->get_node_options().arguments();
	
    if (args.size() < 1)
    {
      std::cout<<command_line_args.size()<<std::endl;
      RCLCPP_ERROR(logger, "Usage: ros2 run your_package_name Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]");
      rclcpp::shutdown();
    }

	bEqual = true;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(args[1], args[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    imugb = new ImuGrabber();
    igb = new ImageGrabber(&SLAM, imugb, bEqual);

    // Maximum delay, 5 seconds
    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 5000, std::bind(&ImuGrabber::GrabImu, imugb, std::placeholders::_1));
    sub_img0 = this->create_subscription<sensor_msgs::msg::Image>(
        "/depth_camera/image_raw", 100, std::bind(&ImageGrabber::GrabImage, igb, std::placeholders::_1));

    sync_thread = std::thread(&ImageGrabber::SyncWithImu, igb);

    rclcpp::spin(this->get_node_base_interface());
  }

private:
  ORB_SLAM3::System *SLAM;
  ImuGrabber *imugb;
  ImageGrabber *igb;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img0;
  std::thread sync_thread;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<MonoInertialNode>(std::vector<std::string>(argv, argv + argc));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception &e)
  {
    std::cout<<"bridge exception"<< std::endl;
  }

  if (cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while (1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.sec;
      if (tIm > mpImuGb->imuBuf.back()->header.stamp.sec)
        continue;
      {
        this->mBufMutex.lock();
        im = GetImage(img0Buf.front());
        img0Buf.pop();
        this->mBufMutex.unlock();
      }

      std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if (!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.sec <= tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.sec;
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if (mbClahe)
        mClahe->apply(im, im);
	
      	mpSLAM->TrackMonocular(im, tIm, vImuMeas);
    	rclcpp::Time current_frame_time = img0Buf.front()->header.stamp;
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

