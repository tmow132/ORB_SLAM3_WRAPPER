#include "mono-slam-node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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
#include "Settings.h"
using std::placeholders::_1;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>
        map_points_pub;
image_transport::Publisher rendered_image_pub;

std::string map_frame_id, pose_frame_id;

// Coordinate transformation matrix from orb coordinate system to ros coordinate
// system
tf2::Matrix3x3 tf_orb_to_ros(0, 0, 1, -1, 0, 0, 0, -1, 0);




sensor_msgs::msg::PointCloud2
tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points,
                                rclcpp::Time current_frame_time) {
    const int num_channels = 3; // x y z

    if (map_points.size() == 0) {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    int j =0;
    cloud.header.stamp = current_frame_time;
    cloud.header.frame_id = map_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points[i]) {

            tf2::Vector3 point_translation(map_points[i]->GetWorldPos()(0),
                                           map_points[i]->GetWorldPos()(1),
                                           map_points[i]->GetWorldPos()(2));

            point_translation = tf_orb_to_ros * point_translation;

            float data_array[num_channels] = {
                    point_translation.x(), point_translation.y(), point_translation.z()};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
                   num_channels * sizeof(float));
        }
    }
    j++;
    return cloud;
}
void publish_ros_pose_tf(rclcpp::Node &node,
                        cv::Mat Tcw, rclcpp::Time current_frame_time) {
 if (!Tcw.empty()) {
   tf2::Transform tf_transform =
       from_orb_to_ros_tf_transform(Tcw);

   publish_tf_transform(node, tf_transform, current_frame_time);

   publish_pose_stamped(tf_transform, current_frame_time);
 }
}

void publish_tf_transform(rclcpp::Node &node, tf2::Transform tf_transform,
                         rclcpp::Time current_frame_time) {
 static tf2_ros::TransformBroadcaster tf_broadcaster(node);

 std_msgs::msg::Header header;
 header.stamp = current_frame_time;
 header.frame_id = map_frame_id;

 geometry_msgs::msg::TransformStamped tf_msg;
 tf_msg.header = header;
 tf_msg.child_frame_id = pose_frame_id;
 tf_msg.transform = tf2::toMsg(tf_transform);

 tf_broadcaster.sendTransform(tf_msg);
}

void publish_pose_stamped(tf2::Transform tf_transform, rclcpp::Time current_frame_time) {
 std_msgs::msg::Header header;
 header.stamp = current_frame_time;
 header.frame_id = pose_frame_id;

 geometry_msgs::msg::Pose pose;
 tf2::toMsg(tf_transform, pose);

 geometry_msgs::msg::PoseStamped pose_msg;
 pose_msg.header = header;
 pose_msg.pose = pose;

 pose_pub->publish(pose_msg);
}

void setup_tf_orb_to_ros(ORB_SLAM3::System::eSensor sensor_type) {
 // The conversion depends on whether IMU is involved:
 //  z is aligned with camera's z axis = without IMU
 //  z is aligned with gravity = with IMU

   tf_orb_to_ros.setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);;
 
}

tf2::Transform
from_orb_to_ros_tf_transform(cv::Mat transformation_mat) {
 cv::Mat orb_rotation(3, 3, CV_32F);
 cv::Mat orb_translation(3, 1, CV_32F);

 orb_rotation = transformation_mat.rowRange(0, 3).colRange(0, 3);
 orb_translation = transformation_mat.rowRange(0, 3).col(3);

 tf2::Matrix3x3 tf_camera_rotation(
     orb_rotation.at<float>(0, 0), orb_rotation.at<float>(0, 1),
     orb_rotation.at<float>(0, 2), orb_rotation.at<float>(1, 0),
     orb_rotation.at<float>(1, 1), orb_rotation.at<float>(1, 2),
     orb_rotation.at<float>(2, 0), orb_rotation.at<float>(2, 1),
     orb_rotation.at<float>(2, 2));

 tf2::Vector3 tf_camera_translation(orb_translation.at<float>(0),
                                                 orb_translation.at<float>(1),
                                                 orb_translation.at<float>(2));

 // cout << setprecision(9) << "Rotation: " << endl << orb_rotation << endl;
 // cout << setprecision(9) << "Translation xyz: " << orb_translation.at<float>
 // (0) << " " << orb_translation.at<float> (1) << " " <<
 // orb_translation.at<float> (2) << endl;

 // Transform from orb coordinate system to ros coordinate system on camera
 // coordinates
 tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
 tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

 // Inverse matrix
 tf_camera_rotation = tf_camera_rotation.transpose();
 tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

 // Transform from orb coordinate system to ros coordinate system on map
 // coordinates
 tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
 tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

 return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

void publish_ros_tracking_img(const cv::Mat &image, const rclcpp::Time &current_frame_time) {
    std_msgs::msg::Header header;
    int j=0;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    const std::shared_ptr<sensor_msgs::msg::Image> rendered_image_msg =
            cv_bridge::CvImage(header, "rgb8", image).toImageMsg();

    rendered_image_pub.publish(rendered_image_msg);
    j++;
}

void publish_ros_tracking_mappoints(
        std::vector<ORB_SLAM3::MapPoint *> map_points,
        const rclcpp::Time &current_frame_time) {
    sensor_msgs::msg::PointCloud2 cloud =
            tracked_mappoints_to_pointcloud(map_points, current_frame_time);

    map_points_pub->publish(cloud);
}

void setup_ros_publishers(rclcpp::Node &node) {
    pose_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/camera", 10);

    map_points_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("/points_mono", 1000);

  std::shared_ptr<rclcpp::Node> image_transport_node = rclcpp::Node::make_shared("image_publisher");
  image_transport::ImageTransport image_transport(image_transport_node);

  rendered_image_pub = image_transport.advertise("orbslam3/tracking_image", 5);
}


MonoSlamNode::MonoSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("orbslamm"),
    m_SLAM(pSLAM)
{
    //rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/camera/image_raw");
    //rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/depth_camera/image_raw");
   rgb_sub= this->create_subscription<ImageMsg>(
        "/camera/image_raw",
        10,
        std::bind(&MonoSlamNode::GrabImage, this, std::placeholders::_1));
    
	setup_ros_publishers(*this);
}

MonoSlamNode::~MonoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoSlamNode::GrabImage(const ImageMsg::SharedPtr msgRGB)
{
	 
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    //cerr<<"doneeee waiiiiiit"<<endl;
    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat((m_SLAM->TrackMonocular(cv_ptrRGB->image, cv_ptrRGB->header.stamp.sec)).matrix());
    rclcpp::Time current_frame_time = cv_ptrRGB->header.stamp;
    map_frame_id = "map";//cv_ptrRGB->header.frame_id;
    pose_frame_id = "odom";
    publish_ros_pose_tf(*this, Tcw, current_frame_time);
    publish_ros_tracking_mappoints(m_SLAM->GetTrackedMapPoints(), current_frame_time);
    publish_ros_tracking_img(Tcw,current_frame_time);
}


