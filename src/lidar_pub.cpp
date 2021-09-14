#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rs_driver/api/lidar_driver.h>
#include <rs_driver/msg/point_cloud_msg.h>
#include <iostream>
#include <deque>
#include <string>
#include <ctime>
#include <math.h>
#include <assert.h>

#include "rs_ros2_lidar/visibility_control.h"


using namespace std::chrono_literals;
using namespace robosense::lidar;


namespace rs_lidar {
class LIDARPub : public rclcpp::Node
{
public:
  RS_LIDAR_PUBLIC
  explicit LIDARPub(const rclcpp::NodeOptions &options)
  : Node("lidar_pub", options)
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();

    RCLCPP_INFO(this->get_logger(), "RS_Driver Core Version: V %d.%d.%d",
        RSLIDAR_VERSION_MAJOR,
        RSLIDAR_VERSION_MINOR,
        RSLIDAR_VERSION_PATCH);

    // initialise lidar
    RSDriverParam param;                  ///< Creat a parameter object
    param.input_param.msop_port = 6699;   ///< Set the lidar msop port number the default 6699
    param.input_param.difop_port = 7788;  ///< Set the lidar difop port number the default 7788
    param.lidar_type = LidarType::RS16;   ///< Set the lidar type. Make sure this type is correct!

    RCLCPP_INFO (this->get_logger(),"RSDriverParam  angle_path: %s frame_id: %s lidar_type: %s",
        param.angle_path.c_str(), param.frame_id.c_str(), param.lidarTypeToStr(param.lidar_type).c_str()
    );
    RCLCPP_INFO (this->get_logger(),"RSInputParam RS16 device_ip: %s msop_port: %d, difop_port: %d",
        param.input_param.device_ip.c_str() ,param.input_param.msop_port, param.input_param.difop_port
    );
    RCLCPP_INFO (this->get_logger(),"RSDecoderParam max_distance: %g min_distance: %f start_angle: %f end_angle: %f split_frame_mode: %d num_pkts_split: %d cut_angle: %f",
        param.decoder_param.max_distance, param.decoder_param.min_distance,param.decoder_param.start_angle, param.decoder_param.end_angle, param.decoder_param.split_frame_mode, param.decoder_param.num_pkts_split, param.decoder_param.cut_angle
    );
    auto f_exception_callback = std::bind(&LIDARPub::lidar_exception_callback, this, std::placeholders::_1);
    lidar_driver_.regExceptionCallback(f_exception_callback);

    auto f_pointcloud_callback = std::bind(&LIDARPub::lidar_point_cloud_callback, this, std::placeholders::_1);
    lidar_driver_.regRecvCallback(f_pointcloud_callback);

    if (!lidar_driver_.init(param))
    {
        RCLCPP_ERROR(this->get_logger(), "Lidar Driver Initialise Error ... doing nothing");
        return;
    }

    pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_point_cloud", qos);

    timer_ = this->create_wall_timer(30ms, std::bind(&LIDARPub::timer_callback, this));

    lidar_driver_.start();
  }


  RS_LIDAR_LOCAL
  ~LIDARPub() {
    lidar_driver_.stop();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;

  std::deque<PointCloudMsg<pcl::PointXYZI>> point_cloud_msg_queue_;
  LidarDriver<pcl::PointXYZI> lidar_driver_;
  RSDriverParam lidar_param_;


  /**
  * @description: The exception callback function. This function will be registered to lidar driver.
  * @param code The error code struct.
  */
  RS_LIDAR_LOCAL
  void lidar_exception_callback(const Error& code) {
      RCLCPP_ERROR(this->get_logger(),"Lidar Error Code: %s", code.toString().c_str());
  }

  /**
  * @description: The point cloud callback function. This funciton will be registered to lidar driver.
  *              When the point cloud message is ready, driver can send out message through this function.
  * @param msg  The lidar point cloud message.
  */
  RS_LIDAR_LOCAL
  void lidar_point_cloud_callback(const PointCloudMsg<pcl::PointXYZI>& msg) {
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the message and process it in another thread is recommended*/
    RCLCPP_INFO_ONCE(this->get_logger(),"queuing first PointcloudMSG ...");
    // RCLCPP_INFO(this->get_logger(),"Lidar msg: %d pointcloud size: %d", msg.seq, msg.pointcloud_ptr->size());
    point_cloud_msg_queue_.push_back(std::move(msg));
  }

  RS_LIDAR_LOCAL
  void timer_callback() {
    // if no point cloud messages then we have nothing to do
    if (point_cloud_msg_queue_.size() == 0) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for first pointcloud from lidar...");
        return;
    }

    while (point_cloud_msg_queue_.size() > 0) {
        auto msg = point_cloud_msg_queue_[0];
        publish_point_cloud_from_msg(msg);
        point_cloud_msg_queue_.pop_front();
    }
  }

  RS_LIDAR_LOCAL
  void publish_point_cloud_from_msg(const PointCloudMsg<pcl::PointXYZI> &msg) {

    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    // pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    // TODO this isnt converting through
    // point_cloud.header.stamp = pcl_conversions::toPCL(rclcpp::Clock().now());
    point_cloud.header.seq = msg.seq;
    point_cloud.header.frame_id = msg.frame_id;
    point_cloud.width = msg.width;
    point_cloud.height = msg.height;
    point_cloud.is_dense = msg.is_dense;

    size_t n_points = msg.point_cloud_ptr->size();

    RCLCPP_INFO_ONCE(this->get_logger(), "publish point cloud height: %d width: %d size: %ld",
      point_cloud.height, point_cloud.width, n_points
    );

    point_cloud.points.resize(n_points);
    // for (size_t i = 0; i < n_points; i++) {
    //     point_cloud.points.at(i) = msg.pointcloud_ptr->at(i);

    //     // pcl::PointXYZI pt_lidar = msg.pointcloud_ptr->at(i);

    //     // auto pt_out = pcl::PointXYZRGB(pt_lidar.intensity, pt_lidar.x, pt_lidar.y);
    //     // pt_out.x = pt_lidar.x;
    //     // pt_out.y = pt_lidar.y;
    //     // pt_out.z = pt_lidar.z;

    //     // point_cloud.points.at(i) = pt_out;
    // }
    memcpy(point_cloud.points.data(), msg.point_cloud_ptr->data(), n_points * sizeof(pcl::PointXYZI));

    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(point_cloud, *pc2_msg);
    pc2_msg->header.stamp = rclcpp::Clock().now();
    this->pc2_pub_->publish(*pc2_msg);
  }
};

} // end namespace car_capture

RCLCPP_COMPONENTS_REGISTER_NODE(rs_lidar::LIDARPub)
