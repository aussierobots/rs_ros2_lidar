#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <string>
#include "rs_ros2_lidar/point_cloud/render.h"

#include "rs_ros2_lidar/visibility_control.h"

using namespace std::chrono_literals;
using namespace std;

namespace rs_lidar {
class LIDARViz : public rclcpp::Node
{
public:
  RS_LIDAR_PUBLIC
  explicit LIDARViz(const rclcpp::NodeOptions &options)
  : Node("lidar_viz", options)
  {

    RCLCPP_INFO(this->get_logger(), "creating PCL visualiser ...");
    viewer_ = pcl::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer_);


    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();

    string pc_topic_param_name = "point_cloud_topic";
    string pc_topic_value = "/lidar_point_cloud";
    pc_topic_value = this->declare_parameter<string>(pc_topic_param_name, pc_topic_value);

    RCLCPP_INFO(this->get_logger(), "subscribing to topic: %s", pc_topic_value.c_str());

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic_value, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr pc2_msg) {
            RCLCPP_INFO_ONCE(this->get_logger(), "received PointCloud2 ... ");
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*pc2_msg, *inputCloudI);
            RCLCPP_INFO_ONCE(this->get_logger(), "performed first fromROSMsg ... ");

            viewer_->removeAllPointClouds();
            viewer_->removeAllShapes();
            renderPointCloud(this->viewer_, inputCloudI, pc2_msg->header.frame_id+to_string(pc_seq_));
            pc_seq_+=1;
            viewer_->spinOnce();
        });
  }

private:

    pcl::visualization::PCLVisualizer::Ptr viewer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr pc_sub_;
    uint16_t pc_seq_ = 0;

    RS_LIDAR_LOCAL
    //setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
    void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
    {

        viewer->setBackgroundColor (0, 0, 0);

        // set camera position and angle
        viewer->initCameraParameters();
        // distance away in meters
        int distance = 16;

        switch(setAngle)
        {
            case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
            case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
            case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
            case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
        }

        if(setAngle!=FPS)
            viewer->addCoordinateSystem (1.0);
    }
};
} // end namespace car_perception

RCLCPP_COMPONENTS_REGISTER_NODE(rs_lidar::LIDARViz)
