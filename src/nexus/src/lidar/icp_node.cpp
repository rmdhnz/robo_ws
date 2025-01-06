#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include <iostream>

class ICPNode : public rclcpp::Node {
public:
    ICPNode() : Node("icp_node"), move_state_(0), target_reached_(false) {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ICPNode::scanCallback, this, std::placeholders::_1));
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_output", 10);
        vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("icp_velocity", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ICPNode::moveRobot, this));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_.projectLaser(*scan_msg, cloud_msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        if (!previous_cloud_) {
            previous_cloud_ = cloud;
            return;
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(previous_cloud_);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        if (icp.hasConverged()) {
            transformation_ = icp.getFinalTransformation();
            target_reached_ = checkTargetReached(transformation_);

            float distance = std::sqrt(std::pow(transformation_(0, 3), 2) + std::pow(transformation_(1, 3), 2));
            std::cout << "Distance: " << distance << std::endl;
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(Final, output);
        output.header = cloud_msg.header;

        cloud_pub_->publish(output);

        previous_cloud_ = cloud;
    }

    bool checkTargetReached(const Eigen::Matrix4f& transformation) {
        float distance = std::sqrt(std::pow(transformation(0, 3), 2) + std::pow(transformation(1, 3), 2));
        return distance >= 0.2;
    }

    void moveRobot() {
        if (target_reached_) {
            move_state_ = (move_state_ + 1) % 4;
            target_reached_ = false;
        }

        auto message = std_msgs::msg::Float32MultiArray();
        switch (move_state_) {
            case 0:
                message.data = {0.1, 0.0, 0.0};
                break;
            case 1:
                message.data = {0.0, 0.1, 0.0}; 
                break;
            case 2:
                message.data = {-0.1, 0.0, 0.0};
                break;
            case 3:
                message.data = {0.0, -0.1, 0.0};
                break;
            default:
                move_state_ = 0;
                break;
        }
        vel_pub_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    laser_geometry::LaserProjection projector_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    Eigen::Matrix4f transformation_;
    int move_state_;
    bool target_reached_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}