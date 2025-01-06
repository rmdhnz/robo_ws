#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class MainNode : public rclcpp::Node
{
public:
  MainNode() : Node("main_node"), x_(0.0), y_(0.0), theta_(0.0), vx_(0.0), vy_(0.0), omega_(0.0), manual_mode_(true)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MainNode::joy_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("joy_command", 10);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    wheel_twist_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "wheel_twist", 100, std::bind(&MainNode::wheel_twist_callback, this, std::placeholders::_1));
    euler_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "bno055/euler", 100, std::bind(&MainNode::euler_callback, this, std::placeholders::_1));
    icp_vel_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "icp_velocity", 10, std::bind(&MainNode::icp_vel_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons[0] == 1) { //tombol X
      manual_mode_ = true;
    } else if (msg->buttons[1] == 1) { //tombol O untuk auto
      manual_mode_ = false;
    }

    if (manual_mode_) {
      vx_ = (std::abs(msg->axes[1]) > 0.2) ? msg->axes[1] * 20 : 0.0;
      vy_ = (std::abs(msg->axes[0]) > 0.2) ? msg->axes[0] * 20 : 0.0;
      omega_ = (std::abs(msg->axes[3]) > 0.2) ? msg->axes[3] * 75 : 0.0;

      auto command_msg = std_msgs::msg::String();
      command_msg.data = std::to_string(vx_) + " " + std::to_string(vy_) + " " + std::to_string(omega_);
      publisher_->publish(command_msg);
    }
  }

  void icp_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!manual_mode_) {
      vx_ = msg->data[0] * 150;
      vy_ = msg->data[1] * 150;
      omega_ = msg->data[2] * 150;

      auto command_msg = std_msgs::msg::String();
      command_msg.data = std::to_string(vx_) + " " + std::to_string(vy_) + " " + std::to_string(omega_);
      publisher_->publish(command_msg);
    }
  }

  void wheel_twist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    //rpm to rad/s
    float wheelLR = msg->data[2] * 2 * M_PI / 60;
    float wheelUR = msg->data[3] * 2 * M_PI / 60;
    float wheelLL = msg->data[1] * 2 * M_PI / 60;
    float wheelUL = msg->data[0] * 2 * M_PI / 60;

    // m/s
    vx_local_ = -0.05*0.01*(wheelLL - wheelUR);
    vy_local_ = -0.05*0.01*(wheelUL - wheelLR);

    const float threshold = 0.1; //gara2 kalo diem ga nol banget

    if (fabs(vx_local_) <= threshold) {
        vx_local_ = 0.0;
    }

    if (fabs(vy_local_) <= threshold) {
        vy_local_ = 0.0;
    }
    
    x_local_ += (vx_local_ * cosf(0.785398) + vy_local_ * sinf(0.785398));
    y_local_ += (-vx_local_ * sinf(0.785398) + vy_local_ * cosf(0.785398));

    // printf("vx: %.2f, vy: %.2f, x: %.2f, y: %.2f, th: %.2f\n", vx_local_, vy_local_, x_local_, y_local_, theta_);
  }

  void euler_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    theta_ = msg->x * 0.0174533; // Convert yaw from degrees to radians

    x_ = (x_local_ * cosf(theta_) + y_local_ * sinf(theta_)) * 0.01;
    y_ = (-x_local_ * sinf(theta_) + y_local_ * cosf(theta_)) * 0.01; //global

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = theta_;

    odom_pub->publish(odom);

    printf("global x: %.4f, y: %.4f, th: %.4f\n", x_, y_, theta_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_twist_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr euler_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr icp_vel_subscription_;
  float x_;
  float y_;
  float theta_;
  float vx_;
  float vy_;
  float vx_local_;
  float vy_local_;
  float x_local_;
  float y_local_;
  float omega_;
  bool manual_mode_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainNode>());
  rclcpp::shutdown();
  return 0;
}