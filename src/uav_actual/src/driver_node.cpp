#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Aria/Aria.h>  // Include the AriaCoda library

class PioneerDriver : public rclcpp::Node {
public:
  PioneerDriver() : Node("driver_node"), robot(nullptr) {
    // Initialize Aria
    Aria::init();
    
    // Create and configure the robot object
    robot = new ArRobot();
    
    // Open connection to the robot on serial port (update if needed)
    if (!robot->openPort("/dev/ttyUSB0")) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open connection to Pioneer on /dev/ttyUSB0");
      Aria::exit(1);
    }

    // Enable motors and run robot processing loop in background
    robot->runAsync(true);
    robot->enableMotors();
    
    // Subscribe to /cmd_vel for movement commands
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&PioneerDriver::cmdVelCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Pioneer driver node initialized using AriaCoda.");
  }

  ~PioneerDriver() {
    if (robot) {
      robot->stop();
      delete robot;
    }
    Aria::exit();
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    robot->setVel(linear * 1000.0);  // Convert to mm/sec (if necessary)
    robot->setRotVel(angular * 180.0 / 3.14159);  // Convert to degrees/sec

    RCLCPP_INFO(this->get_logger(), "Command: linear=%.2f m/s, angular=%.2f rad/s", linear, angular);
  }

  ArRobot* robot;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PioneerDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}