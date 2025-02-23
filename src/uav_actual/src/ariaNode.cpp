/*  
*   A basic node for ros2 that runs with ariaCoda
*   To run use 'ros2 run ariaNode ariaNode -rp /dev/ttyUSB0'
*
*   Author: Kieran Quirke-Brown
*   Date: 12/01/2024
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "Aria/Aria.h"

bool stopRunning = false;  // Global variable for signal handling

using namespace std::chrono_literals;

class AriaNode : public rclcpp::Node {
public:
    AriaNode(float* forwardSpeed, float* rotationSpeed) 
        : Node("aria_node"), currentForwardSpeed(forwardSpeed), currentRotationSpeed(rotationSpeed) {
        
        cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&AriaNode::cmdVelCallback, this, std::placeholders::_1)
        );    
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        *currentForwardSpeed = msg->linear.x;
        *currentRotationSpeed = msg->angular.z;
        RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel: linear=%.2f, angular=%.2f", *currentForwardSpeed, *currentRotationSpeed);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    float* currentForwardSpeed;
    float* currentRotationSpeed;
};

// Signal handler function
void my_handler(int s) {
    printf("Caught signal %d, stopping robot...\n", s);
    stopRunning = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    Aria::init();

    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();

    ArRobot robot;
    ArRobotConnector robotConnector(&parser, &robot);

    signal(SIGINT, my_handler);

    // ✅ Corrected logging
    RCLCPP_DEBUG(rclcpp::get_logger("aria_node"), "Trying to connect to robot...");

    if (!robotConnector.connectRobot()) {
        RCLCPP_ERROR(rclcpp::get_logger("aria_node"), "Could not connect to the robot.");
        Aria::exit(1);
    }

    robot.setAbsoluteMaxTransVel(400);
    float forwardSpeed = 0.0, rotationSpeed = 0.0;

    RCLCPP_DEBUG(rclcpp::get_logger("aria_node"), "Running robot asynchronously...");
    robot.runAsync(true);
    robot.enableMotors();

    auto node = std::make_shared<AriaNode>(&forwardSpeed, &rotationSpeed);

    while (!stopRunning) {
        rclcpp::spin_some(node);
        robot.lock();
        robot.setVel(forwardSpeed * 500);
        robot.setRotVel(rotationSpeed * 50);
        robot.unlock();
        RCLCPP_DEBUG(rclcpp::get_logger("aria_node"), "Motor command sent: linear=%.2f, angular=%.2f", forwardSpeed, rotationSpeed);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("aria_node"), "Disabling motors and stopping robot...");
    robot.disableMotors();
    robot.stopRunning();
    robot.waitForRunExit();

    RCLCPP_DEBUG(rclcpp::get_logger("aria_node"), "Shutting down Aria.");
    Aria::exit(0);
    return 0;
}