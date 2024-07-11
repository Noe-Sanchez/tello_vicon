#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <iostream>
#include "tello.h"
#include "stdint.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TelloWrapper : public rclcpp::Node{
  public:
    TelloWrapper(): Node("low_wrapper_node"){
      // Subscribers
      control_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/control/uaux", 10, std::bind(&TelloWrapper::control_input_callback, this, std::placeholders::_1));

      // Publishers
      battery_publisher = this->create_publisher<std_msgs::msg::Int32>("tello/battery", 10);
 
      control_timer = this->create_wall_timer(10ms, std::bind(&TelloWrapper::control_callback, this));
      battery_timer = this->create_wall_timer(1000ms, std::bind(&TelloWrapper::battery_callback, this));      

    }


    void control_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      control = *msg;
    }

    void battery_callback(){
      battery.data = (int32_t)Tello.battery_percentage;
      battery_publisher->publish(battery);
    }
    
    void control_callback(){
        Tello.right_x = control.linear.x/100;
        Tello.right_y = control.linear.y/100;
        Tello.left_x = control.angular.z/100;
        Tello.left_y = control.linear.z/100;
    }


  private:

    std_msgs::msg::String enable;
    std_msgs::msg::Int32 battery;
    geometry_msgs::msg::Twist control;

    rclcpp::TimerBase::SharedPtr estimator_timer;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::String>::SharedPtr enable_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher;

    tello Tello;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelloWrapper>());
  rclcpp::shutdown();
  return 0;
}
