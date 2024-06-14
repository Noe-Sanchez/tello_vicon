#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class PidController : public rclcpp::Node{
  public:
    PidController(): Node("pid_node"){
      // Subscribers
      vicon_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vicon/tello2/tello2", 10, std::bind(&PidController::vicon_callback, this, std::placeholders::_1));
      position_reference_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/reference/pose", 10, std::bind(&PidController::position_reference_callback, this, std::placeholders::_1));

      // Publishers
      uaux_publisher = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);
      error_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/error", 10);

      // Make 0.1s timer
      control_timer = this->create_wall_timer(100ms, std::bind(&PidController::control_callback, this));
      
    }

    void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      vicon_pose = *msg;
    }
    void position_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){ 
      reference_pose = *msg;
    }

    void control_callback(){
      // Error definition
      _error.pose.position.x = reference_pose.pose.position.x - vicon_pose.pose.position.x;
      _error.pose.position.y = reference_pose.pose.position.y - vicon_pose.pose.position.y;
      _error.pose.position.z = reference_pose.pose.position.z - vicon_pose.pose.position.z;

      // Control law
      uaux.linear.x = kpx * _error.pose.position.x;
      uaux.linear.y = kpy * _error.pose.position.y;
      uaux.linear.z = kpz * _error.pose.position.z;

      // Publish control law
      uaux_publisher->publish(uaux);
      error_publisher->publish(_error);

    }

  private:

    geometry_msgs::msg::PoseStamped vicon_pose; 
    geometry_msgs::msg::PoseStamped reference_pose;
    geometry_msgs::msg::Twist uaux;
    geometry_msgs::msg::PoseStamped _error;

    double kpx = 50.0;
    double kpy = 50.0;
    double kpz = 50.0;

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_reference_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;

    double sigma;
    double sigma_dot;
    double lambda;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidController>());
  rclcpp::shutdown();
  return 0;
}
