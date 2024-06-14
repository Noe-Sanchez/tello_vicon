#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Pid2Controller : public rclcpp::Node{
  public:
    Pid2Controller(): Node("pid2_node"){
      // Subscribers
      vicon_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vicon/tello2/tello2", 10, std::bind(&Pid2Controller::vicon_callback, this, std::placeholders::_1));
      position_reference_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/reference/pose", 10, std::bind(&Pid2Controller::position_reference_callback, this, std::placeholders::_1));
      velocity_reference_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/reference/velocity", 10, std::bind(&Pid2Controller::velocity_reference_callback, this, std::placeholders::_1));
      estimator_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/estimator/pose", 10, std::bind(&Pid2Controller::estimator_pose_callback, this, std::placeholders::_1));
      estimator_velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/estimator/velocity", 10, std::bind(&Pid2Controller::estimator_velocity_callback, this, std::placeholders::_1));    

      // Publishers
      uaux_publisher = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);
      error_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/error", 10);

      // Make 0.1s timer
      control_timer = this->create_wall_timer(100ms, std::bind(&Pid2Controller::control_callback, this));
      
    }

    void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      vicon_pose = *msg;
    }
    void position_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){ 
      reference_pose = *msg;
    }

    void velocity_reference_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      reference_velocity = *msg;
    }
    void estimator_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      estimator_pose = *msg;
    }
    void estimator_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      estimator_velocity = *msg;
    }

    void control_callback(){
      // Error definition
      _error.pose.position.x = reference_pose.pose.position.x - estimator_pose.pose.position.x;
      _error.pose.position.y = reference_pose.pose.position.y - estimator_pose.pose.position.y;
      _error.pose.position.z = reference_pose.pose.position.z - estimator_pose.pose.position.z;

      // Error velocity definition
      _error_velocity.linear.x = reference_velocity.linear.x - estimator_velocity.linear.x;
      _error_velocity.linear.y = reference_velocity.linear.y - estimator_velocity.linear.y;
      _error_velocity.linear.z = reference_velocity.linear.z - estimator_velocity.linear.z;

      _error_integral.linear.x += _error.pose.position.x * 0.1;
      _error_integral.linear.y += _error.pose.position.y * 0.1;
      _error_integral.linear.z += _error.pose.position.z * 0.1;

      // Control law
      uaux.linear.x = kpx * _error.pose.position.x + kvx * _error_velocity.linear.x + kix * _error_integral.linear.x + reference_velocity.linear.x;
      uaux.linear.y = kpy * _error.pose.position.y + kvy * _error_velocity.linear.y + kiy * _error_integral.linear.y + reference_velocity.linear.y;
      uaux.linear.z = kpz * _error.pose.position.z + kvz * _error_velocity.linear.z + kiz * _error_integral.linear.z + reference_velocity.linear.z;

      // Publish control law
      uaux_publisher->publish(uaux);
      error_publisher->publish(_error);

    }

  private:

    geometry_msgs::msg::PoseStamped vicon_pose; 
    geometry_msgs::msg::PoseStamped reference_pose;
    geometry_msgs::msg::PoseStamped estimator_pose;
    geometry_msgs::msg::Twist reference_velocity;
    geometry_msgs::msg::Twist estimator_velocity;
    geometry_msgs::msg::Twist uaux;
    geometry_msgs::msg::PoseStamped _error;
    geometry_msgs::msg::Twist _error_velocity;
    geometry_msgs::msg::Twist _error_integral;

    double kpx = 50.0;
    double kpy = 50.0;
    double kpz = 50.0;

    double kvx = 20.0;
    double kvy = 20.0;
    double kvz = 20.0;

    double kix = 10.0;
    double kiy = 10.0;
    double kiz = 10.0;

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_reference_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_reference_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr estimator_velocity_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pid2Controller>());
  rclcpp::shutdown();
  return 0;
}
