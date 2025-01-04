#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include "mrsl_math.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2_ros/transform_broadcaster.h>

//Include Qos profiles
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

double euclidean2(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2){
  double distance = sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2));
  return distance;
}

double euclidean3(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2){
  double distance = sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2) + pow(p1.pose.position.z - p2.pose.position.z, 2));
  return distance;
}

class SimpleDynamics : public rclcpp::Node{
  public:
    SimpleDynamics(): Node("asmc_node"){
      // Subscribers
      reset_state_subscriber       = this->create_subscription<std_msgs::msg::Bool>("tello/control/reset_state", 10, std::bind(&SimpleDynamics::reset_state_callback, this, std::placeholders::_1));

      rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
      qos.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      qos.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);
      qos.keep_last(10);

      control_input_subscriber     = this->create_subscription<geometry_msgs::msg::Twist>("tello/control/uaux", 10, std::bind(&SimpleDynamics::control_input_callback, this, std::placeholders::_1));
      // Add qos profile to control input subscriber
      //control_input_subscriber     = this->create_subscription<geometry_msgs::msg::Twist>("tello/control/uaux", qos, std::bind(&SimpleDynamics::control_input_callback, this, std::placeholders::_1));

      // Publishers
      estimator_pose_publisher     = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/estimator/pose", 10);
      estimator_velocity_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("tello/estimator/velocity", 10);

      transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      // Make 0.5s timer
      dynamics_timer = this->create_wall_timer(10ms, std::bind(&SimpleDynamics::dynamics_callback, this));
  
      // Get drone id from launchfile parameter
      this->declare_parameter("drone_id", 0);
      this->get_parameter("drone_id", drone_id);

      // Initialize variables 
      q     << 1, 0, 0, 0;
      q_dot << 0, 0, 0, 0;
      p     << drone_id, 0, 1;
      p_dot << 0, 0, 0;

    }

    void reset_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
      if (msg->data){
        q     << 1, 0, 0, 0;
        q_dot << 0, 0, 0, 0;
        p     << 0, 0, 1;
        p_dot << 0, 0, 0;
      }
    }

    void control_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      //q_dot << 0, msg->angular.x, msg->angular.y, msg->angular.z;    
      //p_dot << msg->linear.x, msg->linear.y, msg->linear.z;
      //std::cout << msg->linear.z << std::endl;
      p_dot << 3.2 * (msg->linear.x + 100.0)/(200.0) - 1.6,
               3.2 * (msg->linear.y + 100.0)/(200.0) - 1.6,
               //1.9 * (msg->linear.z + 50.0)/(100.0) - 0.95;
               2 * (msg->linear.z + 100.0)/(200.0) - 1;
      q_dot << 0, 0, 0, 2.0 * (msg->angular.z + 100.0)/(200.0) - 1.0;
    }

    void dynamics_callback(){
      Eigen::Vector4d p_dot_rot;
      Eigen::Vector3d p_dot_rot3;
      Eigen::Vector4d q_dot_rot;
      Eigen::Vector4d q_conj;
      q_conj << q(0), -q(1), -q(2), -q(3);
      p_dot_rot = kronecker(kronecker(q, p_dot), q_conj);
      p_dot_rot3 << p_dot_rot(1), p_dot_rot(2), p_dot_rot(3);
      //q_dot_rot = kronecker(kronecker(q, q_dot), q_conj);
      //p = p + 0.01 * p_dot;
      p = p + 0.01 * p_dot_rot3;

      // Integral of quaternion
      q = q + 0.01 * (0.5 * kronecker(q, q_dot));
      // Normalize q
      q = q / q.norm();
      //q = q + 0.01 * (0.5 * kronecker(q, q_dot_rot));

      // Publish states
      pose.header.frame_id = "world";
      pose.header.stamp = this->now();
      pose.pose.position.x = p(0);
      pose.pose.position.y = p(1);
      pose.pose.position.z = p(2);
      pose.pose.orientation.w = q(0);
      pose.pose.orientation.x = q(1);
      pose.pose.orientation.y = q(2);
      pose.pose.orientation.z = q(3);

      transform.header.frame_id = "world";
      transform.header.stamp = this->now();
      //transform.child_frame_id = "tello";
      transform.child_frame_id = "tello" + std::to_string(drone_id);
      transform.transform.translation.x = p(0);
      transform.transform.translation.y = p(1);
      transform.transform.translation.z = p(2);
      transform.transform.rotation.w = q(0);
      transform.transform.rotation.x = q(1);
      transform.transform.rotation.y = q(2);
      transform.transform.rotation.z = q(3);

      //pose_dot.header.frame_id = "tello";
      pose_dot.header.frame_id = "tello" + std::to_string(drone_id);
      pose_dot.header.stamp = this->now();
      //pose_dot.twist.linear.x = p_dot(0);
      //pose_dot.twist.linear.y = p_dot(1);
      //pose_dot.twist.linear.z = p_dot(2);
      pose_dot.twist.linear.x = p_dot_rot(1);
      pose_dot.twist.linear.y = p_dot_rot(2);
      pose_dot.twist.linear.z = p_dot_rot(3);
      pose_dot.twist.angular.x = q_dot(1);
      pose_dot.twist.angular.y = q_dot(2);
      pose_dot.twist.angular.z = q_dot(3);

      estimator_pose_publisher->publish(pose);
      estimator_velocity_publisher->publish(pose_dot);
      transform_broadcaster->sendTransform(transform);

    }

  private:
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::TwistStamped pose_dot;
    geometry_msgs::msg::TransformStamped transform;

    Eigen::Vector4d q;
    Eigen::Vector4d q_dot;
    Eigen::Vector3d p;
    Eigen::Vector3d p_dot;

    rclcpp::TimerBase::SharedPtr dynamics_timer; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_input_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr estimator_velocity_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;

    int drone_id;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleDynamics>());
  rclcpp::shutdown();
  return 0;
}
