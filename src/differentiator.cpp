#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

Eigen::Vector3d sig(Eigen::Vector3d v, double exponent){
  Eigen::Vector3d s;

  for (int i = 0; i < 3; i++){
    if (v(i) > 0){
      s(i) = 1;
    } else if (v(i) < 0){
      s(i) = -1;
    } else {
      s(i) = 0;
    }

    s(i) = pow(abs(v(i)), exponent) * s(i);
  } 

  return s;
}


Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::Vector4d p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  q_matrix << q.w(), -q.x(), -q.y(), -q.z(),
	      q.x(), q.w(), -q.z(), q.y(),
	      q.y(), q.z(), q.w(), -q.x(),
	      q.z(), -q.y(), q.x(), q.w();
	      
  p_vector = q_matrix * p;
  
  return p_vector;
}

Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::Vector3d p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  p_vector << 0, p(0), p(1), p(2);

  q_matrix << q.w(), -q.x(), -q.y(), -q.z(),
	      q.x(), q.w(), -q.z(), q.y(),
	      q.y(), q.z(), q.w(), -q.x(),
	      q.z(), -q.y(), q.x(), q.w();
	      
  p_vector = q_matrix * p_vector;
  
  return p_vector;
}


using namespace std::chrono_literals;

class Differentiator : public rclcpp::Node{
  public:
    Differentiator(): Node("pid_node"){
      // Subscribers
      vicon_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vicon/tello2/tello2", 10, std::bind(&Differentiator::vicon_callback, this, std::placeholders::_1));

      // Publishers
      estimation_position_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimator/pose", 10);
      estimation_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("estimator/velocity", 10);

      // Make 0.01s timer
      estimator_timer = this->create_wall_timer(10ms, std::bind(&Differentiator::estimator_callback, this));
      
      // Initialize variables
      Eigen::Vector4d identity;
      identity << 1, 0, 0, 0;
      q_hat = identity;
      q_hat_dot = identity;
      q_tilde = identity;
      q_vicon = identity;
      q_tilde_conj = identity;
      e_hat = Eigen::Vector3d::Zero();
      w_hat = Eigen::Vector3d::Zero();
      w_hat_dot = Eigen::Vector3d::Zero();

    }

    void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      vicon_pose = *msg;
    }

    void estimator_callback(){
      q_vicon << vicon_pose.pose.orientation.w,
	         vicon_pose.pose.orientation.x,
		 vicon_pose.pose.orientation.y,
		 vicon_pose.pose.orientation.z;

      // q_tilde definition
      q_tilde = kronecker(q_hat, q_vicon);

      // e_hat definition
      e_hat << 2*q_tilde(1)/q_tilde(0),
	       2*q_tilde(2)/q_tilde(0),
	       2*q_tilde(3)/q_tilde(0);

      // w_hat_dot calculation
      w_hat_dot << epsilon2*sig(e_hat, alpha) + mu2*sig(e_hat, beta);

      // Integrate w_hat_dot to get w_hat
      w_hat += w_hat_dot * 0.01;

      // q_hat_dot calculation
      w_aux = epsilon1 * sig(e_hat, (alpha+1)/2) + mu1 * sig(e_hat, (beta+1)/2);
      w_aux = w_aux + w_hat;
      
      q_tilde_conj << q_tilde(0), -q_tilde(1), -q_tilde(2), -q_tilde(3); 
      q_hat_dot = 0.5 * kronecker(q_hat, kronecker(q_tilde_conj, kronecker(q_w_aux, q_tilde))); 

      // Integrate q_hat_dot to get q_hat
      q_hat += q_hat_dot * 0.01;
      
      estimated_pose.pose.orientation.w = q_hat(0);
      estimated_pose.pose.orientation.x = q_hat(1);
      estimated_pose.pose.orientation.y = q_hat(2);
      estimated_pose.pose.orientation.z = q_hat(3);

      estimated_velocity.linear.x = w_hat(0);
      estimated_velocity.linear.y = w_hat(1);
      estimated_velocity.linear.z = w_hat(2);

      // Publish estimated position and velocity
      estimation_position_publisher->publish(estimated_pose);
      estimation_velocity_publisher->publish(estimated_velocity);
    }

  private:

    geometry_msgs::msg::PoseStamped vicon_pose; 
    geometry_msgs::msg::PoseStamped estimated_pose;
    geometry_msgs::msg::Twist estimated_velocity;
   
    Eigen::Vector4d q_hat;
    Eigen::Vector4d q_hat_dot;
    Eigen::Vector4d q_w_aux;
    Eigen::Vector4d q_tilde;
    Eigen::Vector4d q_vicon;
    Eigen::Vector4d q_tilde_conj;
    Eigen::Vector3d e_hat;
    Eigen::Vector3d w_hat;
    Eigen::Vector3d w_hat_dot;
    Eigen::Vector3d w_aux;
    double epsilon1 = 5;
    double epsilon2 = 6;
    double mu1 = 5;
    double mu2 = 6;
    double alpha = 0.6;
    double beta = 1.2;

    rclcpp::TimerBase::SharedPtr estimator_timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimation_position_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr estimation_velocity_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Differentiator>());
  rclcpp::shutdown();
  return 0;
}
