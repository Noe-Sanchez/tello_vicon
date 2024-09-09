#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

double sig1(double x, double exponent){
  double s;

  if (x > 0){
    s = 1;
  } else if (x < 0){
    s = -1;
  } else {
    s = 0;
  }

  s = pow(abs(x), exponent) * s;

  return s;
} 


Eigen::Vector3d sig3(Eigen::Vector3d v, double exponent){
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

Eigen::Vector4d sig4(Eigen::Vector4d v, double exponent){
	Eigen::Vector4d s;

	for (int i = 0; i < 4; i++){
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


Eigen::Vector4d sig4(Eigen::Vector4d v, Eigen::Vector4d exponent){
  Eigen::Vector4d s;

  for (int i = 0; i < 4; i++){
    if (v(i) > 0){
      s(i) = 1;
    } else if (v(i) < 0){
      s(i) = -1;
    } else {
      s(i) = 0;
    }
    
    s(i) = pow(abs(v(i)), exponent(i)) * s(i);
  }

  return s;
}

Eigen::Vector4d ewise(Eigen::Vector4d v1, Eigen::Vector4d v2){
  Eigen::Vector4d v;

  for (int i = 0; i < 4; i++){
    v(i) = v1(i) * v2(i);
  }

  return v;
}

Eigen::Vector4d sqrt4(Eigen::Vector4d v){
  Eigen::Vector4d s;

  for (int i = 0; i < 4; i++){
    s(i) = sqrt(v(i));
  }

  return s;
}

Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::Vector4d p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  /*q_matrix << q.w(), -q.x(), -q.y(), -q.z(),
	      q.x(), q.w(), -q.z(), q.y(),
	      q.y(), q.z(), q.w(), -q.x(),
	      q.z(), -q.y(), q.x(), q.w();
  */

  q_matrix << q(0), -q(1), -q(2), -q(3),
	      q(1),  q(0), -q(3),  q(2),
	      q(2),  q(3),  q(0), -q(1),
	      q(3), -q(2),  q(1),  q(0);
	      
  p_vector = q_matrix * p;
  
  return p_vector;
}

Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::Vector3d p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  p_vector << 0, p(0), p(1), p(2);

  q_matrix << q(0), -q(1), -q(2), -q(3),
	      q(1),  q(0), -q(3),  q(2),
	      q(2),  q(3),  q(0), -q(1),
	      q(3), -q(2),  q(1),  q(0);
	      
  p_vector = q_matrix * p_vector;
  
  return p_vector;
}

Eigen::Vector4d qlm(Eigen::Vector4d q){
  Eigen::Vector4d q_log;

  double norm = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3));

  if (norm == 0){
    q_log << 0, 0, 0, 0;
  } else {
    // Use arccos to get the angle
    q_log << 0, q(1)/norm, q(2)/norm, q(3)/norm; 
    q_log = acos(q(0)) * q_log;
  }

  return 2*q_log;
}

Eigen::Vector4d exp4(Eigen::Vector4d v, double exponent){
  Eigen::Vector4d v_exp;

  for (int i = 0; i < 4; i++){
    v_exp(i) = pow(v(i), exponent);
  }

  return v_exp;
}

class PidController : public rclcpp::Node{
  public:
    PidController(): Node("pid_node"){
      // Subscribers
      estimator_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/estimator/pose", 10, std::bind(&PidController::estimator_pose_callback, this, std::placeholders::_1));
      estimator_velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/estimator/velocity", 10, std::bind(&PidController::estimator_velocity_callback, this, std::placeholders::_1));
      reference_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/reference/pose", 10, std::bind(&PidController::position_reference_callback, this, std::placeholders::_1));
      reference_velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/reference/velocity", 10, std::bind(&PidController::velocity_reference_callback, this, std::placeholders::_1));

      // Publishers
      uaux_publisher  = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);
      error_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/error", 10);
      ref_rot_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/ref_rot", 10);

      // Make 0.5s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&PidController::control_callback, this));
  
      // Initialize variables
      kp << 0.2, 0.2, 0.2, 0.2;
      kd << 0.1, 0.1, 0.1, 0.1;
      ki << 0.01, 0.01, 0.01, 0.01;

      e << 0, 0, 0, 0;
      ref_rot << 0, 0, 0, 0;
      e_dot << 0, 0, 0, 0;
      e_int << 0, 0, 0, 0;
      q_hat << 1, 0, 0, 0;
      q_hat_conj << 1, 0, 0, 0;
      q_d << 1, 0, 0, 0;
      q_e << 1, 0, 0, 0;
      eta_e << 0, 0, 0, 0;
      uaux << 0, 0, 0, 0;

    }

    void estimator_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      estimator_pose = *msg;
    }
    void estimator_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      estimator_velocity = *msg;
    }
    void position_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      reference_pose = *msg;
    }
    void velocity_reference_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      reference_velocity = *msg;
    }


    void control_callback(){
      // Conjugate q_hat
      q_hat << estimator_pose.pose.orientation.w, estimator_pose.pose.orientation.x, estimator_pose.pose.orientation.y, estimator_pose.pose.orientation.z;
      q_hat_conj << q_hat(0), -q_hat(1), -q_hat(2), -q_hat(3);

      q_d << reference_pose.pose.orientation.w, reference_pose.pose.orientation.x, reference_pose.pose.orientation.y, reference_pose.pose.orientation.z;
      q_e = kronecker(q_hat_conj, q_d);

      // Normalize q_e
      double q_e_norm = sqrt(q_e(1)*q_e(1) + q_e(2)*q_e(2) + q_e(3)*q_e(3));
      if (q_e_norm != 0){
	q_e << q_e(0), q_e(1)/q_e_norm, q_e(2)/q_e_norm, q_e(3)/q_e_norm;
      }else{
	q_e << 1, 0, 0, 0;
      }

      // Logarithmic mapping of q_e (0, eta_e_phi, eta_e_theta, eta_e_psi)
      eta_e << qlm(q_e);

      // Check for NaN in eta_e
      for (int i = 0; i < 4; i++){
        if (std::isnan(eta_e(i))){
          eta_e(i) = 0;
        }
      }

      e << reference_pose.pose.position.x - estimator_pose.pose.position.x,
           reference_pose.pose.position.y - estimator_pose.pose.position.y,
           reference_pose.pose.position.z - estimator_pose.pose.position.z,
	         eta_e(3);
      
      e_dot << estimator_velocity.linear.x  - reference_velocity.linear.x,
	             estimator_velocity.linear.y  - reference_velocity.linear.y,
	             estimator_velocity.linear.z  - reference_velocity.linear.z,
	             estimator_velocity.angular.z - reference_velocity.angular.z;

      e_int += e * 0.01;

      // Control law using ewise
      uaux << ewise(kp, e) + ewise(kd, e_dot) + ewise(ki, e_int);
      
      // Saturate control output
      uaux(0) = std::min(std::max(uaux(0), -1.6), 1.6);
      uaux(1) = std::min(std::max(uaux(1), -1.6), 1.6);
      uaux(2) = std::min(std::max(uaux(2), -0.9), 1.0);
      uaux(3) = std::min(std::max(uaux(3), -1.0), 1.0);

      // Normalize control output
      uaux(0) = 200 * (uaux(0) + 1.6)/(3.2) - 100;
      uaux(1) = 200 * (uaux(1) + 1.6)/(3.2) - 100;
      uaux(2) = 200 * (uaux(2) + 0.9)/(1.9) - 100;
      uaux(3) = 200 * (uaux(3) + 1.0)/(2.0) - 100;

      // Rotate control output in x and y
      Eigen::Vector4d uaux_rot;
      uaux_rot << 0, uaux(0), uaux(1), 0;
      uaux_rot = kronecker(kronecker(q_hat_conj, uaux_rot), q_hat);

      uaux(0) = uaux_rot(1);
      uaux(1) = uaux_rot(2);

      _uaux.linear.x  = uaux(0);
      _uaux.linear.y  = uaux(1);
      _uaux.linear.z  = uaux(2);
      _uaux.angular.z = -uaux(3);

      _error.pose.position.x = e(0);
      _error.pose.position.y = e(1);
      _error.pose.position.z = e(2);
      _error.pose.orientation.w = e(3);

      _ref_rot.pose.position.x = ref_rot(1);
      _ref_rot.pose.position.y = ref_rot(2);
      _ref_rot.pose.position.z = ref_rot(3);
      _ref_rot.pose.orientation.w = reference_pose.pose.orientation.w;
      _ref_rot.pose.orientation.x = reference_pose.pose.orientation.x;
      _ref_rot.pose.orientation.y = reference_pose.pose.orientation.y;
      _ref_rot.pose.orientation.z = reference_pose.pose.orientation.z;
      _ref_rot.header.frame_id = "world";

      uaux_publisher->publish(_uaux);
      error_publisher->publish(_error);
      ref_rot_publisher->publish(_ref_rot);
      
    }

  private:

    geometry_msgs::msg::PoseStamped estimator_pose; 
    geometry_msgs::msg::PoseStamped reference_pose;
    geometry_msgs::msg::PoseStamped _error;
    geometry_msgs::msg::PoseStamped _ref_rot;
    geometry_msgs::msg::Twist estimator_velocity;
    geometry_msgs::msg::Twist reference_velocity;
    geometry_msgs::msg::Twist _uaux;

    Eigen::Vector4d e;
    Eigen::Vector4d e_int;
    Eigen::Vector4d ref_rot;
    Eigen::Vector4d e_dot;
    Eigen::Vector4d q_hat;
    Eigen::Vector4d q_hat_conj;
    Eigen::Vector4d q_d;
    Eigen::Vector4d q_e;
    Eigen::Vector4d eta_e;
    Eigen::Vector4d kp;
    Eigen::Vector4d kd;
    Eigen::Vector4d ki;
    Eigen::Vector4d uaux;

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr estimator_velocity_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reference_velocity_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_rot_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidController>());
  rclcpp::shutdown();
  return 0;
}
