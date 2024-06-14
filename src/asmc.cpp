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

  return q_log;
}

Eigen::Vector4d exp4(Eigen::Vector4d v, double exponent){
  Eigen::Vector4d v_exp;

  for (int i = 0; i < 4; i++){
    v_exp(i) = pow(v(i), exponent);
  }

  return v_exp;
}

class AsmcController : public rclcpp::Node{
  public:
    AsmcController(): Node("asmc_node"){
      // Subscribers
      estimator_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/estimator/pose", 10, std::bind(&AsmcController::estimator_pose_callback, this, std::placeholders::_1));
      estimator_velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/estimator/velocity", 10, std::bind(&AsmcController::estimator_velocity_callback, this, std::placeholders::_1));
      reference_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/reference/pose", 10, std::bind(&AsmcController::position_reference_callback, this, std::placeholders::_1));
      reference_velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/reference/velocity", 10, std::bind(&AsmcController::velocity_reference_callback, this, std::placeholders::_1));

      // Publishers
      uaux_publisher  = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);
      sigma_publisher = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/sigma", 10);
      error_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/error", 10);

      // Make 0.1s timer
      control_timer = this->create_wall_timer(100ms, std::bind(&AsmcController::control_callback, this));
  
      // Initialize variables
      zetta1 << 3, 3, 1.5, 2;
      zetta2 << 1, 1, 1.25, 1;
      lambda1 << 2.5, 2.5, 1.75, 2;
      lambda2 << 1.25, 1.25, 1.25, 1.2;
      alpha << 1.25, 1.25, 1.25, 1.25;
      beta << 1.5, 1.5, 0.75, 1.2;

      e << 0, 0, 0, 0;
      ref_rot << 0, 0, 0, 0;
      e_dot << 0, 0, 0, 0;
      q_hat << 1, 0, 0, 0;
      q_hat_conj << 1, 0, 0, 0;
      q_d << 1, 0, 0, 0;
      q_e << 1, 0, 0, 0;
      eta_e << 0, 0, 0, 0;
      K << 0, 0, 0, 0;
      K_dot << 0, 0, 0, 0;
      sigma << 0, 0, 0, 0;
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

      // Error definition
      ref_rot << 0, reference_pose.pose.position.x, reference_pose.pose.position.y, reference_pose.pose.position.z;
      //ref_rot << kronecker(q_hat, kronecker(ref_rot, q_hat_conj));
      ref_rot << kronecker(kronecker(q_hat, ref_rot), q_hat_conj);
      //e << ref_rot(1) - estimator_pose.pose.position.x,
      //   ref_rot(2) - estimator_pose.pose.position.y,
      //   ref_rot(3) - estimator_pose.pose.position.z,
      //   eta_e(3);
      e << reference_pose.pose.position.x - estimator_pose.pose.position.x,
	   reference_pose.pose.position.y - estimator_pose.pose.position.y,
	   reference_pose.pose.position.z - estimator_pose.pose.position.z,
	   eta_e(3);
      
      std::cout << "Error: x" << e(0) << " y: " << e(1) << " z: " << e(2) << " psi: " << e(3) << std::endl;
      //std::cout << "Ref rot: x" << ref_rot(1) << " y: " << ref_rot(2) << " z: " << ref_rot(3) << std::endl;

      e_dot << estimator_velocity.linear.x  - reference_velocity.linear.x,
	       estimator_velocity.linear.y  - reference_velocity.linear.y,
	       estimator_velocity.linear.z  - reference_velocity.linear.z,
	       estimator_velocity.angular.z - reference_velocity.angular.z;
	
      //std::cout << "Error_dot: x" << e_dot(0) << " y: " << e_dot(1) << " z: " << e_dot(2) << " psi: " << e_dot(3) << std::endl;


      // Sliding surface 
      sigma << e + ewise(zetta1, sig4(e, lambda1)) + ewise(zetta2, sig4(e_dot, lambda2));
      K_dot << ewise(exp4(alpha, 0.5), exp4(sigma.cwiseAbs(), 0.5)) + ewise(exp4(beta, 0.5), exp4(K, 2));

      //std::cout << "Sigma: x" << sigma(0) << " y: " << sigma(1) << " z: " << sigma(2) << " psi: " << sigma(3) << std::endl;
      //std::cout << "Sigma_abs: x" << sigma.cwiseAbs()(0) << " y: " << sigma.cwiseAbs()(1) << " z: " << sigma.cwiseAbs()(2) << " psi: " << sigma.cwiseAbs()(3) << std::endl;
      //std::cout << "K_dot: x" << K_dot(0) << " y: " << K_dot(1) << " z: " << K_dot(2) << " psi: " << K_dot(3) << std::endl;

      //sigma << e + ewise(zetta1, sig4(e, lambda1)) + ewise(zetta2, sig4(e_dot, lambda2));
      //K_dot << ewise(alpha.pow(0.5), sigma.cwiseAbs().pow(0.5)) + ewise(beta.pow(0.5), K.pow(2));

      // Euler integrate K_dot to get K
      K += K_dot * 0.1;

      // Saturate K
      for (int i = 0; i < 4; i++){
	if (K(i) > 1){
	  K(i) = 1;
	} else if (K(i) < -1){
	  K(i) = -1;
	}
      }

      // Control law
      //uaux << -2 * K * sig4(sigma, 0.5) - exp4(K, 2) * sigma * 0.5;

      // Control law using ewise
      uaux << -2 * ewise(K, sig4(sigma, 0.5)) - ewise(exp4(K, 2), sigma) * 0.5;
      
      std::cout << "Control: x" << uaux(0) << " y: " << uaux(1) << " z: " << uaux(2) << " psi: " << uaux(3) << std::endl;

      // Saturate control output
      uaux(0) = std::min(std::max(uaux(0), -1.6), 1.6);
      uaux(1) = std::min(std::max(uaux(1), -1.6), 1.6);
      uaux(2) = std::min(std::max(uaux(2), -0.9), 1.0);
      uaux(3) = std::min(std::max(uaux(3), -1.0), 1.0);

      // Normalize control output
      
      uaux(0) = 50 * (uaux(0) + 1.6)/(3.2) - 25;
      uaux(1) = 50 * (uaux(1) + 1.6)/(3.2) - 25;
      uaux(2) = 50 * (uaux(2) + 0.9)/(1.9) - 25;
      uaux(3) = 50 * (uaux(3) + 1.0)/(2.0) - 25;

      _uaux.linear.x = -uaux(0);
      _uaux.linear.y = -uaux(1);
      _uaux.linear.z = -uaux(2);
      _uaux.angular.z = uaux(3);

      _sigma.linear.x = sigma(0);
      _sigma.linear.y = sigma(1);
      _sigma.linear.z = sigma(2);
      _sigma.angular.z = sigma(3);

      _error.pose.position.x = e(0);
      _error.pose.position.y = e(1);
      _error.pose.position.z = e(2);
      _error.pose.orientation.w = e(3);

      uaux_publisher->publish(_uaux);
      sigma_publisher->publish(_sigma);
      error_publisher->publish(_error);
      
    }

  private:

    geometry_msgs::msg::PoseStamped estimator_pose; 
    geometry_msgs::msg::PoseStamped reference_pose;
    geometry_msgs::msg::PoseStamped _error;
    geometry_msgs::msg::Twist estimator_velocity;
    geometry_msgs::msg::Twist reference_velocity;
    geometry_msgs::msg::Twist _uaux;
    geometry_msgs::msg::Twist _sigma;

    Eigen::Vector4d zetta1;
    Eigen::Vector4d zetta2;
    Eigen::Vector4d lambda1;
    Eigen::Vector4d lambda2;
    Eigen::Vector4d e;
    Eigen::Vector4d ref_rot;
    Eigen::Vector4d e_dot;
    Eigen::Vector4d q_hat;
    Eigen::Vector4d q_hat_conj;
    Eigen::Vector4d q_d;
    Eigen::Vector4d q_e;
    Eigen::Vector4d eta_e;
    Eigen::Vector4d K;
    Eigen::Vector4d K_dot;
    Eigen::Vector4d alpha;
    Eigen::Vector4d beta;
    Eigen::Vector4d sigma;
    Eigen::Vector4d uaux;

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr estimator_velocity_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reference_velocity_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sigma_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsmcController>());
  rclcpp::shutdown();
  return 0;
}
