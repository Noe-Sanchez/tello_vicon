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
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

double sig(double x, double exponent){
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

template <typename VectorType>
VectorType sig(VectorType v, double exponent){
  VectorType s;

  for (int i = 0; i < v.size(); i++){
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

template <typename VectorType>
VectorType sig(VectorType v, VectorType exponent){
  VectorType s;

  for (int i = 0; i < v.size(); i++){
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

  //double norm = sqrt(q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
  double norm = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3));

  if (norm < 0.01){
    q_log << 0, 0, 0, 0;
  } else {
    // Use arccos to get the angle
    q_log << 0, q(1)/norm, q(2)/norm, q(3)/norm;
    if (q(0) > 1){
      q(0) = 1;
    } else if (q(0) < -1){
      q(0) = -1;
    }
    double acosaux = acos(q(0)) < acos(-q(0)) ? acos(q(0)) : acos(-q(0));
    //q_log = acos(q(0)) * q_log;
    q_log = acosaux * q_log;
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

Eigen::Vector4d sqrt4(Eigen::Vector4d v){
  Eigen::Vector4d s;

  for (int i = 0; i < 4; i++){
    s(i) = sqrt(v(i));
  }

  return s;
}

class AsmcController : public rclcpp::Node{
  public:
    AsmcController(): Node("asmc_node"){
      // Subscribers
      estimator_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/estimator/pose", 10, std::bind(&AsmcController::estimator_pose_callback, this, std::placeholders::_1));
      estimator_velocity_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>("tello/estimator/velocity", 10, std::bind(&AsmcController::estimator_velocity_callback, this, std::placeholders::_1));
      reference_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/reference/pose", 10, std::bind(&AsmcController::position_reference_callback, this, std::placeholders::_1));
      reference_velocity_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>("tello/reference/velocity", 10, std::bind(&AsmcController::velocity_reference_callback, this, std::placeholders::_1));

      // Publishers
      uaux_publisher    = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);
      sigma_publisher   = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/sigma", 10);
      error_publisher   = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/error", 10);
      ref_rot_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/ref_rot", 10);

      k_publisher     = this->create_publisher<geometry_msgs::msg::TwistStamped>("tello/control/k", 10);
      e_dot_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("tello/control/e_dot", 10);

      // Make 0.5s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&AsmcController::control_callback, this));
  
      // Initialize variables 
      //zetta << 1, 1, 1, 4;
      zetta << 5.45, 5.45, 4.25, 4;
      lambda << 1.2, 1.2, 1.2, 1;
      alpha << 0.001, 0.001, 0.001, 0.001;
      beta << 0.01, 0.01, 0.01, 0.01;

      lambda_minus_1 << lambda(0)-1, lambda(1)-1, lambda(2)-1, lambda(3)-1;

      e          << 0, 0, 0, 0;
      ref_rot    << 0, 0, 0, 0;
      e_dot      << 0, 0, 0, 0;
      q_hat      << 1, 0, 0, 0;
      q_hat_conj << 1, 0, 0, 0;
      q_d        << 1, 0, 0, 0;
      q_e        << 1, 0, 0, 0;
      eta_e      << 0, 0, 0, 0;
      K          << 0, 0, 0, 0;
      K_dot      << 0, 0, 0, 0;
      sigma      << 0, 0, 0, 0;
      uaux       << 0, 0, 0, 0;

      reference_pose.pose.position.x = 0;
      reference_pose.pose.position.y = 0;
      reference_pose.pose.position.z = 1;
      reference_pose.pose.orientation.w = 1;
      reference_pose.pose.orientation.x = 0;
      reference_pose.pose.orientation.y = 0;
      reference_pose.pose.orientation.z = 0;

    }

    void estimator_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      estimator_pose = *msg;
    }
    void estimator_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
      estimator_velocity = *msg;
    }
    void position_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      reference_pose = *msg;
    }
    void velocity_reference_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
      reference_velocity = *msg;
    }


    void control_callback(){
      // Conjugate q_hat
      q_hat << estimator_pose.pose.orientation.w, estimator_pose.pose.orientation.x, estimator_pose.pose.orientation.y, estimator_pose.pose.orientation.z;
      q_hat_conj << q_hat(0), -q_hat(1), -q_hat(2), -q_hat(3);

      q_d << reference_pose.pose.orientation.w, reference_pose.pose.orientation.x, reference_pose.pose.orientation.y, reference_pose.pose.orientation.z;
      q_e = kronecker(q_hat_conj, q_d);

      // Normalize q_e
      double q_e_norm = sqrt(q_e(0)*q_e(0) + q_e(1)*q_e(1) + q_e(2)*q_e(2) + q_e(3)*q_e(3));
      if (q_e_norm != 0){
	      q_e << q_e(0)/q_e_norm, q_e(1)/q_e_norm, q_e(2)/q_e_norm, q_e(3)/q_e_norm;
      }else{
	q_e << 1, 0, 0, 0;
      }

      // Logarithmic mapping of q_e
      eta_e << qlm(q_e);

      // Check for NaN in eta_e
      for (int i = 0; i < 4; i++){
        if (std::isnan(eta_e(i))){
          std::cout << "Eta_e is NaN at element " << i << std::endl;
          eta_e(i) = 0;
        }
      }

      e << reference_pose.pose.position.x - estimator_pose.pose.position.x,
           reference_pose.pose.position.y - estimator_pose.pose.position.y,
           reference_pose.pose.position.z - estimator_pose.pose.position.z,
	   eta_e(3);
      
      e_dot << reference_velocity.twist.linear.x  - estimator_velocity.twist.linear.x,
	       reference_velocity.twist.linear.y  - estimator_velocity.twist.linear.y,
	       reference_velocity.twist.linear.z  - estimator_velocity.twist.linear.z,
	       reference_velocity.twist.angular.z - estimator_velocity.twist.angular.z;
      Eigen::Vector4d xd_dot;
      xd_dot << reference_velocity.twist.linear.x,
                reference_velocity.twist.linear.y,
                reference_velocity.twist.linear.z,
                reference_velocity.twist.angular.z;

      // Sliding surface 
      sigma << e + ewise(zetta, sig(e, lambda)); 

      // Check for NaN in sigma. Old check, not needed anymore, kept for security
      for (int i = 0; i < 4; i++){
	if (std::isnan(sigma(i))){
	  std::cout << "Sigma is NaN at element " << i << std::endl;
	  sigma(i) = 0;
	}
      }

      // K dynamics
      K_dot << ewise(exp4(alpha, 0.5), exp4(sigma.cwiseAbs(), 0.5)) - ewise(exp4(beta, 0.5), exp4(K, 2));

      // Euler integrate K_dot to get K
      K += K_dot * 0.01;


      // Control law 
      //uaux << -2 * ewise(K, sig(sigma, 0.5)) - ewise(exp4(K, 2), sigma) * 0.5; //Original
      uaux << -2 * ewise(K, sig(sigma, 0.5)) - ewise(K, sigma) * 0.5;
              
      // Original feedback linearization
      /*u_rot << 0,
               -uaux(0) + ewise(zetta, sig(e, lambda))(0) + xd_dot(0),
               -uaux(1) + ewise(zetta, sig(e, lambda))(1) + xd_dot(1),
               -uaux(2) + ewise(zetta, sig(e, lambda))(2) + xd_dot(2);*/
      
      // New feedback linearization, lyapunov based
      term1 << 1 + zetta(0) * lambda(0) * pow(abs(e(0)), lambda_minus_1(0)),
	       1 + zetta(1) * lambda(1) * pow(abs(e(1)), lambda_minus_1(1)), 
	       1 + zetta(2) * lambda(2) * pow(abs(e(2)), lambda_minus_1(2)),
	       1 + zetta(3) * lambda(3) * pow(abs(e(3)), lambda_minus_1(3));
      u_rot << 0,
               xd_dot(0) -uaux(0)/term1(0),
	       xd_dot(1) -uaux(1)/term1(1),
	       xd_dot(2) -uaux(2)/term1(2);
      u_rot = kronecker(kronecker(q_hat_conj, u_rot), q_hat);

      uaux(3) = -uaux(3) + xd_dot(3) + ewise(zetta, sig(e, lambda))(3);

      uaux(0) = std::min(std::max(u_rot(1), -1.6), 1.6);
      uaux(1) = std::min(std::max(u_rot(2), -1.6), 1.6);
      uaux(2) = std::min(std::max(u_rot(3), -1.0), 1.0);
      uaux(3) = std::min(std::max(uaux(3),  -1.0), 1.0);


      // Normalize control output
      
      uaux(0) = 100 * (uaux(0) + 1.6)/(3.2) - 50;
      uaux(1) = 100 * (uaux(1) + 1.6)/(3.2) - 50;
      //uaux(2) = 100 * (uaux(2) + 0.9)/(1.9) - 50;
      uaux(2) = 100 * (uaux(2) + 1.0)/(2.0) - 50;
      uaux(3) = 100 * (uaux(3) + 1.0)/(2.0) - 50;

      _uaux.linear.x =  uaux(0);
      _uaux.linear.y =  uaux(1);
      _uaux.linear.z =  uaux(2);
      _uaux.angular.z = uaux(3);

      _sigma.linear.x = sigma(0);
      _sigma.linear.y = sigma(1);
      _sigma.linear.z = sigma(2);
      _sigma.angular.z = sigma(3);

      _error.pose.position.x = e(0);
      _error.pose.position.y = e(1);
      _error.pose.position.z = e(2);
      _error.pose.orientation.w = e(3);

      _ref_rot.pose.position.x = u_rot(1);
      _ref_rot.pose.position.y = u_rot(2);
      _ref_rot.pose.position.z = u_rot(3);
      //_ref_rot.pose.position.x = K(0);
      //_ref_rot.pose.position.y = K(1);
      //_ref_rot.pose.position.z = K(2);
      //_ref_rot.pose.orientation.w = K(3);
      _ref_rot.pose.orientation.w = reference_pose.pose.orientation.w;
      _ref_rot.pose.orientation.x = reference_pose.pose.orientation.x;
      _ref_rot.pose.orientation.y = reference_pose.pose.orientation.y;
      _ref_rot.pose.orientation.z = reference_pose.pose.orientation.z;
      _ref_rot.header.frame_id = "world";

      _k.twist.linear.x  = K(0);
      _k.twist.linear.y  = K(1);
      _k.twist.linear.z  = K(2);
      _k.twist.angular.z = K(3);
      _k.header.frame_id = "tello";
      _k.header.stamp = this->now();

      _e_dot.twist.linear.x  = e_dot(0);
      _e_dot.twist.linear.y  = e_dot(1);
      _e_dot.twist.linear.z  = e_dot(2);
      _e_dot.twist.angular.z = e_dot(3);
      _e_dot.header.frame_id = "tello";
      _e_dot.header.stamp = this->now();

      uaux_publisher->publish(_uaux);
      sigma_publisher->publish(_sigma);
      error_publisher->publish(_error);
      ref_rot_publisher->publish(_ref_rot);
      
      k_publisher->publish(_k);
      e_dot_publisher->publish(_e_dot);
    }

  private:

    geometry_msgs::msg::PoseStamped estimator_pose; 
    geometry_msgs::msg::PoseStamped reference_pose;
    geometry_msgs::msg::PoseStamped _error;
    geometry_msgs::msg::PoseStamped _ref_rot;
    geometry_msgs::msg::TwistStamped estimator_velocity;
    geometry_msgs::msg::TwistStamped reference_velocity;
    geometry_msgs::msg::TwistStamped _k;
    geometry_msgs::msg::TwistStamped _e_dot;
    geometry_msgs::msg::Twist _uaux;
    geometry_msgs::msg::Twist _sigma;

    Eigen::Vector4d zetta;
    Eigen::Vector4d lambda;
    Eigen::Vector4d lambda_minus_1;
    Eigen::Vector4d term1;
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
    Eigen::Vector4d u_rot;

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr estimator_velocity_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_velocity_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sigma_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_rot_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr k_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr e_dot_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsmcController>());
  rclcpp::shutdown();
  return 0;
}
