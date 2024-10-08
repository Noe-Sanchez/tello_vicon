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

Eigen::Vector4d sqrt4(Eigen::Vector4d v){
  Eigen::Vector4d s;

  for (int i = 0; i < 4; i++){
    s(i) = sqrt(v(i));
  }

  return s;
}

double euclidean2(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2){
  double distance = sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2));
  return distance;
}

double euclidean3(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2){
  double distance = sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2) + pow(p1.pose.position.z - p2.pose.position.z, 2));
  return distance;
}

class AsmcAvoidanceController : public rclcpp::Node{
  public:
    AsmcAvoidanceController(): Node("asmc_avoidance_node"){
      // Subscribers
      //estimator_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/estimator/pose", 10, std::bind(&AsmcAvoidanceController::estimator_pose_callback, this, std::placeholders::_1));
      estimator_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vicon/TelloMount1/TelloMount1", 10, std::bind(&AsmcAvoidanceController::estimator_pose_callback, this, std::placeholders::_1));
      estimator_velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/estimator/velocity", 10, std::bind(&AsmcAvoidanceController::estimator_velocity_callback, this, std::placeholders::_1));
      reference_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/reference/pose", 10, std::bind(&AsmcAvoidanceController::position_reference_callback, this, std::placeholders::_1));
      reference_velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("tello/reference/velocity", 10, std::bind(&AsmcAvoidanceController::velocity_reference_callback, this, std::placeholders::_1));
      wand_pose_subscriber          = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vicon/Stick/Stick", 10, std::bind(&AsmcAvoidanceController::wand_pose_callback, this, std::placeholders::_1));

      // Publishers
      uaux_publisher  = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);
      sigma_publisher = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/sigma", 10);
      error_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/error", 10);
      ref_rot_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/ref_rot", 10);
      gamma_publisher = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/gamma", 10);

      // Make 0.5s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&AsmcAvoidanceController::control_callback, this));
  
      // Initialize variables 
      zetta1 << 1.75, 1.75, 1.5, 1.5;
      zetta2 << 1.5, 1.5, 2, 2;
      // lambda1 > lambda2
      lambda1 << 2, 2, 2, 10;
      lambda2 << 1.3, 1.3, 1.3, 1.3;
      alpha << 0.075, 0.075, 0.1, 0.1;
      beta << 5, 5, 1, 5;

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
    void estimator_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      estimator_velocity = *msg;
    }
    void position_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      reference_pose = *msg;
    }
    void velocity_reference_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      reference_velocity = *msg;
    }
    void wand_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      wand_pose = *msg;
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

      // Logarithmic mapping of q_e (0, eta_e_phi, eta_e_theta, eta_e_psi)
      std ::cout << "q_e: w" << q_e(0) << " x: " << q_e(1) << " y: " << q_e(2) << " z: " << q_e(3) << std::endl;
      eta_e << qlm(q_e);

      // Check for NaN in eta_e
      for (int i = 0; i < 4; i++){
        if (std::isnan(eta_e(i))){
          std::cout << "Eta_e is NaN at element " << i << std::endl;
          eta_e(i) = 0;
        }
      }


      // Error definition
      //ref_rot << 0, reference_pose.pose.position.x, reference_pose.pose.position.y, reference_pose.pose.position.z;
      //ref_rot << kronecker(q_hat, kronecker(ref_rot, q_hat_conj));
      //ref_rot << kronecker(kronecker(q_hat, ref_rot), q_hat_conj);
      //ref_rot << ref_rot(0),
      //ref_rot(1) + estimator_pose.pose.position.x,
      //ref_rot(2) + estimator_pose.pose.position.y,
      //ref_rot(3);

      //std::cout << "Ref x: " << reference_pose.pose.position.x << " y: " << reference_pose.pose.position.y << " z: " << reference_pose.pose.position.z << std::endl;
      //std::cout << "Ref rot: x" << ref_rot(1) << " y: " << ref_rot(2) << " z: " << ref_rot(3) << std::endl;

      //e << ref_rot(1) - estimator_pose.pose.position.x, 
      //     ref_rot(2) - estimator_pose.pose.position.y,
      //     ref_rot(3) - estimator_pose.pose.position.z,
      //     eta_e(3);
       
      //e << ref_rot(1),
      //     ref_rot(2),
      //     ref_rot(3), 
      //     eta_e(3);

      e << reference_pose.pose.position.x - estimator_pose.pose.position.x,
           reference_pose.pose.position.y - estimator_pose.pose.position.y,
           reference_pose.pose.position.z - estimator_pose.pose.position.z,
           //1-estimator_pose.pose.position.z,
	         eta_e(3);
      
      //std::cout << "Error: x" << e(0) << " y: " << e(1) << " z: " << e(2) << " psi: " << e(3) << std::endl;
      //std::cout << "Ref rot: x" << ref_rot(1) << " y: " << ref_rot(2) << " z: " << ref_rot(3) << std::endl;

      e_dot << estimator_velocity.linear.x  - reference_velocity.linear.x,
	       estimator_velocity.linear.y  - reference_velocity.linear.y,
	       estimator_velocity.linear.z  - reference_velocity.linear.z,
	       estimator_velocity.angular.z - reference_velocity.angular.z;
	
      //std::cout << "Error_dot: x" << e_dot(0) << " y: " << e_dot(1) << " z: " << e_dot(2) << " psi: " << e_dot(3) << std::endl;

      // Sliding surface 
      sigma << e + ewise(zetta1, sig(e, lambda1)); //+ ewise(zetta2, sig(e_dot, lambda2));

      // Check for NaN in sigma
      for (int i = 0; i < 4; i++){
	      if (std::isnan(sigma(i))){
          std::cout << "Sigma is NaN at element " << i << std::endl;
	        sigma(i) = 0;
	      }
      }

      K_dot << ewise(exp4(alpha, 0.5), exp4(sigma.cwiseAbs(), 0.5)) - ewise(exp4(beta, 0.5), exp4(K, 2));

      //std::cout << "Sigma: x" << sigma(0) << " y: " << sigma(1) << " z: " << sigma(2) << " psi: " << sigma(3) << std::endl;
      //std::cout << "Sigma_abs: x" << sigma.cwiseAbs()(0) << " y: " << sigma.cwiseAbs()(1) << " z: " << sigma.cwiseAbs()(2) << " psi: " << sigma.cwiseAbs()(3) << std::endl;
      //std::cout << "K_dot: x" << K_dot(0) << " y: " << K_dot(1) << " z: " << K_dot(2) << " psi: " << K_dot(3) << std::endl;
      //std::cout << "K: x" << K(0) << " y: " << K(1) << " z: " << K(2) << " psi: " << K(3) << std::endl;

      //sigma << e + ewise(zetta1, sig4(e, lambda1)) + ewise(zetta2, sig4(e_dot, lambda2));
      //K_dot << ewise(alpha.pow(0.5), sigma.cwiseAbs().pow(0.5)) + ewise(beta.pow(0.5), K.pow(2));

      // Euler integrate K_dot to get K
      K += K_dot * 0.01;

      // Saturate K
      /*for (int i = 0; i < 4; i++){
	      if (K(i) > 10){
	        K(i) = 10;
	      } else if (K(i) < -10){
	        K(i) = -10;
	      }
      }*/

      // Control law
      //uaux << -2 * K * sig4(sigma, 0.5) - exp4(K, 2) * sigma * 0.5;

      // Control law using ewise
      //uaux << -2 * ewise(K, sig4(sigma, 0.5)) - ewise(exp4(K, 2), sigma) * 0.5;
      uaux << -2 * ewise(K, sig(sigma, 0.5)) - ewise(K, sigma) * 0.5;
      
      //std::cout << "Control: x" << uaux(0) << " y: " << uaux(1) << " z: " << uaux(2) << " psi: " << uaux(3) << std::endl;

      // RVF calculation
      double D = 2.0; // Sensing distance
      //double d = 0.5; // Safe distance
      //float V = (pow(d,2) / (pow(estimator.pose.position.x - wand.pose.position.x, 2) + pow(estimator.pose.position.y - wand.pose.position.y, 2))) - 1 + 0.1;
      double V = (pow(D,2) / pow(euclidean2(estimator_pose, wand_pose), 2)) - 1 + 0.1;
      
      // Conmutation parameter
      int delta = 0;
      if ( euclidean2(estimator_pose, wand_pose) < D){
        delta = 1;
      } else {
        delta = 0;
      }

      double fieldx = delta * V * ((estimator_pose.pose.position.x - wand_pose.pose.position.x) - (estimator_pose.pose.position.y - wand_pose.pose.position.y));
      double fieldy = delta * V * ((estimator_pose.pose.position.x - wand_pose.pose.position.x) + (estimator_pose.pose.position.y - wand_pose.pose.position.y));

      double eta = 1; // Field scaling parameter
      

      // Integrate field into uaux
      uaux(0) -= fieldx * eta;
      uaux(1) -= fieldy * eta;

      // Rotate control output in x and y
      Eigen::Vector4d uaux_rot;
      uaux_rot << 0, uaux(0), uaux(1), 0;
      uaux_rot = kronecker(kronecker(q_hat_conj, uaux_rot), q_hat);
      //uaux_rot = kronecker(kronecker(q_hat, uaux_rot), q_hat_conj);     

      uaux(0) = uaux_rot(1);
      uaux(1) = uaux_rot(2);

      // Saturate control output
      uaux(0) = std::min(std::max(uaux(0), -1.6), 1.6);
      uaux(1) = std::min(std::max(uaux(1), -1.6), 1.6);
      uaux(2) = std::min(std::max(uaux(2), -0.9), 1.0);
      uaux(3) = std::min(std::max(uaux(3), -1.0), 1.0);

      // Normalize control output
      
      uaux(0) = 100 * (uaux(0) + 1.6)/(3.2) - 50;
      uaux(1) = 100 * (uaux(1) + 1.6)/(3.2) - 50;
      uaux(2) = 100 * (uaux(2) + 0.9)/(1.9) - 50;
      uaux(3) = 100 * (uaux(3) + 1.0)/(2.0) - 50;

      
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

      _ref_rot.pose.position.x = uaux_rot(1);
      _ref_rot.pose.position.y = uaux_rot(2);
      _ref_rot.pose.position.z = uaux_rot(3);
      //_ref_rot.pose.position.x = K(0);
      //_ref_rot.pose.position.y = K(1);
      //_ref_rot.pose.position.z = K(2);
      //_ref_rot.pose.orientation.w = K(3);
      _ref_rot.pose.orientation.w = reference_pose.pose.orientation.w;
      _ref_rot.pose.orientation.x = reference_pose.pose.orientation.x;
      _ref_rot.pose.orientation.y = reference_pose.pose.orientation.y;
      _ref_rot.pose.orientation.z = reference_pose.pose.orientation.z;
      _ref_rot.header.frame_id = "world";

      _gamma.linear.x = fieldx * eta;
      _gamma.linear.y = fieldy * eta;

      uaux_publisher->publish(_uaux);
      sigma_publisher->publish(_sigma);
      error_publisher->publish(_error);
      ref_rot_publisher->publish(_ref_rot);
      gamma_publisher->publish(_gamma);
     
    }

  private:

    geometry_msgs::msg::PoseStamped estimator_pose; 
    geometry_msgs::msg::PoseStamped reference_pose;
    geometry_msgs::msg::PoseStamped wand_pose;
    geometry_msgs::msg::PoseStamped _error;
    geometry_msgs::msg::PoseStamped _ref_rot;
    geometry_msgs::msg::Twist estimator_velocity;
    geometry_msgs::msg::Twist reference_velocity;
    geometry_msgs::msg::Twist _uaux;
    geometry_msgs::msg::Twist _sigma;
    geometry_msgs::msg::Twist _gamma;

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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr wand_pose_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sigma_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_rot_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gamma_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsmcAvoidanceController>());
  rclcpp::shutdown();
  return 0;
}
