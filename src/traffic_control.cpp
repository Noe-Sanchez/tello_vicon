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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std::chrono_literals;

Eigen::Vector4d cross4(Eigen::Vector4d a, Eigen::Vector4d b){
//Eigen::Vector4d cross4(Eigen::Vector4d a, Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr b){
  // Calculate cross product of a and b, where both are [0, x, y, z]^T
  // Result is [0, a x b]^T
  Eigen::Vector4d c;
  c(0) = 0;
  c(1) = a(2)*b(3) - a(3)*b(2);
  c(2) = a(3)*b(1) - a(1)*b(3);
  c(3) = a(1)*b(2) - a(2)*b(1);
  return c;
}

/*Eigen::Vector4d sig(Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr v, double exponent){
  Eigen::Vector4d s;
  s(0) = v(0);
  s(1) = v(1);
  s(2) = v(2);
  s(3) = v(3);

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
}*/

// Quick and dirty quaternion multiplication for matrix slices
/*Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  q_matrix << q(0), -q(1), -q(2), -q(3),
	      q(1),  q(0), -q(3),  q(2),
	      q(2),  q(3),  q(0), -q(1),
	      q(3), -q(2),  q(1),  q(0);

  p_vector = q_matrix * p;

  return p_vector;
}
Eigen::Vector4d kronecker(Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr q, Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  q_matrix << q(0), -q(1), -q(2), -q(3),
	      q(1),  q(0), -q(3),  q(2),
	      q(2),  q(3),  q(0), -q(1),
	      q(3), -q(2),  q(1),  q(0);

  p_vector = q_matrix * p;

  return p_vector;
}

Eigen::Vector4d kronecker(Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr q, Eigen::Vector4d p){
	Eigen::Matrix4d q_matrix;
	Eigen::Vector4d p_vector;

	q_matrix << q(0), -q(1), -q(2), -q(3),
	      q(1),  q(0), -q(3),  q(2),
	      q(2),  q(3),  q(0), -q(1),
	      q(3), -q(2),  q(1),  q(0);

	p_vector = q_matrix * p;

	return p_vector;
}*/


class TrafficControl : public rclcpp::Node{
  public:
    TrafficControl(): Node("traffic_control_node"){
      // Get drone_id parameter
      this->declare_parameter("num_drones", 0);
      num_drones = this->get_parameter("num_drones").as_int();

      // Formation definition subscriber
      formation_definition_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation_definition", 10, std::bind(&TrafficControl::formation_definition_callback, this, std::placeholders::_1));
      // Leader pose subscriber
      leader_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tello_leader/tello/estimator/pose", 10, std::bind(&TrafficControl::leader_pose_callback, this, std::placeholders::_1));
      
      // Make 0.5s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&TrafficControl::control_callback, this));
  
      // Initialize variables
      Eigen::Vector4d zero4d = Eigen::Vector4d::Zero();
      dl     = zero4d;
      vl     = zero4d;
      omegal = zero4d;
      ql_dot = zero4d; 
      ql     << 1, 0, 0, 0; 
      ql_conj << 1, 0, 0, 0;

      /*Eigen::Matrix<double, 4, Eigen::Dynamic> zero4d_dyn = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, num_drones); 
      dfs         = zero4d_dyn;
      vfs         = zero4d_dyn;
      gammas      = zero4d_dyn;
      gammas_dot  = zero4d_dyn;
      lambdas     = zero4d_dyn;
      lambdas_dot = zero4d_dyn;
      gammasd     = zero4d_dyn;
      gammasd_dot = zero4d_dyn;
      etas        = zero4d_dyn;
      efs         = zero4d_dyn;
      efs_prev    = zero4d_dyn;
      efs_int     = zero4d_dyn;
      k           = zero4d_dyn;
      kappa1      = zero4d_dyn;
      kappa2      = zero4d_dyn;
      sigmaf      = zero4d_dyn;
      ufs         = zero4d_dyn;
      ufs_prev    = zero4d_dyn;
      ufs_int     = zero4d_dyn;
      omegafu     = zero4d_dyn;
      qfs         = zero4d_dyn;
      qfs_conj    = zero4d_dyn;
      qdfs        = zero4d_dyn;
      qfu         = zero4d_dyn;*/

      //Eigen::Vector<double, num_drones> ones_dyn = Eigen::Vector<double, num_drones>::Ones();
      //Eigen::Vector<double, Eigen::Dynamic> ones_dyn = Eigen::Vector<double, Eigen::Dynamic>::Ones(num_drones);

      // Set all columns of quaternion matrices to identity ( qfs, qfs_conj, qdfs, qfu)
      /*qfs.row(0)      = ones_dyn;
      qfs_conj.row(0) = ones_dyn;
      qdfs.row(0)     = ones_dyn;
      qfu.row(0)       = ones_dyn;*/
      Eigen::Vector4d identity;
      identity << 1, 0, 0, 0;

      // Initialize subscribers and publishers
      for (int i = 0; i < num_drones; i++){
	std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> follower_proto = std::bind(&TrafficControl::follower_pose_callback, this, std::placeholders::_1, i);
	follower_pose_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/estimator/pose", 10, follower_proto));
	std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)> follower_vel_proto = std::bind(&TrafficControl::follower_vel_callback, this, std::placeholders::_1, i);	
	follower_vel_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/estimator/velocity", 10, follower_vel_proto));

	follower_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/reference/pose", 10));
	follower_vel_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/reference/velocity", 10));
	//follower_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/echo/pose", 10));
	//follower_vel_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/echo/velocity", 10));
	
	follower_pose_msgs.push_back(geometry_msgs::msg::PoseStamped());
	follower_vel_msgs.push_back(geometry_msgs::msg::TwistStamped());

	// Set all matrices to zero, and all quaternions to identity, except gains
	dfs.push_back(zero4d);
	vfs.push_back(zero4d);
	gammas.push_back(zero4d);
	gammas_dot.push_back(zero4d);
	lambdas.push_back(zero4d);
	lambdas_dot.push_back(zero4d);
	gammasd.push_back(zero4d);
	gammasd_dot.push_back(zero4d);
	etas.push_back(zero4d);
	efs.push_back(zero4d);
	efs_prev.push_back(zero4d);
	efs_int.push_back(zero4d);
	sigmaf.push_back(zero4d);
	uauxs.push_back(zero4d);
	ufs.push_back(zero4d);
	ufs_prev.push_back(zero4d);
	ufs_int.push_back(zero4d);
	omegafu.push_back(zero4d);
	qfu_dot.push_back(zero4d);
	qfu_dot_prev.push_back(zero4d);
	qfu.push_back(identity);
	qfs.push_back(identity);
	qfs_conj.push_back(identity);
	qdfs.push_back(identity);
	qfu_conj.push_back(identity);

	k.push_back(zero4d);
	k.back() << 0.00001, 0.00001, 0.00001, 0.00001;
	kappa1.push_back(zero4d);
	kappa1.back() << 0.1, 0.1, 0.1, 0.1; 
	kappa2.push_back(zero4d);
	kappa2.back() << 0.00001, 0.00001, 0.00001, 0.00001; 

      }
    }

    void control_callback(){
      for (int i = 0; i < num_drones; i++){
	// Calculate gammas and lambdas
	//lambdas[i] = dfs[i] - dl; // Both are [0, x, y, z]^T
	//lambdas_dot[i] = vfs[i] - vl; // Both are [x_dot, y_dot, z_dot, yaw_dot]^T
	lambdas[i] = ufs_int[i] - dl; // Both are [0, x, y, z]^T
	lambdas_dot[i] = ufs[i] - vl; // Both are [x_dot, y_dot, z_dot, yaw_dot]^T
	gammas[i] = kronecker(kronecker(ql, lambdas[i]), ql_conj); 
	//std::cout << "Gammas num " << i << ": " << gammas[i](0) << ", " << gammas[i](1) << ", " << gammas[i](2) << ", " << gammas[i](3) << std::endl;
	gammas_dot[i] = cross4(omegal, gammas[i]) + kronecker(kronecker(ql, lambdas_dot[i]), ql_conj);
	//std::cout << "Gammas_dot num " << i << ": " << gammas_dot[i](0) << ", " << gammas_dot[i](1) << ", " << gammas_dot[i](2) << ", " << gammas_dot[i](3) << std::endl;

	// Calculate errors and etas
	//etas[i] = qlm(qfs_conj[i], kronecker(ql, qdfs[i]));
	Eigen::Vector4d eta_norm;
	eta_norm << 0, 0, 0, 0;
	//eta_norm = kronecker(qfs_conj[i], kronecker(ql, qdfs[i]));
	//eta_norm = kronecker(qfu[i], kronecker(ql, qdfs[i]));
	eta_norm = kronecker(qfu_conj[i], kronecker(ql, qdfs[i]));
	etas[i] = eta_norm / eta_norm.norm();
	etas[i] = qlm(etas[i]);
	std::cout << "Etas num " << i << ": " << etas[i](0) << ", " << etas[i](1) << ", " << etas[i](2) << ", " << etas[i](3) << std::endl;

	//etas[i] = qlm(kronecker(qfs_conj[i], kronecker(ql, qdfs[i])));
	// Fill efs elementwise
	/*efs(0, i) = gammasd(1, i) - gammas(1, i);
	efs(1, i) = gammasd(2, i) - gammas(2, i);
	efs(2, i) = gammasd(3, i) - gammas(3, i);
	efs(3, i) = etas(3, i);*/
	efs[i](0) = gammasd[i](1) - gammas[i](1);
	efs[i](1) = gammasd[i](2) - gammas[i](2);
	efs[i](2) = gammasd[i](3) - gammas[i](3);
	efs[i](3) = etas[i](3);
	//std::cout << "Efs num " << i << ": " << efs[i](0) << ", " << efs[i](1) << ", " << efs[i](2) << ", " << efs[i](3) << std::endl;

	// Compute sliding surface
	// efs trapezoidal integral
	efs_int[i] += 0.5*(efs[i] + efs_prev[i]);
	efs_prev[i] = efs[i];

	sigmaf[i] = efs[i] + k[i].cwiseProduct(efs_int[i]);
	//sigmaf[i] << efs[i];
	
	// Compute auxiliar control output (Continuous non-adaptive smc)
	uauxs[i] = -kappa1[i].cwiseProduct(sig(sigmaf[i], 0.5)) - kappa2[i].cwiseProduct(sigmaf[i]);
	//uauxs[i] << -kappa1[i].cwiseProduct(sigmaf[i]);
	//std::cout << "Sigmaf num " << i << ": " << sigmaf[i] << std::endl;
	//std::cout << "Uauxs num " << i << ": " << uauxs[i] << std::endl;
	//std::cout << "Efs num " << i << ": " << efs[i] << std::endl;
	//uauxs[i] << 0.001, 0, 0, 0;
	//std::cout << "Uauxs num " << i << ": " << uauxs[i](0) << ", " << uauxs[i](1) << ", " << uauxs[i](2) << ", " << uauxs[i](3) << std::endl;
	
	// Epitelos, compute total control output
	//Eigen::Vector4d c13 = k[i].cwiseProduct(efs[i]) + uauxs[i];
	//Eigen::Vector4d c13 = k[i].cwiseProduct(efs[i]) - uauxs[i];
	Eigen::Vector4d c13 = -uauxs[i]; 
	Eigen::Vector4d v13;
	v13(0) = 0;
	v13(1) = vl(0);
	v13(2) = vl(1);
	v13(3) = vl(2);
	// Reorder elements for quaternion multiplication
	//omegafu(3, i) = c13(3);
	omegafu[i](3) = -c13(3);
	std::cout << "Omegafu num " << i << ": " << omegafu[i] << std::endl;
        c13(3) = c13(2);
	c13(2) = c13(1);
	c13(1) = c13(0);
	c13(0) = 0;
	// C13 should be [0, x, y, z]^T
	// Bullshit term because Eigen .col() is trash
	Eigen::Vector4d bullshit;
	Eigen::Vector4d bullshit2;
	bullshit  << 0, 0, 0, 0;
	bullshit2 << 0, 0, 0, 0;
	/*std::cout << "PREPRINT" << std::endl;
	std::cout << "Bullshit num " << i << ": " << bullshit(0) << ", " << bullshit(1) << ", " << bullshit(2) << ", " << bullshit(3) << std::endl;
	std::cout << "Cross4 num " << i << ": " << cross4(omegal, gammas[i]) << std::endl; 
	std::cout << "C13 num " << i << ": " << c13(0) << ", " << c13(1) << ", " << c13(2) << ", " << c13(3) << std::endl;
	std::cout << "Gammas_dot num " << i << ": " << gammas_dot[i](0) << ", " << gammas_dot[i](1) << ", " << gammas_dot[i](2) << ", " << gammas_dot[i](3) << std::endl;
	std::cout << "Bullshit num " << i << ": " << bullshit(0) << ", " << bullshit(1) << ", " << bullshit(2) << ", " << bullshit(3) << std::endl;
	std::cout << "Putain de merde" << gammas_dot[i] + c13 << std::endl;
	std::cout << "Putain de merde2" << cross4(omegal, gammas[i]) << std::endl;*/
	bullshit << gammasd_dot[i] + c13 - cross4(omegal, gammas[i]);
	/*std::cout << "Bullshit num " << i << ": " << bullshit(0) << ", " << bullshit(1) << ", " << bullshit(2) << ", " << bullshit(3) << std::endl;
	std::cout << "Cross4 num " << i << ": " << cross4(omegal, gammas[i])(0) << ", " << cross4(omegal, gammas[i])(1) << ", " << cross4(omegal, gammas[i])(2) << ", " << cross4(omegal, gammas[i])(3) << std::endl;
	std::cout << "C13 num " << i << ": " << c13(0) << ", " << c13(1) << ", " << c13(2) << ", " << c13(3) << std::endl;
	std::cout << "Gammas_dot num " << i << ": " << gammas_dot[i](0) << ", " << gammas_dot[i](1) << ", " << gammas_dot[i](2) << ", " << gammas_dot[i](3) << std::endl;
	std::cout << "Bullshit num " << i << ": " << bullshit(0) << ", " << bullshit(1) << ", " << bullshit(2) << ", " << bullshit(3) << std::endl;
	std::cout << "POSTPRINT" << std::endl;*/
	bullshit2 = v13 + kronecker(kronecker( ql_conj, bullshit ), ql);
	//ufs[i] = kronecker(kronecker(qfs_conj[i], v13 + kronecker(kronecker( ql_conj, gammas_dot[i] + c13 - cross4(omegal, gammas[i])), ql)), qfs[i]);
	//std::cout << "Bullshit num " << i << ": " << bullshit << std::endl;
	//std::cout << "Bullshit2 num " << i << ": " << bullshit2 << std::endl;
	//ufs[i] = kronecker(kronecker(qfs_conj[i], bullshit2), qfs[i]);
	
	//ufs[i] = kronecker(kronecker(qfu_conj[i], bullshit2), qfu[i]);
	ufs[i] << bullshit2;
	// Saturate ufs
	ufs[i](0) = std::min(std::max(ufs[i](0), -1.6), 1.6);
	ufs[i](1) = std::min(std::max(ufs[i](1), -1.6), 1.6);
	ufs[i](2) = std::min(std::max(ufs[i](2), -1.0), 1.0);
	ufs[i](3) = std::min(std::max(ufs[i](3), -1.0), 1.0);

	//std::cout << "Ufs num " << i << ": " << ufs[i] << std::endl; 
	// Ufs should be [0, x, y, z]^T
	// Map omegafu to qfu_dot, then trapezoidal integral
	qfu_dot[i] = 0.5*kronecker(qfu[i], omegafu[i]);
	qfu[i] += 0.5*(qfu_dot[i] + qfu_dot_prev[i]);

	// Normalize qfu
	qfu[i] = qfu[i] / qfu[i].norm();

	qfu_dot_prev[i] = qfu_dot[i];
	qfu_conj[i](0) = qfu[i](0);
	qfu_conj[i](1) = -qfu[i](1);
	qfu_conj[i](2) = -qfu[i](2);
	qfu_conj[i](3) = -qfu[i](3);

	// ufs trapezoidal integral
	ufs_int[i] += 0.5*(ufs[i] + ufs_prev[i]);
	ufs_prev[i] = ufs[i];
	
	//std::cout << "Ufs int num " << i << ": " << ufs_int[i] << std::endl;

	// Publish integrals
	/*follower_pose_msgs[i].pose.position.x = ufs_int(1, i);
	follower_pose_msgs[i].pose.position.y = ufs_int(2, i);
	follower_pose_msgs[i].pose.position.z = ufs_int(3, i);
	follower_pose_msgs[i].pose.orientation.w = qfu(0, i);
	follower_pose_msgs[i].pose.orientation.x = qfu(1, i);
	follower_pose_msgs[i].pose.orientation.y = qfu(2, i);
	follower_pose_msgs[i].pose.orientation.z = qfu(3, i);*/
	follower_pose_msgs[i].pose.position.x = ufs_int[i](1);
	follower_pose_msgs[i].pose.position.y = ufs_int[i](2);
	follower_pose_msgs[i].pose.position.z = ufs_int[i](3);
	follower_pose_msgs[i].pose.orientation.w = qfu[i](0);
	follower_pose_msgs[i].pose.orientation.x = qfu[i](1);
	follower_pose_msgs[i].pose.orientation.y = qfu[i](2);
	follower_pose_msgs[i].pose.orientation.z = qfu[i](3);

	// Leave velocity as zero for now

	follower_pose_msgs[i].header.stamp = this->now();
	follower_vel_msgs[i].header.stamp = this->now();
	follower_pose_msgs[i].header.frame_id = "world";
	follower_vel_msgs[i].header.frame_id = "world";

	follower_pose_publishers[i]->publish(follower_pose_msgs[i]);
	//follower_vel_publishers[i]->publish(follower_vel_msgs[i]);
      
      }
    }

    void follower_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int i){
      // Get follower position and quaternion
      /*dfs(1, i) = msg->pose.position.x;
      dfs(2, i) = msg->pose.position.y;
      dfs(3, i) = msg->pose.position.z;
      qfs(0, i) = msg->pose.orientation.w;
      qfs(1, i) = msg->pose.orientation.x;
      qfs(2, i) = msg->pose.orientation.y;
      qfs(3, i) = msg->pose.orientation.z;
      qfs_conj(0, i) = qfs(0, i);
      qfs_conj(1, i) = -qfs(1, i);
      qfs_conj(2, i) = -qfs(2, i);
      qfs_conj(3, i) = -qfs(3, i);*/
      dfs[i](1) = msg->pose.position.x;
      dfs[i](2) = msg->pose.position.y;
      dfs[i](3) = msg->pose.position.z;
      qfs[i](0) = msg->pose.orientation.w;
      qfs[i](1) = msg->pose.orientation.x;
      qfs[i](2) = msg->pose.orientation.y;
      qfs[i](3) = msg->pose.orientation.z;
      qfs_conj[i](0) = qfs[i](0);
      qfs_conj[i](1) = -qfs[i](1);
      qfs_conj[i](2) = -qfs[i](2);
      qfs_conj[i](3) = -qfs[i](3);
      //std::cout << "dfs num " << i << ": " << dfs[i](1) << ", " << dfs[i](2) << ", " << dfs[i](3) << std::endl;
    }
    void follower_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg, int i){
      // Get follower velocity
      /*vfs(0, i) = msg->twist.linear.x;
      vfs(1, i) = msg->twist.linear.y;
      vfs(2, i) = msg->twist.linear.z;
      vfs(3, i) = msg->twist.angular.z;*/
      vfs[i](0) = msg->twist.linear.x;
      vfs[i](1) = msg->twist.linear.y;
      vfs[i](2) = msg->twist.linear.z;
      vfs[i](3) = msg->twist.angular.z;
    }
    void leader_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      // Get leader position and quaternion
      dl(1) = msg->pose.position.x;
      dl(2) = msg->pose.position.y;
      dl(3) = msg->pose.position.z;
      ql(0) = msg->pose.orientation.w;
      ql(1) = msg->pose.orientation.x;
      ql(2) = msg->pose.orientation.y;
      ql(3) = msg->pose.orientation.z;
      ql_conj(0) = ql(0);
      ql_conj(1) = -ql(1);
      ql_conj(2) = -ql(2);
      ql_conj(3) = -ql(3);
    }
    void formation_definition_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      // Get formation definition
      for (int i = 0; i < num_drones; i++){
	/*gammasd(1, i) = msg->poses[i].position.x;
	gammasd(2, i) = msg->poses[i].position.y;
	gammasd(3, i) = msg->poses[i].position.z;
	qdfs(0, i)    = msg->poses[i].orientation.w;
	qdfs(1, i)    = msg->poses[i].orientation.x;
	qdfs(2, i)    = msg->poses[i].orientation.y;
	qdfs(3, i)    = msg->poses[i].orientation.z;*/
	gammasd[i](1) = msg->poses[i].position.x;
	gammasd[i](2) = msg->poses[i].position.y;
	gammasd[i](3) = msg->poses[i].position.z;
	qdfs[i](0)    = msg->poses[i].orientation.w;
	qdfs[i](1)    = msg->poses[i].orientation.x;
	qdfs[i](2)    = msg->poses[i].orientation.y;
	qdfs[i](3)    = msg->poses[i].orientation.z;
	//std::cout << "Gammasd num " << i << ": " << gammasd[i](1) << ", " << gammasd[i](2) << ", " << gammasd[i](3) << std::endl;
      }
    }


  private:
    int num_drones; // Number of agents (leader is not included) 

    Eigen::Vector4d              dl;               // World position of the leader [0, x, y, z]^T
    Eigen::Vector4d              vl;               // World velocity of the leader [x_dot, y_dot, z_dot, yaw_dot]^T
    Eigen::Vector4d              ql;               // Leader quaternion [w, x, y, z]^T
    Eigen::Vector4d              ql_conj;          // Leader quaternion conjugate [w, -x, -y, -z]^T
    Eigen::Vector4d              ql_dot;           // Leader quaternion derivative [w_dot, x_dot, y_dot, z_dot]^T
    Eigen::Vector4d              omegal;           // Leader angular velocity [0, x_dot, y_dot, z_dot]^T
    std::vector<Eigen::Vector4d> qfs;         // Followers quaternions [w, x, y, z]^T 
    std::vector<Eigen::Vector4d> qfs_conj;	  // Followers quaternions conjugate [w, -x, -y, -z]^T
    std::vector<Eigen::Vector4d> dfs;         // World positions of the followers
    std::vector<Eigen::Vector4d> vfs;         // World velocities of the followers [x_dot, y_dot, z_dot, yaw_dot]^T
    std::vector<Eigen::Vector4d> gammas;      // Follower position with respect to the leader (Body)
    std::vector<Eigen::Vector4d> gammas_dot;  // Follower velocity with respect to the leader (Body)
    std::vector<Eigen::Vector4d> lambdas;     // Follower position with respect to the leader (World)
    std::vector<Eigen::Vector4d> lambdas_dot; // Follower velocity with respect to the leader (World)
    std::vector<Eigen::Vector4d> gammasd;     // Desired follower position with respect to the leader (Body)
    std::vector<Eigen::Vector4d> gammasd_dot; // Desired follower velocity with respect to the leader (Body)
    std::vector<Eigen::Vector4d> qdfs;        // Desired follower quaternion with respect to the leader
    std::vector<Eigen::Vector4d> etas;        // QLM of the followers
    std::vector<Eigen::Vector4d> efs;         // Formation error [x, y, z, qlm(3)]^T
    std::vector<Eigen::Vector4d> efs_prev;    // Formation error previous
    std::vector<Eigen::Vector4d> efs_int;     // Formation error integral
    std::vector<Eigen::Vector4d> k;           // Formation sliding gain [kx, ky, kz, kpsi]^T
    std::vector<Eigen::Vector4d> kappa1;      // Formation constant effort gain
    std::vector<Eigen::Vector4d> kappa2;      // Formation smooth effort gain
    std::vector<Eigen::Vector4d> sigmaf;      // Formation sliding surface 
    std::vector<Eigen::Vector4d> uauxs;       // Formation auxiliar control output
    std::vector<Eigen::Vector4d> ufs;         // Formation control output
    std::vector<Eigen::Vector4d> ufs_prev;    // Formation control output previous
    std::vector<Eigen::Vector4d> ufs_int;     // Formation control output integral [x, y, z, psi]^T (Psi unused)
    std::vector<Eigen::Vector4d> omegafu;     // Formation output angular velocity [0, 0, 0, ufs(4)]^T
    std::vector<Eigen::Vector4d> qfu;         // Formation output quaternion [w, x, y, z]^T
    std::vector<Eigen::Vector4d> qfu_dot;     // Formation output quaternion derivative [w_dot, x_dot, y_dot, z_dot]^T 
    std::vector<Eigen::Vector4d> qfu_dot_prev;// Formation output quaternion derivative previous 
    std::vector<Eigen::Vector4d> qfu_conj;    // Formation output quaternion conjugate [w, -x, -y, -z]^T

    std::vector<geometry_msgs::msg::PoseStamped> follower_pose_msgs; // For publishing
    std::vector<geometry_msgs::msg::TwistStamped> follower_vel_msgs; // For publishing

    rclcpp::TimerBase::SharedPtr control_timer;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>  follower_pose_subscriptions;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> follower_vel_subscriptions;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>     follower_pose_publishers;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>    follower_vel_publishers;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  		   leader_pose_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr  		   formation_definition_subscription;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficControl>());
  rclcpp::shutdown();
  return 0;
}
