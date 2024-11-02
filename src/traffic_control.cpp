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

//Eigen::Vector4d cross4(Eigen::Vector4d a, Eigen::Vector4d b){
Eigen::Vector4d cross4(Eigen::Vector4d a, Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr b){
  // Calculate cross product of a and b, where both are [0, x, y, z]^T
  // Result is [0, a x b]^T
  Eigen::Vector4d c;
  c(1) = a(2)*b(3) - a(3)*b(2);
  c(2) = a(3)*b(1) - a(1)*b(3);
  c(3) = a(1)*b(2) - a(2)*b(1);
  return c;
}

Eigen::Vector4d sig(Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr v, double exponent){
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
}

// Quick and dirty quaternion multiplication for matrix slices
Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::DenseBase<Eigen::Matrix<double, 4, -1> >::ColXpr p){
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
}


class TrafficControl : public rclcpp::Node{
  public:
    TrafficControl(): Node("traffic_control_node"){
      // Get drone_id parameter
      this->declare_parameter("num_drones", 0);
      num_drones = this->get_parameter("num_drones").as_int();
      
      // Make 0.5s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&TrafficControl::control_callback, this));
  
      // Initialize variables
      Eigen::Vector4d zero4d = Eigen::Vector4d::Zero();
      dl     = zero4d;
      vl     = zero4d;
      omegal = zero4d;
      ql_dot = zero4d; 
      ql     << 1, 0, 0, 0; 

      Eigen::Matrix<double, 4, Eigen::Dynamic> zero4d_dyn = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, num_drones); 
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
      qfu         = zero4d_dyn;

      //Eigen::Vector<double, num_drones> ones_dyn = Eigen::Vector<double, num_drones>::Ones();
      Eigen::Vector<double, Eigen::Dynamic> ones_dyn = Eigen::Vector<double, Eigen::Dynamic>::Ones(num_drones);

      // Set all columns of quaternion matrices to identity ( qfs, qfs_conj, qdfs, qfu)
      qfs.row(0)      = ones_dyn;
      qfs_conj.row(0) = ones_dyn;
      qdfs.row(0)     = ones_dyn;
      qfu.row(0)       = ones_dyn;

      // Initialize subscribers and publishers
      for (int i = 0; i < num_drones; i++){
	std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> follower_proto = std::bind(&TrafficControl::follower_pose_callback, this, std::placeholders::_1, i);
	follower_pose_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/estimator/pose", 10, follower_proto));
	std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)> follower_vel_proto = std::bind(&TrafficControl::follower_vel_callback, this, std::placeholders::_1, i);	
	follower_vel_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/estimator/velocity", 10, follower_vel_proto));

	//follower_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/reference/pose", 10));
	//follower_vel_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/reference/velocity", 10));
	follower_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/echo/pose", 10));
	follower_vel_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/echo/velocity", 10));
	
	follower_pose_msgs.push_back(geometry_msgs::msg::PoseStamped());
	follower_vel_msgs.push_back(geometry_msgs::msg::TwistStamped());
      }
    }

    void control_callback(){
      for (int i = 0; i < num_drones; i++){
	// Calculate gammas and lambdas
	lambdas.col(i) = dfs.col(i) - dl; // Both are [0, x, y, z]^T
	lambdas_dot.col(i) = vfs.col(i) - vl; // Both are [x_dot, y_dot, z_dot, yaw_dot]^T
	gammas.col(i) = kronecker(kronecker(ql, lambdas.col(i)), ql_conj); 
	gammas_dot.col(i) = cross4(omegal, gammas.col(i)) + kronecker(kronecker(ql, lambdas_dot.col(i)), ql_conj);

	// Calculate errors and etas
	//etas.col(i) = qlm(qfs_conj.col(i), kronecker(ql, qdfs.col(i)));
	etas.col(i) = qlm(kronecker(qfs_conj.col(i), kronecker(ql, qdfs.col(i))));
	// Fill efs elementwise
	efs(0, i) = gammasd(1, i) - gammas(1, i);
	efs(1, i) = gammasd(2, i) - gammas(2, i);
	efs(2, i) = gammasd(3, i) - gammas(3, i);
	efs(3, i) = etas(3, i);

	// Compute sliding surface
	// efs trapezoidal integral
	efs_int.col(i) += 0.5*(efs.col(i) + efs_prev.col(i));
	efs_prev.col(i) = efs.col(i);

	sigmaf.col(i) = efs.col(i) + k.col(i).cwiseProduct(efs_int.col(i));
	
	// Compute auxiliar control output (Continuous non-adaptive smc)
	uauxs.col(i) = -kappa1.col(i).cwiseProduct(sig(sigmaf.col(i), 0.5)) - kappa2.col(i).cwiseProduct(sigmaf.col(i));
	
	// Epitelos, compute total control output
	Eigen::Vector4d c13 = k.col(i).cwiseProduct(efs.col(i)) + uauxs.col(i);
	Eigen::Vector4d v13;
	v13(0) = 0;
	v13(1) = vl(0);
	v13(2) = vl(1);
	v13(3) = vl(2);
	// Reorder elements for quaternion multiplication
	omegafu(3, i) = c13(3);
        c13(3) = c13(2);
	c13(2) = c13(1);
	c13(1) = c13(0);
	c13(0) = 0;
	// Bullshit term because Eigen .col() is trash
	Eigen::Vector4d bullshit;
	bullshit = gammas_dot.col(i) + c13 - cross4(omegal, gammas.col(i));
	//ufs.col(i) = kronecker(kronecker(qfs_conj.col(i), v13 + kronecker(kronecker( ql_conj, gammas_dot.col(i) + c13 - cross4(omegal, gammas.col(i))), ql)), qfs.col(i));
	ufs.col(i) = kronecker(kronecker(qfs_conj.col(i), v13 + kronecker(kronecker( ql_conj, bullshit ), ql)), qfs.col(i));
	// Ufs should be [0, x, y, z]^T
	// Map omegafu to qfu_dot, then trapezoidal integral
	qfu_dot.col(i) = 0.5*kronecker(qfu.col(i), omegafu.col(i));
	qfu.col(i) += 0.5*(qfu_dot.col(i) + qfu_dot_prev.col(i));
	qfu_dot_prev.col(i) = qfu_dot.col(i);

	// ufs trapezoidal integral
	ufs_int.col(i) += 0.5*(ufs.col(i) + ufs_prev.col(i));
	ufs_prev.col(i) = ufs.col(i);

	// Publish integrals
	follower_pose_msgs[i].pose.position.x = ufs_int(1, i);
	follower_pose_msgs[i].pose.position.y = ufs_int(2, i);
	follower_pose_msgs[i].pose.position.z = ufs_int(3, i);
	follower_pose_msgs[i].pose.orientation.w = qfu(0, i);
	follower_pose_msgs[i].pose.orientation.x = qfu(1, i);
	follower_pose_msgs[i].pose.orientation.y = qfu(2, i);
	follower_pose_msgs[i].pose.orientation.z = qfu(3, i);

	// Leave velocity as zero for now

	follower_pose_msgs[i].header.stamp = this->now();
	follower_vel_msgs[i].header.stamp = this->now();
	follower_pose_msgs[i].header.frame_id = "world";
	follower_vel_msgs[i].header.frame_id = "world";

	follower_pose_publishers[i]->publish(follower_pose_msgs[i]);
	follower_vel_publishers[i]->publish(follower_vel_msgs[i]);
      
      }
    }

    void follower_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int i){
      // Get follower position and quaternion
      dfs(1, i) = msg->pose.position.x;
      dfs(2, i) = msg->pose.position.y;
      dfs(3, i) = msg->pose.position.z;
      qfs(0, i) = msg->pose.orientation.w;
      qfs(1, i) = msg->pose.orientation.x;
      qfs(2, i) = msg->pose.orientation.y;
      qfs(3, i) = msg->pose.orientation.z;
      qfs_conj(0, i) = qfs(0, i);
      qfs_conj(1, i) = -qfs(1, i);
      qfs_conj(2, i) = -qfs(2, i);
      qfs_conj(3, i) = -qfs(3, i);
    }
    void follower_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg, int i){
      // Get follower velocity
      vfs(0, i) = msg->twist.linear.x;
      vfs(1, i) = msg->twist.linear.y;
      vfs(2, i) = msg->twist.linear.z;
      vfs(3, i) = msg->twist.angular.z;
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
	gammasd(1, i) = msg->poses[i].position.x;
	gammasd(2, i) = msg->poses[i].position.y;
	gammasd(3, i) = msg->poses[i].position.z;
	qdfs(0, i)    = msg->poses[i].orientation.w;
	qdfs(1, i)    = msg->poses[i].orientation.x;
	qdfs(2, i)    = msg->poses[i].orientation.y;
	qdfs(3, i)    = msg->poses[i].orientation.z;
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
    Eigen::Matrix<double, 4, Eigen::Dynamic> qfs;         // Followers quaternions [w, x, y, z]^T 
    Eigen::Matrix<double, 4, Eigen::Dynamic> qfs_conj;	  // Followers quaternions conjugate [w, -x, -y, -z]^T
    Eigen::Matrix<double, 4, Eigen::Dynamic> dfs;         // World positions of the followers
    Eigen::Matrix<double, 4, Eigen::Dynamic> vfs;         // World velocities of the followers [x_dot, y_dot, z_dot, yaw_dot]^T
    Eigen::Matrix<double, 4, Eigen::Dynamic> gammas;      // Follower position with respect to the leader (Body)
    Eigen::Matrix<double, 4, Eigen::Dynamic> gammas_dot;  // Follower velocity with respect to the leader (Body)
    Eigen::Matrix<double, 4, Eigen::Dynamic> lambdas;     // Follower position with respect to the leader (World)
    Eigen::Matrix<double, 4, Eigen::Dynamic> lambdas_dot; // Follower velocity with respect to the leader (World)
    Eigen::Matrix<double, 4, Eigen::Dynamic> gammasd;     // Desired follower position with respect to the leader (Body)
    Eigen::Matrix<double, 4, Eigen::Dynamic> gammasd_dot; // Desired follower velocity with respect to the leader (Body)
    Eigen::Matrix<double, 4, Eigen::Dynamic> qdfs;        // Desired follower quaternion with respect to the leader
    Eigen::Matrix<double, 4, Eigen::Dynamic> etas;        // QLM of the followers
    Eigen::Matrix<double, 4, Eigen::Dynamic> efs;         // Formation error [x, y, z, qlm(3)]^T
    Eigen::Matrix<double, 4, Eigen::Dynamic> efs_prev;    // Formation error previous
    Eigen::Matrix<double, 4, Eigen::Dynamic> efs_int;     // Formation error integral
    Eigen::Matrix<double, 4, Eigen::Dynamic> k;           // Formation sliding gain [kx, ky, kz, kpsi]^T
    Eigen::Matrix<double, 4, Eigen::Dynamic> kappa1;      // Formation constant effort gain
    Eigen::Matrix<double, 4, Eigen::Dynamic> kappa2;      // Formation smooth effort gain
    Eigen::Matrix<double, 4, Eigen::Dynamic> sigmaf;      // Formation sliding surface 
    Eigen::Matrix<double, 4, Eigen::Dynamic> uauxs;       // Formation auxiliar control output
    Eigen::Matrix<double, 4, Eigen::Dynamic> ufs;         // Formation control output
    Eigen::Matrix<double, 4, Eigen::Dynamic> ufs_prev;    // Formation control output previous
    Eigen::Matrix<double, 4, Eigen::Dynamic> ufs_int;     // Formation control output integral [x, y, z, psi]^T (Psi unused)
    Eigen::Matrix<double, 4, Eigen::Dynamic> omegafu;     // Formation output angular velocity [0, 0, 0, ufs(4)]^T
    Eigen::Matrix<double, 4, Eigen::Dynamic> qfu;         // Formation output quaternion [w, x, y, z]^T
    Eigen::Matrix<double, 4, Eigen::Dynamic> qfu_dot;     // Formation output quaternion derivative [w_dot, x_dot, y_dot, z_dot]^T 
    Eigen::Matrix<double, 4, Eigen::Dynamic> qfu_dot_prev;// Formation output quaternion derivative previous 

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
