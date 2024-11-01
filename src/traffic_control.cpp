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

using namespace std::chrono_literals;

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
      ql     = zero4d;
      ql_dot = zero4d;
      omegal = zero4d;

      Eigen::Matrix<double, 4, Dynamic> zero4d_dyn = Eigen::Matrix<double, 4, Dynamic>::Zero(4, num_drones); 
      qfs         = zero4d_dyn;
      dfs         = zero4d_dyn;
      vfs         = zero4d_dyn;
      gammas      = zero4d_dyn;
      gammas_dot  = zero4d_dyn;
      lambdas     = zero4d_dyn;
      lambdas_dot = zero4d_dyn;
      gammasd     = zero4d_dyn;
      gammasd_dot = zero4d_dyn;
      qdfs        = zero4d_dyn;
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
      qfu         = zero4d_dyn;

      // Initialize subscribers and publishers
      for (int i = 0; i < num_drones; i++){
	std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> follower_proto = std::bind(&TrafficControl::follower_pose_callback, this, std::placeholders::_1, i);
	follower_pose_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/estimator/pose", follower_proto));
	std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)> follower_vel_proto = std::bind(&TrafficControl::follower_vel_callback, this, std::placeholders::_1, i);	
	follower_vel_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/estimator/velocity", follower_vel_proto));

	follower_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("/tello_" + std::to_string(i) + "/tello/reference/pose", 10));
	follower_vel_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>("/tello_" + std::to_string(i) + "/tello/reference/velocity", 10));
      }
    }

    void control_callback(){
    }

    void follower_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int i){}
    void follower_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg, int i){}
    void leader_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){}
    void formation_definition_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){}


  private:
    int num_drones; // Number of agents (leader is not included) 

    Eigen::Vector4d              dl;          // World position of the leader [0, x, y, z]^T
    Eigen::Vector4d              vl;          // World velocity of the leader [x_dot, y_dot, z_dot, yaw_dot]^T
    Eigen::Vector4d              ql;          // Leader quaternion [w, x, y, z]^T
    Eigen::Vector4d              ql_dot;      // Leader quaternion derivative [w_dot, x_dot, y_dot, z_dot]^T
    Eigen::Vector4d              omegal;      // Leader angular velocity [0, x_dot, y_dot, z_dot]^T
    Eigen::Matrix<double, 4, Dynamic> qfs;         // Followers quaternions [w, x, y, z]^T 
    Eigen::Matrix<double, 4, Dynamic> dfs;         // World positions of the followers
    Eigen::Matrix<double, 4, Dynamic> vfs;         // World velocities of the followers
    Eigen::Matrix<double, 4, Dynamic> gammas;      // Follower position with respect to the leader (Body)
    Eigen::Matrix<double, 4, Dynamic> gammas_dot;  // Follower velocity with respect to the leader (Body)
    Eigen::Matrix<double, 4, Dynamic> lambdas;     // Follower position with respect to the leader (World)
    Eigen::Matrix<double, 4, Dynamic> lambdas_dot; // Follower velocity with respect to the leader (World)
    Eigen::Matrix<double, 4, Dynamic> gammasd;     // Desired follower position with respect to the leader (Body)
    Eigen::Matrix<double, 4, Dynamic> gammasd_dot; // Desired follower velocity with respect to the leader (Body)
    Eigen::Matrix<double, 4, Dynamic> qdfs;        // Desired follower quaternion with respect to the leader
    Eigen::Matrix<double, 4, Dynamic> etas;        // QLM of the followers
    Eigen::Matrix<double, 4, Dynamic> efs;         // Formation error [x, y, z, qlm(3)]^T
    Eigen::Matrix<double, 4, Dynamic> efs_prev;    // Formation error previous
    Eigen::Matrix<double, 4, Dynamic> efs_int;     // Formation error integral
    Eigen::Matrix<double, 4, Dynamic> k;           // Formation sliding gain [kx, ky, kz, kpsi]^T
    Eigen::Matrix<double, 4, Dynamic> kappa1;      // Formation constant effort gain
    Eigen::Matrix<double, 4, Dynamic> kappa2;      // Formation smooth effort gain
    Eigen::Matrix<double, 4, Dynamic> sigmaf;      // Formation sliding surface 
    Eigen::Matrix<double, 4, Dynamic> ufs;         // Formation control output
    Eigen::Matrix<double, 4, Dynamic> ufs_prev;    // Formation control output previous
    Eigen::Matrix<double, 4, Dynamic> ufs_int;     // Formation control output integral [x, y, z, psi]^T (Psi unused)
    Eigen::Matrix<double, 4, Dynamic> omegafu;     // Formation output angular velocity [0, 0, 0, ufs(4)]^T
    Eigen::Matrix<double, 4, Dynamic> qfu;         // Formation output quaternion [w, x, y, z]^T

    rclcpp::TimerBase::SharedPtr control_timer;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>  follower_pose_subscriptions;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> follower_vel_subscriptions;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>     follower_pose_publishers;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>    follower_vel_publishers;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>  		   leader_pose_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr>  		   formation_definition_subscription;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficControl>());
  rclcpp::shutdown();
  return 0;
}
