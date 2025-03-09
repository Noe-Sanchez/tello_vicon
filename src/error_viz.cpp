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
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class ErrorViz : public rclcpp::Node{
  public:
    ErrorViz(): Node("error_viz_node"){
      //reference_velocity_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>("tello/reference/velocity", 10, std::bind(&ErrorViz::velocity_reference_callback, this, std::placeholders::_1));
      //uaux_publisher    = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);

      viz_timer = this->create_wall_timer(50ms, std::bind(&ErrorViz::viz_callback, this));
      //viz_timer = this->create_wall_timer(500ms, std::bind(&ErrorViz::viz_callback, this));
  
      // Declare num_drones parameter
      this->declare_parameter("num_drones", 1);
      this->get_parameter("num_drones", num_drones);

      // Fill drone_positions with zeros 
      //for (int i = 0; i < num_markers; i++){
      //	drone_errors.push_back(Eigen::Vector3d(0, 0, 0));
      //}

      // Declare drone pose subscriptions
      for(int i = 0; i < num_drones; i++){
	std::function <void(const geometry_msgs::msg::PoseStamped::SharedPtr)> cb = std::bind(&ErrorViz::drone_error_callback, this, std::placeholders::_1, i);
	drone_error_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("tello_" + std::to_string(i) + "/tello/formation/error", 10, cb));
	drone_errors.push_back(geometry_msgs::msg::PoseStamped());
      } 
      drone_vec_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("tellos/error_viz", 10);
    }

    void drone_error_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int index){
      drone_errors[index] = *msg;
      //std::cout << "Drone " << index << " error: " << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << std::endl;
    }

    void viz_callback(){
      this->get_parameter("num_drones", num_drones);
      drone_vec.markers.clear();
      for (int i = 0; i < num_drones; i++){      
	visualization_msgs::msg::Marker marker;
	geometry_msgs::msg::Point p;
	marker.points.push_back(p);
	p.x = drone_errors[i].pose.position.x; 
	p.y = drone_errors[i].pose.position.y;
	p.z = drone_errors[i].pose.position.z;
	//std::cout << "Drone " << i << " error: " << p.x << ", " << p.y << ", " << p.z << std::endl;
	marker.points.push_back(p);
	marker.header.frame_id = "tello" + std::to_string(i);
	//marker.header.stamp = this->now();
	marker.type = visualization_msgs::msg::Marker::ARROW;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.color.a = 0.25;
	// Change color depending on drone (i variable)
	marker.color.g = ((float)i)/((float)num_drones-1); 
	marker.color.r = 0.2;
	marker.color.b = 1 - ((float)i)/((float)num_drones-1); 
	marker.id = i;
	marker.scale.z = 0.1;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.frame_locked = true;

	drone_vec.markers.push_back(marker);

      }
      drone_vec_publisher->publish(drone_vec);
      //std::cout << "Published marker array for drone " << i << std::endl;
    }

  private:
    std::vector<geometry_msgs::msg::PoseStamped> drone_errors;
    //std::vector<Eigen::Vector3d> drone_errors;
    visualization_msgs::msg::MarkerArray drone_vec;
    
    int num_drones; 

    rclcpp::TimerBase::SharedPtr viz_timer; 
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> drone_error_subscriptions;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr drone_vec_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ErrorViz>());
  rclcpp::shutdown();
  return 0;
}
