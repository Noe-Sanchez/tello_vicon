#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

class FormationsHandler : public rclcpp::Node{
  public:
    FormationsHandler(): Node("formations_node"){
      centroid_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("centroid", 10, std::bind(&FormationsHandler::centroid_callback, this, std::placeholders::_1));
      formation_definition_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>("formation_definition", 10, std::bind(&FormationsHandler::formation_definition_callback, this, std::placeholders::_1));

      transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      transform_timer = this->create_wall_timer(1ms, std::bind(&FormationsHandler::transform_timer_callback, this));

      // Initial formation
      centroid.pose.position.x = 0;
      centroid.pose.position.y = 0;
      centroid.pose.position.z = 0.5;
      centroid.pose.orientation.x = 0;
      centroid.pose.orientation.y = 0;
      centroid.pose.orientation.z = 0;
      centroid.pose.orientation.w = 1;

      this->declare_parameter("num_drones", 3);
      int num_drones = this->get_parameter("num_drones").as_int();
      std::cout << "Broadcasting configurations for " << num_drones << " followers" << std::endl;

      for (int i = 0; i < num_drones; i++){ 
	individual_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("tello_" + std::to_string(i) + "/tello/reference/pose", 10));
        geometry_msgs::msg::Pose pose;
        geometry_msgs::msg::PoseStamped pose_stamped;
        if (num_drones % 2 != 0){
          pose.position.x = cos(((2 * M_PI * i) / num_drones) + (M_PI / (2 * num_drones)));
          pose.position.y = sin(((2 * M_PI * i) / num_drones) + (M_PI / (2 * num_drones)));
        } else {
          pose.position.x = cos(((2 * M_PI * i) / num_drones) + (M_PI / num_drones)); 
          pose.position.y = sin(((2 * M_PI * i) / num_drones) + (M_PI / num_drones));
        }
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        formation_definition.poses.push_back(pose);
	pose_stamped.pose = pose;
	individual_publishers[i]->publish(pose_stamped);
      }
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      this->transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      std::cout << "Broadcast started" << std::endl;

    }
    
    void transform_timer_callback(){
      // Broadcast centroid
      geometry_msgs::msg::TransformStamped centroid_transform;
      centroid_transform.header.stamp = this->now();
      centroid_transform.header.frame_id = "world";
      centroid_transform.child_frame_id = "centroid";
      centroid_transform.transform.translation.x = centroid.pose.position.x;
      centroid_transform.transform.translation.y = centroid.pose.position.y;
      centroid_transform.transform.translation.z = centroid.pose.position.z;
      centroid_transform.transform.rotation = centroid.pose.orientation;
      transform_broadcaster->sendTransform(centroid_transform);

      // Broadcast formation
      //for (int i = 0; i < (int)formation_definition.poses.size(); i++){
      for (int i = 0; i < this->get_parameter("num_drones").as_int(); i++){ 
        geometry_msgs::msg::TransformStamped formation_transform;
        formation_transform.header.stamp = this->now();
        formation_transform.header.frame_id = "centroid";
        formation_transform.child_frame_id = "follower_" + std::to_string(i);
        formation_transform.transform.translation.x = formation_definition.poses[i].position.x;
        formation_transform.transform.translation.y = formation_definition.poses[i].position.y;
        formation_transform.transform.translation.z = formation_definition.poses[i].position.z;
        formation_transform.transform.rotation = formation_definition.poses[i].orientation;
        transform_broadcaster->sendTransform(formation_transform);
        try{
	  formation_transform = tf_buffer->lookupTransform("world", "follower_" + std::to_string(i), rclcpp::Time(0)); 
	} catch (tf2::TransformException &ex){
	  std::cout << "Could not find transform for follower " << i << std::endl;
	  continue;
	}
	//follower.pose = formation_definition.poses[i];
	follower.pose.position.x = formation_transform.transform.translation.x;
	follower.pose.position.y = formation_transform.transform.translation.y;
	follower.pose.position.z = formation_transform.transform.translation.z;
	follower.pose.orientation = formation_transform.transform.rotation;
	follower.pose.orientation.x = formation_transform.transform.rotation.x;
	follower.pose.orientation.y = formation_transform.transform.rotation.y;
	follower.pose.orientation.z = formation_transform.transform.rotation.z;
	follower.pose.orientation.w = formation_transform.transform.rotation.w;
	std::cout << "Broadcasting follower " << i << std::endl;
	individual_publishers[i]->publish(follower);
      }
    }

    void centroid_callback(const geometry_msgs::msg::PoseStamped::SharedPtr centroid){
      this->centroid = *centroid;
    }
    void formation_definition_callback(const geometry_msgs::msg::PoseArray::SharedPtr formation_definition){
      this->formation_definition = *formation_definition;
    }

  private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr centroid_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr formation_definition_subscriber;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    rclcpp::TimerBase::SharedPtr transform_timer;

    // First transform is the centroid, rest are the formation
    std::vector<geometry_msgs::msg::TransformStamped> formation;
    geometry_msgs::msg::PoseStamped centroid;
    geometry_msgs::msg::PoseStamped follower;
    geometry_msgs::msg::PoseArray formation_definition;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> individual_publishers;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationsHandler>());
  rclcpp::shutdown();
  return 0;
}
