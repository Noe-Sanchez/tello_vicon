#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>

// Unix TCP socket includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

typedef struct {
  float x;
  float y;
  float z;
  float q0;
  float q1;
  float q2;
  float q3;
} DronePosition; 


using namespace std::chrono_literals;

class UnityUIWrapper : public rclcpp::Node{
  public:
    UnityUIWrapper(): Node("unity_ui"){
      // Publishers
      centroid_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("centroid", 10);
      formation_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("formation_definition", 10);

      RCLCPP_INFO(this->get_logger(), "Unity UI Wrapper Node Started");

      this->declare_parameter("num_drones", 3);
      num_drones = this->get_parameter("num_drones").as_int();

      drone_positions.resize(num_drones);
      formation.resize(num_drones);

      server_socket = socket(AF_INET, SOCK_STREAM, 0);
      server_address.sin_family = AF_INET;
      server_address.sin_port = htons(18000);
      server_address.sin_addr.s_addr = INADDR_ANY;
      bind(server_socket, (struct sockaddr*)&server_address, sizeof(server_address));

      centroid.pose.position.x = 0;
      centroid.pose.position.y = 0;
      centroid.pose.position.z = 0.5;
      centroid.pose.orientation.x = 0;
      centroid.pose.orientation.y = 0;
      centroid.pose.orientation.z = 0;
      centroid.pose.orientation.w = 1;

      RCLCPP_INFO(this->get_logger(), "Waiting for connection");
      //Wait until a connection is established, blocking call
      listen(server_socket, 1);
      client_socket = accept(server_socket, NULL, NULL);
      RCLCPP_INFO(this->get_logger(), "Connection established");

      while (1){
        recv(client_socket, positions_buffer, 28*(num_drones+1), 0); 
        //printf("Received formation");

        for (int i = 0; i < (num_drones+1); i++){
          drone_positions[i].x = *(float*)(positions_buffer + i*28);
          drone_positions[i].z = *(float*)(positions_buffer + i*28 + 4);
          drone_positions[i].y = *(float*)(positions_buffer + i*28 + 8);
          drone_positions[i].q0 = *(float*)(positions_buffer + i*28 + 12);
          drone_positions[i].q2 = (-1)*(*(float*)(positions_buffer + i*28 + 16));
          drone_positions[i].q1 = *(float*)(positions_buffer + i*28 + 20);
          drone_positions[i].q3 = *(float*)(positions_buffer + i*28 + 24);

          if(i == num_drones){
            centroid.pose.position.x = drone_positions[i].x;
            centroid.pose.position.y = drone_positions[i].y;
            centroid.pose.position.z = drone_positions[i].z;
            centroid.pose.orientation.x = drone_positions[i].q0;
            centroid.pose.orientation.y = drone_positions[i].q1;
            centroid.pose.orientation.z = drone_positions[i].q2;
            centroid.pose.orientation.w = drone_positions[i].q3;
          }else{
            formation[i].position.x = drone_positions[i].x;
            formation[i].position.y = drone_positions[i].y;
            formation[i].position.z = drone_positions[i].z;
            formation[i].orientation.x = drone_positions[i].q0;
            formation[i].orientation.y = drone_positions[i].q1;
            formation[i].orientation.z = drone_positions[i].q2;
            formation[i].orientation.w = drone_positions[i].q3;
          }
        }

        centroid_publisher->publish(centroid);
        formation_msg.poses = formation; 
        formation_publisher->publish(formation_msg);

      }

    }

  private:

    std::vector<geometry_msgs::msg::Pose> formation;
    geometry_msgs::msg::PoseArray formation_msg; 
    geometry_msgs::msg::PoseStamped centroid;

    int num_drones;
    int server_socket;
    int client_socket;
    struct sockaddr_in server_address;
    char positions_buffer[308];
    std::vector<DronePosition> drone_positions;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr centroid_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr formation_publisher; 

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnityUIWrapper>());
  rclcpp::shutdown();
  return 0;
}
