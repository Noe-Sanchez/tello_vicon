#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

class TestNode : public rclcpp::Node{
  public:
    TestNode() : Node("test_node"){
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TestNode::timer_callback, this));
        int num_topics = 10;
        for(int i = 0; i < num_topics; i++){
            std::function <void(const std_msgs::msg::String::SharedPtr)> cb = std::bind(&TestNode::sample_topic_callback, this, std::placeholders::_1, i);
            subscriptions_.push_back(this->create_subscription<std_msgs::msg::String>("topic_" + std::to_string(i), 10, cb));
            publishers_.push_back(this->create_publisher<std_msgs::msg::String>("topic_" + std::to_string(i) + "_out", 10));
            data_.push_back("empty");
        }
    }

  private:
    void timer_callback(){
      for(int i = 0; i < (int)data_.size(); i++){
        std_msgs::msg::String msg;
        msg.data = data_[i];
        publishers_[i]->publish(msg);
      }
    }

    void sample_topic_callback(const std_msgs::msg::String::SharedPtr msg, int index){
      data_[index] = msg->data;
    }

    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
    std::vector<std::string> data_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}
 
