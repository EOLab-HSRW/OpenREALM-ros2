
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("demo_node"), count_(0)
    {

        readParam();
        
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:

    void readParam() 
    {
        this->declare_parameter("type", std::string("uninitalized"));
        type_ = this->get_parameter("type").as_string();

        this->declare_parameter("master", false);
        is_master_ = this->get_parameter("master").as_bool();
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();

      if (is_master_){
        message.data = "Running master program";
      } else {
        message.data = "Running worker program";
      }
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    // ros parameter
    std::string type_;
    bool is_master_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}