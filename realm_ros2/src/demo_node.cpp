
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include <mutex>
#include <unordered_map>
#include <string>

//#include <tf/transform_broadcaster.h>


// #include <OpenREALM/realm_core/structs.h>
// #include <OpenREALM/realm_core/camera_settings_factory.h>
// #include <OpenREALM/realm_vslam_base/visual_slam_settings_factory.h>
// #include <OpenREALM/realm_densifier_base/densifier_settings_factory.h>
// #include <OpenREALM/realm_io/utilities.h>

// #include <OpenREALM/realm_stages/stage_settings_factory.h>
// #include <OpenREALM/realm_stages/pose_estimation.h>
// #include <OpenREALM/realm_stages/densification.h>
// #include <OpenREALM/realm_stages/surface_generation.h>
// #include <OpenREALM/realm_stages/ortho_rectification.h>
// #include <OpenREALM/realm_stages/mosaicing.h>
// #include <OpenREALM/realm_stages/tileing.h>

// #include <std_msgs/String.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <realm_ros/conversions.h>
// #include <realm_msgs/Frame.h>
// #include <realm_msgs/CvGridMap.h>
// #include <realm_msgs/GroundImageCompressed.h>

// #include <std_srvs/Trigger.h>
// #include <realm_msgs/ParameterChange.h> 


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("demo_node"), count_(0)
    {

        readParams();
        
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:

    void readParams() 
    {
        this->declare_parameter("type", std::string("uninitalized"));
        type_ = this->get_parameter("type").as_string();

        this->declare_parameter("master", false);
        is_master_ = this->get_parameter("master").as_bool();

        // Read parameters from launch file
        //ros::NodeHandle param_nh("~");
        this->declare_parameter("stage/type", std::string("uninitialised"));
        _type_stage = this->get_parameter("stage/type").as_string();
        
        this->declare_parameter("stage/master", false);
        _is_master_stage = this->get_parameter("stage/master").as_bool();

        this->declare_parameter("stage/output_dir", std::string("uninitialised"));
        _path_stage_output = this->get_parameter("stage/output_dir").as_string();

        this->declare_parameter("topics/input/frame", std::string("uninitialised"));
        _topic_frame_in = this->get_parameter("topics/input/frame").as_string();

        this->declare_parameter("topics/input/imu", std::string("uninitialised"));
        _topic_imu_in = this->get_parameter("topics/input/imu").as_string();

        this->declare_parameter("topics/output", std::string("uninitialised"));
        _topic_frame_out = this->get_parameter("topics/output").as_string();

        this->declare_parameter("config/id", std::string("uninitialised"));
        _id_camera = this->get_parameter("config/id").as_string();

        this->declare_parameter("config/profile", std::string("uninitialised"));
        _profile = this->get_parameter("config/profile").as_string();

        this->declare_parameter("config/method", std::string("uninitialised"));
        _method = this->get_parameter("config/method").as_string();

        this->declare_parameter("config/opt/working_directory", std::string("uninitialised"));
        _path_working_directory = this->get_parameter("config/opt/working_directory").as_string();

        this->declare_parameter("config/opt/output_directory", std::string("uninitialised"));
        _path_output = this->get_parameter("config/opt/output_directory").as_string();

        /* Set specific config file paths
        if (_profile == "uninitialised")
          throw(std::invalid_argument("Error: Stage settings profile must be provided in launch file."));
        if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
          throw(std::invalid_argument("Error: Working directory does not exist!"));
        */

    }

    void setPaths()
    {
        if (_path_working_directory == "uninitialised")
          //_path_working_directory = ros::package::getPath("realm_ros2");
          _path_working_directory = "/home/azam/realm_ros2/src/OpenREALM-ros2/realm_ros2";
          
          _path_profile = _path_working_directory + "/profiles/" + _profile;

        if (_path_output == "uninitialised")
          _path_output = _path_working_directory + "/output";

        // Set settings filepaths
        _file_settings_camera = _path_profile + "/camera/calib.yaml";
        _file_settings_imu    = _path_profile + "/config/imu.yaml";
        _file_settings_stage = _path_profile + "/" + _type_stage + "/stage_settings.yaml";
        _file_settings_method = _path_profile + "/" + _type_stage + "/method/" + _method + "_settings.yaml";

        if (!io::dirExists(_path_profile))
          throw(std::runtime_error("Error: Profile folder '" + _path_profile + "' was not found!"));
        if (!io::dirExists(_path_output))
          io::createDir(_path_output);

        // Master priviliges
        //if (_is_master_stage)
        //{
          // Create sub directory with timestamp
          //_dir_date_time = io::getDateTime();
          //if (!io::dirExists(_path_output + "/" + _dir_date_time))
          //  io::createDir(_path_output + "/" + _dir_date_time);
        //}
    }




    void timer_callback()
    {
      auto message = std_msgs::msg::String();

      if (is_master_){
        message.data = "Running master program " + std::to_string(count_++);
      } else {
        message.data = "Running worker program " + std::to_string(count_++);
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

    std::string _type_stage;
    bool _is_master_stage;
    std::string _path_stage_output;
    std::string _topic_frame_in;
    std::string _topic_imu_in;
    std::string _topic_frame_out;
    std::string _id_camera;
    std::string _profile;
    std::string _method;
    std::string _path_working_directory;
    std::string _path_output;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}