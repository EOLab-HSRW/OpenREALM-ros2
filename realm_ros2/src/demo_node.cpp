
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_srvs/srv/empty.hpp>

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

      // Read basic launch file inputs
      readParams();
      
      // Specify stage
      setPaths();
      //readStageSettings();

      
      // Set naming conventions
      _topic_prefix = "/realm/" + _id_camera + "/" + _type_stage + "/";
      _tf_base_frame_name = "realm_base";
      _tf_stage_frame_name = "realm_" + _id_camera + "_" + _type_stage;

      // Set ros subscriber according to launch input
      _sub_input_frame = this->create_subscription<std_msgs::msg::String>(
      _topic_frame_in, 5, 
      std::bind(&MinimalPublisher::subFrame, this, std::placeholders::_1));

      if (_is_master_stage)
      {
          publisher_.insert({"general/output_dir", 
              this->create_publisher<std_msgs::msg::String>(
                  "/realm/" + _id_camera + "/general/output_dir", 5)});
          publisher_.insert({"general/gnss_base", 
              this->create_publisher<sensor_msgs::msg::NavSatFix>(
                  "/realm/" + _id_camera + "/general/gnss_base", 5)});
      }
      else
      {
          _sub_output_dir = this->create_subscription<std_msgs::msg::String>(
              "/realm/" + _id_camera + "/general/output_dir", 5, 
              std::bind(&MinimalPublisher::subOutputPath, this, std::placeholders::_1));
      }

      // Set ros services for stage handling
      _srv_req_finish = this->create_service<std_srvs::srv::Empty>(
      _topic_prefix + "request_finish",
      std::bind(&MinimalPublisher::srvFinish, this, std::placeholders::_1, std::placeholders::_2));

      _srv_req_stop = this->create_service<std_srvs::srv::Empty>(
      _topic_prefix + "request_stop",
      std::bind(&MinimalPublisher::srvStop, this, std::placeholders::_1, std::placeholders::_2));

      _srv_req_resume = this->create_service<std_srvs::srv::Empty>(
      _topic_prefix + "request_resume",
      std::bind(&MinimalPublisher::srvResume, this, std::placeholders::_1, std::placeholders::_2));

      _srv_req_reset = this->create_service<std_srvs::srv::Empty>(
      _topic_prefix + "request_reset",
      std::bind(&MinimalPublisher::srvReset, this, std::placeholders::_1, std::placeholders::_2));

      _srv_change_param = this->create_service<std_srvs::srv::Empty>(
      _topic_prefix + "change_param",
      std::bind(&MinimalPublisher::srvChangeParam, this, std::placeholders::_1, std::placeholders::_2));

      // Provide camera information a priori to all stages
      RCLCPP_INFO(this->get_logger(), "STAGE_NODE [%s]: : Loading camera from path:\n\t%s", _type_stage.c_str(),_file_settings_camera.c_str());
      _settings_camera = CameraSettingsFactory::load(_file_settings_camera);
      RCLCPP_INFO(this->get_logger(), "STAGE_NODE [%s]: : Detected camera model: '%s'", _type_stage.c_str(), std::to_string((_settings_camera)["type"]).c_str());

      // Create stages
      if (_type_stage == "pose_estimation")
        createStagePoseEstimation();
        //RCLCPP_INFO(this->get_logger(), "Calling pose estimation stage creation");
      if (_type_stage == "densification")
        //createStageDensification();
        RCLCPP_INFO(this->get_logger(), "Calling densification stage creation");
      if (_type_stage == "surface_generation")
        //createStageSurfaceGeneration();
        RCLCPP_INFO(this->get_logger(), "Calling surface generation stage creation");
      if (_type_stage == "ortho_rectification")
        //createStageOrthoRectification();
        RCLCPP_INFO(this->get_logger(), "Calling ortho rectification stage creation");
      if (_type_stage == "mosaicing")
        //createStageMosaicing();
        RCLCPP_INFO(this->get_logger(), "Calling mosaicing stage creation");
      if (_type_stage == "tileing")
        //createStageTileing();
        RCLCPP_INFO(this->get_logger(), "Calling tileing stage creation");

      // set stage path if master stage
      if (_is_master_stage)
        _stage->initStagePath(_path_output + "/" + _dir_date_time);

      // Start the thread for processing
      _stage->start();
      RCLCPP_INFO(this->get_logger(), "STAGE_NODE [%s]: Started stage node successfully!", _type_stage.c_str());

        
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:

    void readParams() /
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

        // if (!io::dirExists(_path_profile))
        //   throw(std::runtime_error("Error: Profile folder '" + _path_profile + "' was not found!"));
        // if (!io::dirExists(_path_output))
        //   io::createDir(_path_output);

        // Master priviliges
        //if (_is_master_stage)
        //{
          // Create sub directory with timestamp
          //_dir_date_time = io::getDateTime();
          //if (!io::dirExists(_path_output + "/" + _dir_date_time))
          //  io::createDir(_path_output + "/" + _dir_date_time);
        //}
    }

    void readStageSettings()
    {
      // Load stage settings
      RCLCPP_INFO(this->get_logger(), "STAGE_NODE [%s]: Loading stage settings from path:\n\t%s", _type_stage.c_str(), _file_settings_stage.c_str());
      //_settings_stage = StageSettingsFactory::load(_type_stage, _file_settings_stage);
      //RCLCPP_INFO("STAGE_NODE [%s]: Detected stage type: '%s'", _type_stage.c_str(), std::to_string((*_settings_stage)["type"]).c_str());
    }

    void createStagePoseEstimation()
    {
      // Pose estimation uses external frameworks, therefore load settings for that
      RCLCPP_INFO(this->get_logger(), "STAGE_NODE [%s]: : Loading vslam settings from path:\n\t%s", _type_stage.c_str(), _file_settings_method.c_str());
      VisualSlamSettings::Ptr settings_vslam = VisualSlamSettingsFactory::load(_file_settings_method, _path_profile + "/" + _type_stage + "/method");
      RCLCPP_INFO(this->get_logger(), "STAGE_NODE [%s]: : Detected vslam type: '%s'", _type_stage.c_str(), (settings_vslam)["type"].toString().c_str());

      ImuSettings::Ptr settings_imu = nullptr;
      if ((*_settings_stage)["use_imu"].toInt() > 0)
      {
        settings_imu = std::make_shared<ImuSettings>();
        settings_imu->loadFromFile(_file_settings_imu);
      }

      // Topic and stage creation
      __stage = std::make_shared<stages::PoseEstimation>(_settings_stage, settings_vslam, _settings_camera, settings_imu, (*_settings_camera)["fps"].toDouble());
      publisher_.insert({"output/frame", this->create_publisher<realm_msgs::msg::Frame>(_topic_frame_out, 5)});
      publisher_.insert({"output/pose/visual/utm", this->create_publisher<geometry_msgs::msg::PoseStamped>(_topic_prefix + "pose/visual/utm", 5)});
      publisher_.insert({"output/pose/visual/wgs", this->create_publisher<geometry_msgs::msg::PoseStamped>(_topic_prefix + "pose/visual/wgs", 5)});
      publisher_.insert({"output/pose/visual/traj", this->create_publisher<nav_msgs::msg::Path>(_topic_prefix + "pose/visual/traj", 5)});
      publisher_.insert({"output/pose/gnss/utm", this->create_publisher<geometry_msgs::msg::PoseStamped>(_topic_prefix + "pose/gnss/utm", 5)});
      publisher_.insert({"output/pose/gnss/wgs", this->create_publisher<geometry_msgs::msg::PoseStamped>(_topic_prefix + "pose/gnss/wgs", 5)});
      publisher_.insert({"output/pose/gnss/traj", this->create_publisher<nav_msgs::msg::Path>(_topic_prefix + "pose/gnss/traj", 5)});
      publisher_.insert({"output/pointcloud", this->create_publisher<sensor_msgs::msg::PointCloud2>(_topic_prefix + "pointcloud", 5)});
      publisher_.insert({"debug/tracked", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "tracked", 5)});
      linkStageTransport();

      if (_topic_imu_in != "uninitialised")
      {
          _sub_input_imu = this->create_subscription<sensor_msgs::msg::Imu>(_topic_imu_in, 100, std::bind(&MinimalPublisher::subImu, this, std::placeholders::_1));
      }
    }

    void linkStageTransport()
    {
      namespace ph = std::placeholders;
      auto transport_frame = std::bind(&MinimalPublisher::pubFrame, this, ph::_1, ph::_2);
      auto transport_pose = std::bind(&MinimalPublisher::pubPose, this, ph::_1, ph::_2, ph::_3, ph::_4);
      auto transport_pointcloud = std::bind(&MinimalPublisher::pubPointCloud, this, ph::_1, ph::_2);
      auto transport_img = std::bind(&MinimalPublisher::pubImage, this, ph::_1, ph::_2);
      auto transport_depth = std::bind(&MinimalPublisher::pubDepthMap, this, ph::_1, ph::_2);
      auto transport_mesh = std::bind(&MinimalPublisher::pubMesh, this, ph::_1, ph::_2);
      auto transport_cvgridmap = std::bind(&MinimalPublisher::pubCvGridMap, this, ph::_1, ph::_2, ph::_3, ph::_4);
      _stage->registerFrameTransport(transport_frame);
      _stage->registerPoseTransport(transport_pose);
      _stage->registerPointCloudTransport(transport_pointcloud);
      _stage->registerImageTransport(transport_img);
      _stage->registerDepthMapTransport(transport_img);
      _stage->registerMeshTransport(transport_mesh);
      _stage->registerCvGridMapTransport(transport_cvgridmap);
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
    
    //Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_input_frame;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_output_dir;
    
    // Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_req_finish;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_req_stop;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_req_resume;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_req_reset;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _srv_change_param;

    
    std::shared_ptr<StageSettings> _settings_stage;
    std::shared_ptr<CameraSettings> _settings_camera;
    std::shared_ptr<StageBase> _stage;
    

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

    std::string _topic_prefix;
    std::string _tf_base_frame_name;
    std::string _tf_stage_frame_name;
    std::string _path_profile;
    std::string _file_settings_camera;
    std::string _file_settings_imu;
    std::string _file_settings_stage;
    std::string _file_settings_method;
    std::string _dir_date_time;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}