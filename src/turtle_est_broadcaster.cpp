/**
 * Amro Al-Baali 2022
 * @file turtle_est_broadcaster.cpp
 * @brief Node that listens to the turtle's estimated pose topic and updates the turtle's estimated
 * pose in the turtlesim console
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Feb-23
 */

// Temporary: subscribe to the true pose directly until the estimated pose topic is ready

// Action items:
// - Spawn a new robot `turtle_est` if not created already
// - Subscribe to the true pose topic. The callback function must
//  - Publish odom->/est_pose transform
//  - Publish map->odom transform
//  - Request to teleport `turtle_est`

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/msg/pose.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <functional>

using std::placeholders::_1;

// Turtle spawner service
using SrvSpawn = turtlesim::srv::Spawn;
using SrvTeleportRequest = turtlesim::srv::TeleportAbsolute;

class EstimatorBroadcaster : public rclcpp::Node
{
public:
  EstimatorBroadcaster()
  : Node("turtle_est_broadcaster")
  {

    // Declare and acquire parameters
    //  Topic to subscribe to
    this->declare_parameter<std::string>("pose_subscribe_topic", "/true_turtle/pose");
    this->get_parameter("pose_subscribe_topic", pose_subscription_topic_);

    //  Teleport service topic
    this->declare_parameter<std::string>("teleport_service", "/est_turtle/teleport_absolute");
    this->get_parameter("teleport_service", teleport_service_topic_);

    //  Estimated turtle name
    this->declare_parameter<std::string>("target_name", "turtle_est_nm");
    this->get_parameter("target_name", turtle_name_);

    //  Estimated turtle frame
    this->declare_parameter<std::string>("target_frame", "turtle_est_frm");
    this->get_parameter("target_frame", target_frame_);

    //  Odometry frame
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame_);

    //  Map frame
    this->declare_parameter<std::string>("map_frame", "map");
    this->get_parameter("map_frame", map_frame_);

    // Turtle not spawned yet
    turtle_spawning_service_ready_ = false;
    est_turtle_spawned_ = false;

    // Create a client to spawn a turtle
    spawner_ = this->create_client<SrvSpawn>("spawn");

    // Create a client to teleport the turtle in turtlesim
    teleporter_ = this->create_client<SrvTeleportRequest>(teleport_service_topic_);

    // Subscribe to estimated pose topic
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            pose_subscription_topic_, 1,
            std::bind(&EstimatorBroadcaster::pose_callback, this, _1));
  }

private:
  /**
   * @brief Call back function when subscribing to the estimated pose topic
   * 
   * @param msg 
   */
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    // Spawn robot if not spawned already
    if (!est_turtle_spawned_) {
      SpawnTurtle(*msg.get(), turtle_name_);
    }

    TeleportPose(msg);
  }

  /**
   * @brief Teleport turtlesim robot
   * 
   * @param[in] msg Turtlesim pose
   */
  void TeleportPose(const turtlesim::msg::Pose::SharedPtr msg) {
    // Abort if service isn't ready
    if (!teleporter_->service_is_ready())      
      return;

    // Initialize request
    auto request = std::make_shared<SrvTeleportRequest::Request>();
    request->set__x(msg->x);
    request->set__y(msg->y);
    request->set__theta(msg->theta);

    auto result = teleporter_->async_send_request(request);
    
    std::stringstream ss;
    ss << "x: " << msg->x << ", y: " << msg->y << ", theta: " << msg->theta;
    RCLCPP_INFO(this->get_logger(), ss.str());
  }

  /**
   * @brief Spawn/start a robot at a given pose location and turtle name
   * 
   * @param[in] in_pose 
   * @param[in] turtle_name 
   * @return true Turtle successfully spawned
   */
  bool SpawnTurtle(const turtlesim::msg::Pose& in_pose, const std::string& turtle_name) {
    while (!spawner_->service_is_ready())
      RCLCPP_DEBUG(this->get_logger(), "Spawn service is not ready");
    
    if (turtle_spawning_service_ready_) {
      if (!est_turtle_spawned_) {
        est_turtle_spawned_ = true;
        RCLCPP_INFO(this->get_logger(), "Successfully spawned");        
      }
    } else {
      // Spawn only if the service is ready
      if (spawner_->service_is_ready()) {
        // Initialize request
        auto request = std::make_shared<SrvSpawn::Request>();
        request->x = in_pose.x;
        request->y = in_pose.y;
        request->theta = in_pose.theta;
        request->name = turtle_name;

        // Call request
        using ServiceResponseFuture =
            rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this, turtle_name](ServiceResponseFuture future) {
          auto result = future.get();
          if (result->name == turtle_name) {
            // Successfully spawned turtle
            std::stringstream ss;
            this->turtle_spawning_service_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "\033[92;1mSuccessfully\033[0m spawned robot");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Spawn service callback result mismatch");
          }
        };
        auto result = spawner_->async_send_request(request, response_received_callback);
      
      } else {
        RCLCPP_INFO(this->get_logger(), "Spawn service is not ready");
      }
    }

    return est_turtle_spawned_;
  }

  // Turtlesim Pose subscriber
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;

  // Transform publisher
  // It's a unique pointer so only this class has access to the pointer
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // Turtle successfully spawned
  bool est_turtle_spawned_;

  // Spawner service client
  rclcpp::Client<SrvSpawn>::SharedPtr spawner_{nullptr};

  // Turtlesim absolute teleport service
  rclcpp::Client<SrvTeleportRequest>::SharedPtr teleporter_{nullptr};

  // Pose topic to subscribe to
  std::string pose_subscription_topic_;
  
  // Teleport service request topic
  std::string teleport_service_topic_;

  // Spawned turtle name
  std::string turtle_name_;

  // Frame names
  std::string target_frame_;
  std::string odom_frame_;
  std::string map_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimatorBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

