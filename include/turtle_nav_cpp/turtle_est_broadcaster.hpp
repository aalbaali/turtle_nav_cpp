/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file turtle_est_broadcaster.hpp
 * @brief Node that listens to the turtle's estimated pose topic and updates the turtle's estimated
 *        pose in the turtlesim console
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-17
 */
#ifndef TURTLE_NAV_CPP_TURTLE_EST_BROADCASTER_HPP_
#define TURTLE_NAV_CPP_TURTLE_EST_BROADCASTER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace turtle_nav_cpp
{
// Turtle spawner service
using SrvSpawn = turtlesim::srv::Spawn;
using SrvTeleportRequest = turtlesim::srv::TeleportAbsolute;

/**
 * @brief Class that broadcasts TF2 transforms and communicates with turtlesim
 *
 */
class EstimatorBroadcaster : public rclcpp::Node
{
public:
  EstimatorBroadcaster();

private:
  /**
   * @brief Call back function when subscribing to the estimated pose topic
   *
   * @param msg
   */
  void PoseCallback(const turtlesim::msg::Pose::SharedPtr msg);

  /**
   * @brief Send `base_link->odom` and `odom->map` TF transforms
   *
   * @param[in] msg Estimated turtle pose with respect to the odometry frame
   */
  void SendTf2Transforms(const turtlesim::msg::Pose::SharedPtr msg) const;

  /**
   * @brief Overloads `SendTf2Transform`, where time is generated within the function
   *
   * @param[in] msg Pose to send to TF2
   * @param[in] header_frame_id Header (i.e., "from") frame ID
   * @param[in] child_frame_id Child (i.e., "to") frame ID
   */
  void SendTf2Transform(
    const turtlesim::msg::Pose::SharedPtr msg, const std::string & header_frame_id,
    const std::string & child_frame_id) const;

  /**
   * @brief Send a single transform to TF2
   *
   * @param[in] msg Pose to send to TF2
   * @param[in] header_frame_id Header (i.e., "from") frame ID
   * @param[in] child_frame_id Child (i.e., "to") frame ID
   * @param[in] time Input time to be recorded in the header
   */
  void SendTf2Transform(
    const turtlesim::msg::Pose::SharedPtr msg, const std::string & header_frame_id,
    const std::string & child_frame_id, const rclcpp::Time & time) const;

  /**
   * @brief Teleport turtlesim robot
   *
   * @param[in] msg Turtlesim pose
   */
  void TeleportPose(const turtlesim::msg::Pose::SharedPtr msg);

  /**
   * @brief Spawn/start a robot at a given pose location and turtle name
   *
   * @param[in] in_pose
   * @param[in] turtle_name
   * @return true Turtle successfully spawned
   */
  bool SpawnTurtle(const turtlesim::msg::Pose & in_pose, const std::string & turtle_name);

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
  std::string est_turtle_frame_;
  std::string true_turtle_frame_;
  std::string odom_frame_;
  std::string map_frame_;
};

}  // namespace turtle_nav_cpp

#endif  // TURTLE_NAV_CPP_TURTLE_EST_BROADCASTER_HPP_
