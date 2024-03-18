// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SUBSCRIBER_HPP_
#define SUBSCRIBER_HPP_
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
#include <utils.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace reaction_analyzer::subscriber
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_internal_msgs::msg::PublishedTime;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

// Buffers to be used to store subscribed messages' data, pipeline Header, and PublishedTime
using MessageBuffer = std::optional<PublishedTime>;
// We need to store the past AckermannControlCommands to analyze the first brake
using ControlCommandBuffer = std::pair<std::vector<AckermannControlCommand>, MessageBuffer>;
// Variant to store different types of buffers
using MessageBufferVariant = std::variant<ControlCommandBuffer, MessageBuffer>;

template <typename MessageType>
struct SubscriberVariables
{
  using ExactTimePolicy = message_filters::sync_policies::ExactTime<MessageType, PublishedTime>;

  std::unique_ptr<message_filters::Subscriber<MessageType>> sub1_;
  std::unique_ptr<message_filters::Subscriber<PublishedTime>> sub2_;
  std::unique_ptr<message_filters::Synchronizer<ExactTimePolicy>> synchronizer_;
  // tmp: only for the messages who don't have header e.g. AckermannControlCommand
  std::unique_ptr<message_filters::Cache<PublishedTime>> cache_;
};

// Variant to create subscribers for different message types
using SubscriberVariablesVariant = std::variant<
  SubscriberVariables<PointCloud2>, SubscriberVariables<DetectedObjects>,
  SubscriberVariables<TrackedObjects>, SubscriberVariables<PredictedObjects>,
  SubscriberVariables<Trajectory>, SubscriberVariables<AckermannControlCommand>>;

// The supported message types
enum class SubscriberMessageType {
  UNKNOWN = 0,
  ACKERMANN_CONTROL_COMMAND = 1,
  TRAJECTORY = 2,
  POINTCLOUD2 = 3,
  DETECTED_OBJECTS = 4,
  PREDICTED_OBJECTS = 5,
  TRACKED_OBJECTS = 6,
};

// Reaction Types
enum class ReactionType {
  UNKNOWN = 0,
  FIRST_BRAKE = 1,
  SEARCH_ZERO_VEL = 2,
  SEARCH_ENTITY = 3,
};

// The configuration of the topic to be subscribed which are defined in reaction_chain
struct TopicConfig
{
  std::string node_name;
  std::string topic_address;
  std::string time_debug_topic_address;
  SubscriberMessageType message_type;
};

// Place for the reaction functions' parameter configuration
struct FirstBrakeParams
{
  bool debug_control_commands;
  double control_cmd_buffer_time_interval;
  size_t min_number_descending_order_control_cmd;
  double min_jerk_for_brake_cmd;
};

struct SearchZeroVelParams
{
  double max_looking_distance;
};

struct SearchEntityParams
{
  double search_radius_offset;
};

// Place for the store the reaction parameter configuration (currently only for first brake)
struct ReactionParams
{
  FirstBrakeParams first_brake_params;
  SearchZeroVelParams search_zero_vel_params;
  SearchEntityParams search_entity_params;
};

using ChainModules = std::vector<TopicConfig>;

class SubscriberBase
{
public:
  explicit SubscriberBase(
    rclcpp::Node * node, Odometry::ConstSharedPtr & odometry, std::atomic<bool> & spawn_object_cmd,
    const EntityParams & entity_params);

  // Instances of SubscriberBase cannot be copied
  SubscriberBase(const SubscriberBase &) = delete;
  SubscriberBase & operator=(const SubscriberBase &) = delete;

  ~SubscriberBase() = default;

  std::optional<std::unordered_map<std::string, MessageBufferVariant>> getMessageBuffersMap();
  void reset();

private:
  std::mutex mutex_;

  // Init
  rclcpp::Node * node_;
  Odometry::ConstSharedPtr odometry_;
  std::atomic<bool> & spawn_object_cmd_;
  EntityParams entity_params_;

  // Variables to be initialized in constructor
  ChainModules chain_modules_;
  ReactionParams reaction_params_{};
  geometry_msgs::msg::Pose entity_pose_;
  double entity_search_radius_;

  // Variants
  std::unordered_map<std::string, SubscriberVariablesVariant> subscriber_variables_map_;
  std::unordered_map<std::string, MessageBufferVariant> message_buffers_;

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Functions
  void init_reaction_chains_and_params();
  bool init_subscribers();
  std::optional<size_t> find_first_brake_idx(
    const std::vector<AckermannControlCommand> & cmd_array);
  void set_control_command_to_buffer(
    std::vector<AckermannControlCommand> & buffer, const AckermannControlCommand & cmd);

  // Callbacks for modules are subscribed
  void on_control_command(
    const std::string & node_name, const AckermannControlCommand::ConstSharedPtr & msg_ptr);
  void on_trajectory(const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr);
  void on_trajectory(
    const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_pointcloud(const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr);
  void on_pointcloud(
    const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_predicted_objects(
    const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr);
  void on_predicted_objects(
    const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_detected_objects(
    const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr);
  void on_detected_objects(
    const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_tracked_objects(
    const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr);
  void on_tracked_objects(
    const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
};

}  // namespace reaction_analyzer::subscriber

#endif  // SUBSCRIBER_HPP_
