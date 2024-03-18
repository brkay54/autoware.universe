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

#include "utils.hpp"

namespace reaction_analyzer
{
rclcpp::SubscriptionOptions create_subscription_options(rclcpp::Node * node)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}

// std::vector<std::tuple<std::string, std::vector<double>, double>> sort_results_by_median(
//   const std::unordered_map<std::string, std::vector<double>> test_results)
//{
//   std::vector<std::tuple<std::string, std::vector<double>, double>> sorted_data;
//   for (const auto & pair : test_results) {
//     auto vec = pair.second;
//
//     // Calculate the median
//     const size_t mid_index = vec.size() / 2;
//     std::nth_element(vec.begin(), vec.begin() + mid_index, vec.end());
//     const double median = vec[mid_index];
//
//     sorted_data.emplace_back(pair.first, pair.second, median);
//   }
//
//   // Sort based on the computed median
//   std::sort(sorted_data.begin(), sorted_data.end(), [](const auto & a, const auto & b) {
//     return std::get<2>(a) < std::get<2>(b);  // Change to > for descending order
//   });
//
//   return sorted_data;
// }

std::vector<std::string> split(const std::string & str, char delimiter)
{
  std::vector<std::string> elements;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    elements.push_back(item);
  }
  return elements;
}

size_t get_index_after_distance(
  const Trajectory & traj, const size_t curr_id, const double distance)
{
  const TrajectoryPoint & curr_p = traj.points.at(curr_id);

  size_t target_id = curr_id;
  double current_distance = 0.0;
  for (size_t traj_id = curr_id + 1; traj_id < traj.points.size(); ++traj_id) {
    current_distance = tier4_autoware_utils::calcDistance3d(traj.points.at(traj_id), curr_p);
    if (current_distance >= distance) {
      break;
    }
    target_id = traj_id;
  }
  return target_id;
}

double calculate_time_diff_ms(const rclcpp::Time & start, const rclcpp::Time & end)
{
  const auto duration = end - start;

  const auto duration_ns = duration.to_chrono<std::chrono::nanoseconds>();
  return static_cast<double>(duration_ns.count()) / 1e6;
}

TimestampedReactionPairsVector convert_pipeline_map_to_sorted_vector(
  const PipelineMap & pipelineMap)
{
  std::vector<std::tuple<rclcpp::Time, std::vector<ReactionPair>>> sortedVector;

  for (const auto & entry : pipelineMap) {
    auto sortedReactions = entry.second;
    // Sort the vector of ReactionPair based on the published stamp
    std::sort(
      sortedReactions.begin(), sortedReactions.end(),
      [](const ReactionPair & a, const ReactionPair & b) {
        return rclcpp::Time(a.second.published_stamp) < rclcpp::Time(b.second.published_stamp);
      });

    // Add to the vector as a tuple
    sortedVector.push_back(std::make_tuple(entry.first, sortedReactions));
  }

  // Sort the vector of tuples by rclcpp::Time
  std::sort(sortedVector.begin(), sortedVector.end(), [](const auto & a, const auto & b) {
    return std::get<0>(a) < std::get<0>(b);
  });

  return sortedVector;
}

unique_identifier_msgs::msg::UUID generate_uuid_msg(const std::string & input)
{
  static auto generate_uuid = boost::uuids::name_generator(boost::uuids::random_generator()());
  const auto uuid = generate_uuid(input);

  unique_identifier_msgs::msg::UUID uuid_msg;
  std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());
  return uuid_msg;
}

geometry_msgs::msg::Pose create_entity_pose(const EntityParams & entity_params)
{
  geometry_msgs::msg::Pose entity_pose;
  entity_pose.position.x = entity_params.x;
  entity_pose.position.y = entity_params.y;
  entity_pose.position.z = entity_params.z;

  tf2::Quaternion entity_q_orientation;
  entity_q_orientation.setRPY(
    tier4_autoware_utils::deg2rad(entity_params.roll),
    tier4_autoware_utils::deg2rad(entity_params.pitch),
    tier4_autoware_utils::deg2rad(entity_params.yaw));
  entity_pose.orientation = tf2::toMsg(entity_q_orientation);
  return entity_pose;
}

geometry_msgs::msg::Pose pose_params_to_pose(const PoseParams & pose_params)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose_params.x;
  pose.position.y = pose_params.y;
  pose.position.z = pose_params.z;

  tf2::Quaternion pose_q_orientation;
  pose_q_orientation.setRPY(
    tier4_autoware_utils::deg2rad(pose_params.roll),
    tier4_autoware_utils::deg2rad(pose_params.pitch),
    tier4_autoware_utils::deg2rad(pose_params.yaw));
  pose.orientation = tf2::toMsg(pose_q_orientation);
  return pose;
}

PointCloud2::SharedPtr create_entity_pointcloud_ptr(
  const EntityParams & entity_params, const double pointcloud_sampling_distance)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  tf2::Quaternion entity_q_orientation;

  entity_q_orientation.setRPY(
    tier4_autoware_utils::deg2rad(entity_params.roll),
    tier4_autoware_utils::deg2rad(entity_params.pitch),
    tier4_autoware_utils::deg2rad(entity_params.yaw));
  tf2::Transform tf(entity_q_orientation);
  const auto origin = tf2::Vector3(entity_params.x, entity_params.y, entity_params.z);
  tf.setOrigin(origin);

  const double it_x = entity_params.x_l / pointcloud_sampling_distance;
  const double it_y = entity_params.y_l / pointcloud_sampling_distance;
  const double it_z = entity_params.z_l / pointcloud_sampling_distance;

  // Sample the box and rotate
  for (int i = 0; i <= it_z; ++i) {
    for (int j = 0; j <= it_y; ++j) {
      for (int k = 0; k <= it_x; ++k) {
        const double p_x = -entity_params.x_l / 2 + k * pointcloud_sampling_distance;
        const double p_y = -entity_params.y_l / 2 + j * pointcloud_sampling_distance;
        const double p_z = -entity_params.z_l / 2 + i * pointcloud_sampling_distance;
        const auto tmp = tf2::Vector3(p_x, p_y, p_z);
        tf2::Vector3 data_out = tf * tmp;
        point_cloud.emplace_back(pcl::PointXYZ(data_out.x(), data_out.y(), data_out.z()));
      }
    }
  }
  PointCloud2::SharedPtr entity_pointcloud_ptr;
  entity_pointcloud_ptr = std::make_shared<PointCloud2>();
  pcl::toROSMsg(point_cloud, *entity_pointcloud_ptr);
  return entity_pointcloud_ptr;
}

PredictedObjects::SharedPtr create_entity_predicted_objects_ptr(const EntityParams & entity_params)
{
  unique_identifier_msgs::msg::UUID uuid_msg;

  PredictedObject obj;
  const auto entity_pose = create_entity_pose(entity_params);
  geometry_msgs::msg::Vector3 dimension;
  dimension.set__x(entity_params.x_l);
  dimension.set__y(entity_params.y_l);
  dimension.set__z(entity_params.z_l);
  obj.shape.set__dimensions(dimension);

  obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.existence_probability = 1.0;
  obj.kinematics.initial_pose_with_covariance.pose = entity_pose;

  autoware_auto_perception_msgs::msg::PredictedPath path;
  path.confidence = 1.0;
  path.path.emplace_back(entity_pose);
  obj.kinematics.predicted_paths.emplace_back(path);

  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  obj.classification.emplace_back(classification);
  obj.set__object_id(generate_uuid_msg("test_obstacle"));

  PredictedObjects pred_objects;
  pred_objects.objects.emplace_back(obj);
  return std::make_shared<PredictedObjects>(pred_objects);
}

double calculate_entity_search_radius(const EntityParams & entity_params)
{
  return std::sqrt(
           std::pow(entity_params.x_l, 2) + std::pow(entity_params.y_l, 2) +
           std::pow(entity_params.z_l, 2)) /
         2.0;
}

bool search_pointcloud_near_pose(
  const pcl::PointCloud<pcl::PointXYZ> & pcl_pointcloud, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  bool isAnyPointWithinRadius = std::any_of(
    pcl_pointcloud.points.begin(), pcl_pointcloud.points.end(),
    [pose, search_radius](const auto & point) {
      return tier4_autoware_utils::calcDistance3d(pose.position, point) <= search_radius;
    });

  if (isAnyPointWithinRadius) {
    return true;
  }
  return false;
}

bool search_predicted_objects_near_pose(
  const PredictedObjects & predicted_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  bool isAnyObjectWithinRadius = std::any_of(
    predicted_objects.objects.begin(), predicted_objects.objects.end(),
    [pose, search_radius](const PredictedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               pose.position, object.kinematics.initial_pose_with_covariance.pose.position) <=
             search_radius;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

bool search_detected_objects_near_pose(
  const DetectedObjects & detected_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  bool isAnyObjectWithinRadius = std::any_of(
    detected_objects.objects.begin(), detected_objects.objects.end(),
    [pose, search_radius](const DetectedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               pose.position, object.kinematics.pose_with_covariance.pose.position) <=
             search_radius;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

bool search_tracked_objects_near_pose(
  const TrackedObjects & tracked_objects, const geometry_msgs::msg::Pose & pose,
  const double search_radius)
{
  bool isAnyObjectWithinRadius = std::any_of(
    tracked_objects.objects.begin(), tracked_objects.objects.end(),
    [pose, search_radius](const TrackedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               pose.position, object.kinematics.pose_with_covariance.pose.position) <=
             search_radius;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

void write_results(
  rclcpp::Node * node, const std::string & output_file_path, const RunningMode & node_running_mode,
  const std::vector<PipelineMap> & pipeline_map_vector)
{
  // create csv file
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << output_file_path;
  if (!output_file_path.empty() && output_file_path.back() != '/') {
    ss << "/";  // Ensure the path ends with a slash
  }
  if (node_running_mode == RunningMode::PlanningControl) {
    ss << "planning_control-";
  } else {
    ss << "perception_planning-";
  }

  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  ss << "-reaction-results.csv";

  // open file
  std::ofstream file(ss.str());

  // Check if the file was opened successfully
  if (!file.is_open()) {
    RCLCPP_ERROR_ONCE(node->get_logger(), "Failed to open file: %s", ss.str().c_str());
    return;
  }

  size_t test_count = 0;
  std::map<std::string, std::vector<std::pair<double,double>>> test_results;

  for (const auto & pipeline_map : pipeline_map_vector) {
    test_count++;
    // convert pipeline_map to vector of tuples
    const auto sorted_results_vector = convert_pipeline_map_to_sorted_vector(pipeline_map);
    file << "Test " << test_count << "\n";

    rclcpp::Time spawn_cmd_time;  // init it to parse total latency
    for (size_t i = 0; i < sorted_results_vector.size(); ++i) {
      const auto & [pipeline_header_time, pipeline_reactions] = sorted_results_vector[i];

      if (i == 0) {
        spawn_cmd_time = pipeline_reactions[0].second.header.stamp;
      }

      // total time pipeline lasts
      file << "Pipeline - " << i + 1 << ",";

      // pipeline nodes
      for (const auto & [node_name, reaction] : pipeline_reactions) {
        file << node_name << ",";
      }

      file << "\nPipeline Latency - Total Latency [ms],";

      for (size_t j = 0; j < pipeline_reactions.size(); ++j) {
        const auto & reaction = pipeline_reactions[j].second;
        const auto & node_name = pipeline_reactions[j].first;
        if (j == 0) {
          const auto pipeline_latency =
            calculate_time_diff_ms(reaction.header.stamp, reaction.published_stamp);
          const auto total_latency =
            calculate_time_diff_ms(spawn_cmd_time, reaction.published_stamp);
          file << pipeline_latency << " - " << total_latency << ",";
          test_results
        } else {
          const auto & prev_reaction = pipeline_reactions[j - 1].second;
          const auto pipeline_latency =
            calculate_time_diff_ms(prev_reaction.published_stamp, reaction.published_stamp);
          const auto total_latency =
            calculate_time_diff_ms(spawn_cmd_time, reaction.published_stamp);
          file << pipeline_latency << " - " << total_latency << ",";
        }
      }
      file << "\n";
    }
  }
  file.close();
  RCLCPP_INFO(node->get_logger(), "Results written to: %s", ss.str().c_str());
}
}  // namespace reaction_analyzer
