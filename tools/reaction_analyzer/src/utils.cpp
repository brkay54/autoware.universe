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

std::vector<std::tuple<std::string, std::vector<double>, double>> sort_results_by_median(
  const std::unordered_map<std::string, std::vector<double>> test_results)
{
  std::vector<std::tuple<std::string, std::vector<double>, double>> sorted_data;
  for (const auto & pair : test_results) {
    auto vec = pair.second;

    // Calculate the median
    const size_t mid_index = vec.size() / 2;
    std::nth_element(vec.begin(), vec.begin() + mid_index, vec.end());
    const double median = vec[mid_index];

    sorted_data.emplace_back(pair.first, pair.second, median);
  }

  // Sort based on the computed median
  std::sort(sorted_data.begin(), sorted_data.end(), [](const auto & a, const auto & b) {
    return std::get<2>(a) < std::get<2>(b);  // Change to > for descending order
  });

  return sorted_data;
}

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
  // Get Current Trajectory Point
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
}  // namespace reaction_analyzer
