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

#include "reaction_analyzer_node.hpp"

#include <functional>
#include <memory>

namespace reaction_analyzer
{

void ReactionAnalyzerNode::operationModeCallback(OperationModeState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  operation_mode_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::routeStateCallback(RouteState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_route_state_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::vehiclePoseCallback(Odometry::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_ptr_ = msg_ptr;
}

void ReactionAnalyzerNode::initializationStateCallback(
  LocalizationInitializationState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  initialization_state_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::groundTruthPoseCallback(PoseStamped::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ground_truth_pose_ptr_ = std::move(msg_ptr);
}

ReactionAnalyzerNode::ReactionAnalyzerNode(rclcpp::NodeOptions options)
: Node("reaction_analyzer_node", options.automatically_declare_parameters_from_overrides(true))
{
  using std::placeholders::_1;

  node_params_.running_mode = get_parameter("running_mode").as_string();

  // set running mode
  if (node_params_.running_mode == "planning_control") {
    node_running_mode_ = RunningMode::PlanningControl;
  } else if (node_params_.running_mode == "perception_planning") {
    node_running_mode_ = RunningMode::PerceptionPlanning;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid running mode. Node couldn't be initialized. Failed.");
    return;
  }

  node_params_.timer_period = get_parameter("timer_period").as_double();
  node_params_.test_iteration = get_parameter("test_iteration").as_int();
  node_params_.output_file_path = get_parameter("output_file_path").as_string();

  node_params_.spawn_time_after_init = get_parameter("spawn_time_after_init").as_double();
  node_params_.spawn_distance_threshold = get_parameter("spawn_distance_threshold").as_double();
  node_params_.spawned_pointcloud_sampling_distance =
    get_parameter("spawned_pointcloud_sampling_distance").as_double();
  node_params_.dummy_perception_publisher_period =
    get_parameter("dummy_perception_publisher_period").as_double();

  // Position parameters
  node_params_.initial_pose.x = get_parameter("initialization_pose.x").as_double();
  node_params_.initial_pose.y = get_parameter("initialization_pose.y").as_double();
  node_params_.initial_pose.z = get_parameter("initialization_pose.z").as_double();
  node_params_.initial_pose.roll = get_parameter("initialization_pose.roll").as_double();
  node_params_.initial_pose.pitch = get_parameter("initialization_pose.pitch").as_double();
  node_params_.initial_pose.yaw = get_parameter("initialization_pose.yaw").as_double();

  node_params_.goal_pose.x = get_parameter("goal_pose.x").as_double();
  node_params_.goal_pose.y = get_parameter("goal_pose.y").as_double();
  node_params_.goal_pose.z = get_parameter("goal_pose.z").as_double();
  node_params_.goal_pose.roll = get_parameter("goal_pose.roll").as_double();
  node_params_.goal_pose.pitch = get_parameter("goal_pose.pitch").as_double();
  node_params_.goal_pose.yaw = get_parameter("goal_pose.yaw").as_double();

  node_params_.entity_params.x = get_parameter("entity_params.x").as_double();
  node_params_.entity_params.y = get_parameter("entity_params.y").as_double();
  node_params_.entity_params.z = get_parameter("entity_params.z").as_double();
  node_params_.entity_params.roll = get_parameter("entity_params.roll").as_double();
  node_params_.entity_params.pitch = get_parameter("entity_params.pitch").as_double();
  node_params_.entity_params.yaw = get_parameter("entity_params.yaw").as_double();
  node_params_.entity_params.x_l = get_parameter("entity_params.x_dimension").as_double();
  node_params_.entity_params.y_l = get_parameter("entity_params.y_dimension").as_double();
  node_params_.entity_params.z_l = get_parameter("entity_params.z_dimension").as_double();

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, std::bind(&ReactionAnalyzerNode::vehiclePoseCallback, this, _1),
    createSubscriptionOptions(this));
  sub_localization_init_state_ = create_subscription<LocalizationInitializationState>(
    "input/localization_initialization_state", rclcpp::QoS(1).transient_local(),
    std::bind(&ReactionAnalyzerNode::initializationStateCallback, this, _1),
    createSubscriptionOptions(this));
  sub_route_state_ = create_subscription<RouteState>(
    "input/routing_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::routeStateCallback, this, _1),
    createSubscriptionOptions(this));
  sub_operation_mode_ = create_subscription<OperationModeState>(
    "input/operation_mode_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::operationModeCallback, this, _1),
    createSubscriptionOptions(this));

  pub_goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("output/goal", rclcpp::QoS(1));

  initAnalyzerVariables();

  if (node_running_mode_ == RunningMode::PlanningControl) {
    pub_initial_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "output/initialpose", rclcpp::QoS(1));
    pub_pointcloud_ = create_publisher<PointCloud2>("output/pointcloud", rclcpp::SensorDataQoS());
    pub_predicted_objects_ = create_publisher<PredictedObjects>("output/objects", rclcpp::QoS(1));

    client_change_to_autonomous_ =
      create_client<ChangeOperationMode>("service/change_to_autonomous");

    // init dummy perception publisher
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(node_params_.dummy_perception_publisher_period));

    dummy_perception_timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns,
      std::bind(&ReactionAnalyzerNode::dummyPerceptionPublisher, this));

  } else if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    // Create topic publishers
    topic_publisher_ptr_ =
      std::make_shared<topic_publisher::TopicPublisher>(this, spawn_object_cmd_, spawn_cmd_time_);

    // Subscribe to the ground truth position
    sub_ground_truth_pose_ = create_subscription<PoseStamped>(
      "input/ground_truth_pose", rclcpp::QoS{1},
      std::bind(&ReactionAnalyzerNode::groundTruthPoseCallback, this, _1),
      createSubscriptionOptions(this));
  }

  // Load the subscriber to listen the topics for reactions
  odometry_ptr_ =
    std::make_shared<Odometry>();  // initialize the odometry before init the subscriber
  subscriber_ptr_ = std::make_unique<subscriber::SubscriberBase>(
    this, odometry_ptr_, entity_pose_, spawn_object_cmd_);

  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(node_params_.timer_period));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ReactionAnalyzerNode::onTimer, this));
}

void ReactionAnalyzerNode::onTimer()
{
  mutex_.lock();
  const auto current_odometry_ptr = odometry_ptr_;
  const auto initialization_state_ptr = initialization_state_ptr_;
  const auto route_state_ptr = current_route_state_ptr_;
  const auto operation_mode_ptr = operation_mode_ptr_;
  const auto ground_truth_pose_ptr = ground_truth_pose_ptr_;
  const auto spawn_cmd_time = spawn_cmd_time_;
  mutex_.unlock();

  // Init the test environment
  if (!test_environment_init_time_) {
    initEgoForTest(
      initialization_state_ptr, route_state_ptr, operation_mode_ptr, ground_truth_pose_ptr,
      current_odometry_ptr);
    return;
  }

  spawnObstacle(current_odometry_ptr->pose.pose.position);

  if (!spawn_cmd_time) return;

  const auto message_buffers = subscriber_ptr_->getMessageBuffersMap();

  if (message_buffers) {
    // if reacted, calculate the results
    calculateResults(message_buffers.value(), spawn_cmd_time.value());
    reset();
  }
}

void ReactionAnalyzerNode::dummyPerceptionPublisher()
{
  if (!spawn_object_cmd_) {
    // do not spawn it, send empty pointcloud
    pcl::PointCloud<pcl::PointXYZ> pcl_empty;
    PointCloud2 empty_pointcloud;
    PredictedObjects empty_predicted_objects;
    pcl::toROSMsg(pcl_empty, empty_pointcloud);

    const auto current_time = this->now();
    empty_pointcloud.header.frame_id = "base_link";
    empty_pointcloud.header.stamp = current_time;

    empty_predicted_objects.header.frame_id = "map";
    empty_predicted_objects.header.stamp = current_time;

    pub_pointcloud_->publish(empty_pointcloud);
    pub_predicted_objects_->publish(empty_predicted_objects);
  } else {
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped{};
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link", "map", this->now(), rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to look up transform from map to base_link");
      return;
    }

    // transform by using eigen matrix
    PointCloud2 transformed_points{};
    const Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(affine_matrix, *entity_pointcloud_ptr_, transformed_points);
    const auto current_time = this->now();

    transformed_points.header.frame_id = "base_link";
    transformed_points.header.stamp = current_time;

    predicted_objects_ptr_->header.frame_id = "map";
    predicted_objects_ptr_->header.stamp = current_time;

    pub_pointcloud_->publish(transformed_points);
    pub_predicted_objects_->publish(*predicted_objects_ptr_);
    if (!is_object_spawned_message_published_) {
      mutex_.lock();
      spawn_cmd_time_ = this->now();
      mutex_.unlock();
      is_object_spawned_message_published_ = true;
    }
  }
}

void ReactionAnalyzerNode::spawnObstacle(const geometry_msgs::msg::Point & ego_pose)
{
  if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    rclcpp::Duration time_diff = this->now() - test_environment_init_time_.value();
    if (time_diff > rclcpp::Duration::from_seconds(node_params_.spawn_time_after_init)) {
      if (!spawn_object_cmd_) {
        spawn_object_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "Spawn command is sent.");
      }
    }
  } else {
    if (
      tier4_autoware_utils::calcDistance3d(ego_pose, entity_pose_.position) <
      node_params_.spawn_distance_threshold) {
      if (!spawn_object_cmd_) {
        spawn_object_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "Spawn command is sent.");
      }
    }
  }
}

void ReactionAnalyzerNode::calculateResults(
  const std::unordered_map<std::string, subscriber::MessageBufferVariant> & message_buffers,
  const rclcpp::Time & spawn_cmd_time)
{
  // Map the reaction times w.r.t its header time to categorize it
  PipelineMap pipeline_map;

  {
    // set the spawn_cmd_time as the first reaction pair
    ReactionPair reaction_pair;
    reaction_pair.first = "spawn_cmd_time";
    reaction_pair.second.header.stamp = spawn_cmd_time;
    reaction_pair.second.published_stamp = spawn_cmd_time;
    pipeline_map[reaction_pair.second.header.stamp].emplace_back(reaction_pair);
  }

  for (const auto & [key, variant] : message_buffers) {
    ReactionPair reaction_pair;
    if (auto * control_message = std::get_if<subscriber::ControlCommandBuffer>(&variant)) {
      if (control_message->second) {
        reaction_pair.first = key;
        reaction_pair.second = control_message->second.value();
      }
    } else if (auto * general_message = std::get_if<subscriber::MessageBuffer>(&variant)) {
      if (general_message->has_value()) {
        reaction_pair.first = key;
        reaction_pair.second = general_message->value();
      }
    }
    pipeline_map[reaction_pair.second.header.stamp].emplace_back(reaction_pair);
  }
  pipeline_map_vector_.emplace_back(pipeline_map);
  test_iteration_count_++;
}

void ReactionAnalyzerNode::initAnalyzerVariables()
{
  tf2::Quaternion entity_q_orientation;
  entity_q_orientation.setRPY(
    tier4_autoware_utils::deg2rad(node_params_.entity_params.roll),
    tier4_autoware_utils::deg2rad(node_params_.entity_params.pitch),
    tier4_autoware_utils::deg2rad(node_params_.entity_params.yaw));
  entity_pose_.position.set__x(node_params_.entity_params.x);
  entity_pose_.position.set__y(node_params_.entity_params.y);
  entity_pose_.position.set__z(node_params_.entity_params.z);
  entity_pose_.orientation.set__x(entity_q_orientation.x());
  entity_pose_.orientation.set__y(entity_q_orientation.y());
  entity_pose_.orientation.set__z(entity_q_orientation.z());
  entity_pose_.orientation.set__w(entity_q_orientation.w());

  tf2::Quaternion goal_pose_q_orientation;
  goal_pose_q_orientation.setRPY(
    tier4_autoware_utils::deg2rad(node_params_.goal_pose.roll),
    tier4_autoware_utils::deg2rad(node_params_.goal_pose.pitch),
    tier4_autoware_utils::deg2rad(node_params_.goal_pose.yaw));

  goal_pose_.pose.position.set__x(node_params_.goal_pose.x);
  goal_pose_.pose.position.set__y(node_params_.goal_pose.y);
  goal_pose_.pose.position.set__z(node_params_.goal_pose.z);
  goal_pose_.pose.orientation.set__x(goal_pose_q_orientation.x());
  goal_pose_.pose.orientation.set__y(goal_pose_q_orientation.y());
  goal_pose_.pose.orientation.set__z(goal_pose_q_orientation.z());
  goal_pose_.pose.orientation.set__w(goal_pose_q_orientation.w());

  if (node_running_mode_ == RunningMode::PlanningControl) {
    tf2::Quaternion initial_pose_q_orientation;
    initial_pose_q_orientation.setRPY(
      tier4_autoware_utils::deg2rad(node_params_.initial_pose.roll),
      tier4_autoware_utils::deg2rad(node_params_.initial_pose.pitch),
      tier4_autoware_utils::deg2rad(node_params_.initial_pose.yaw));

    init_pose_.pose.pose.position.set__x(node_params_.initial_pose.x);
    init_pose_.pose.pose.position.set__y(node_params_.initial_pose.y);
    init_pose_.pose.pose.position.set__z(node_params_.initial_pose.z);
    init_pose_.pose.pose.orientation.set__x(initial_pose_q_orientation.x());
    init_pose_.pose.pose.orientation.set__y(initial_pose_q_orientation.y());
    init_pose_.pose.pose.orientation.set__z(initial_pose_q_orientation.z());
    init_pose_.pose.pose.orientation.set__w(initial_pose_q_orientation.w());

    initPointcloud();
    initPredictedObjects();
  }
}

void ReactionAnalyzerNode::initPointcloud()
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  // prepare transform matrix
  tf2::Quaternion entity_q_orientation;
  entity_q_orientation.setX(entity_pose_.orientation.x);
  entity_q_orientation.setY(entity_pose_.orientation.y);
  entity_q_orientation.setZ(entity_pose_.orientation.z);
  entity_q_orientation.setW(entity_pose_.orientation.w);

  tf2::Transform tf(entity_q_orientation);
  const auto origin =
    tf2::Vector3(entity_pose_.position.x, entity_pose_.position.y, entity_pose_.position.z);
  tf.setOrigin(origin);

  const double it_x =
    node_params_.entity_params.x_l / node_params_.spawned_pointcloud_sampling_distance;
  const double it_y =
    node_params_.entity_params.y_l / node_params_.spawned_pointcloud_sampling_distance;
  const double it_z =
    node_params_.entity_params.z_l / node_params_.spawned_pointcloud_sampling_distance;

  // Sample the box and rotate
  for (int i = 0; i <= it_z; ++i) {
    for (int j = 0; j <= it_y; ++j) {
      for (int k = 0; k <= it_x; ++k) {
        const double p_x = -node_params_.entity_params.x_l / 2 +
                           k * node_params_.spawned_pointcloud_sampling_distance;
        const double p_y = -node_params_.entity_params.y_l / 2 +
                           j * node_params_.spawned_pointcloud_sampling_distance;
        const double p_z = -node_params_.entity_params.z_l / 2 +
                           i * node_params_.spawned_pointcloud_sampling_distance;
        const auto tmp = tf2::Vector3(p_x, p_y, p_z);
        tf2::Vector3 data_out = tf * tmp;
        point_cloud.emplace_back(pcl::PointXYZ(data_out.x(), data_out.y(), data_out.z()));
      }
    }
  }
  entity_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::toROSMsg(point_cloud, *entity_pointcloud_ptr_);
}

void ReactionAnalyzerNode::initPredictedObjects()
{
  auto generateUUIDMsg = [](const std::string & input) {
    static auto generate_uuid = boost::uuids::name_generator(boost::uuids::random_generator()());
    const auto uuid = generate_uuid(input);

    unique_identifier_msgs::msg::UUID uuid_msg;
    std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());
    return uuid_msg;
  };

  PredictedObject obj;

  geometry_msgs::msg::Vector3 dimension;
  dimension.set__x(node_params_.entity_params.x_l);
  dimension.set__y(node_params_.entity_params.y_l);
  dimension.set__z(node_params_.entity_params.z_l);
  obj.shape.set__dimensions(dimension);

  obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.existence_probability = 1.0;
  obj.kinematics.initial_pose_with_covariance.pose = entity_pose_;

  autoware_auto_perception_msgs::msg::PredictedPath path;
  path.confidence = 1.0;
  path.path.emplace_back(entity_pose_);
  obj.kinematics.predicted_paths.emplace_back(path);

  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  obj.classification.emplace_back(classification);
  obj.set__object_id(generateUUIDMsg("test_obstacle"));

  PredictedObjects pred_objects;
  pred_objects.objects.emplace_back(obj);
  predicted_objects_ptr_ = std::make_shared<PredictedObjects>(pred_objects);
}

void ReactionAnalyzerNode::initEgoForTest(
  const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
  const RouteState::ConstSharedPtr & route_state_ptr,
  const OperationModeState::ConstSharedPtr & operation_mode_ptr,
  const PoseStamped::ConstSharedPtr & ground_truth_pose_ptr,
  const Odometry::ConstSharedPtr & odometry_ptr)
{
  const auto current_time = this->now();

  // Initialize the test environment
  constexpr double initialize_call_period = 1.0;  // sec

  if (
    !last_test_environment_init_request_time_ ||
    (current_time - last_test_environment_init_request_time_.value()).seconds() >=
      initialize_call_period) {
    last_test_environment_init_request_time_ = current_time;

    // Pose initialization of the ego
    if (
      initialization_state_ptr &&
      (initialization_state_ptr->state != LocalizationInitializationState::INITIALIZED ||
       !is_vehicle_initialized_)) {
      if (initialization_state_ptr->state == LocalizationInitializationState::INITIALIZED) {
        is_vehicle_initialized_ = true;
      }
      if (node_running_mode_ == RunningMode::PlanningControl) {
        // publish initial pose
        init_pose_.header.stamp = current_time;
        init_pose_.header.frame_id = "map";
        pub_initial_pose_->publish(init_pose_);
      }
      return;
    }

    // Wait until odometry_ptr is initialized
    if (!odometry_ptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Odometry is not received. Waiting for odometry...");
      return;
    }

    // Check is position initialized accurately, if node is running PerceptionPlanning mode
    if (node_running_mode_ == RunningMode::PerceptionPlanning) {
      if (!ground_truth_pose_ptr) {
        RCLCPP_WARN(
          get_logger(), "Ground truth pose is not received. Waiting for Ground truth pose...");
        return;
      } else {
        constexpr double deviation_threshold = 0.1;
        const auto deviation = tier4_autoware_utils::calcPoseDeviation(
          ground_truth_pose_ptr->pose, odometry_ptr->pose.pose);
        if (
          deviation.longitudinal > deviation_threshold || deviation.lateral > deviation_threshold ||
          deviation.yaw > deviation_threshold) {
          RCLCPP_ERROR(
            get_logger(),
            "Deviation between ground truth position and ego position is high. Node is shutting "
            "down. Longitudinal deviation: %f, Lateral deviation: %f, Yaw deviation: %f",
            deviation.longitudinal, deviation.lateral, deviation.yaw);
          rclcpp::shutdown();
        }
      }
    }

    if (route_state_ptr && (route_state_ptr->state != RouteState::SET || !is_route_set_)) {
      if (route_state_ptr->state == RouteState::SET) {
        is_route_set_ = true;
      }
      // publish goal pose
      goal_pose_.header.stamp = current_time;
      goal_pose_.header.frame_id = "map";
      pub_goal_pose_->publish(goal_pose_);
      return;
    }

    // if node is running PlanningControl mode, change ego to Autonomous mode.
    if (node_running_mode_ == RunningMode::PlanningControl) {
      // change to autonomous
      if (operation_mode_ptr && operation_mode_ptr->mode != OperationModeState::AUTONOMOUS) {
        callOperationModeServiceWithoutResponse();
        return;
      }
    }
    const bool is_ready =
      (is_vehicle_initialized_ && is_route_set_ &&
       (operation_mode_ptr->mode == OperationModeState::AUTONOMOUS ||
        node_running_mode_ == RunningMode::PerceptionPlanning));
    if (is_ready) {
      test_environment_init_time_ = this->now();
    }
  }
}

void ReactionAnalyzerNode::callOperationModeServiceWithoutResponse()
{
  auto req = std::make_shared<ChangeOperationMode::Request>();

  RCLCPP_INFO(this->get_logger(), "client request");

  if (!client_change_to_autonomous_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "client is unavailable");
    return;
  }

  client_change_to_autonomous_->async_send_request(
    req, [this](typename rclcpp::Client<ChangeOperationMode>::SharedFuture result) {
      RCLCPP_INFO(
        this->get_logger(), "Status: %d, %s", result.get()->status.code,
        result.get()->status.message.c_str());
    });
}

void ReactionAnalyzerNode::reset()
{
  if (test_iteration_count_ >= node_params_.test_iteration) {
    writeResultsToFile();
    RCLCPP_INFO(get_logger(), "%zu Tests are finished. Node shutting down.", test_iteration_count_);
    rclcpp::shutdown();
    return;
  }
  is_vehicle_initialized_ = false;
  is_route_set_ = false;
  test_environment_init_time_ = std::nullopt;
  last_test_environment_init_request_time_ = std::nullopt;
  spawn_object_cmd_ = false;
  is_object_spawned_message_published_ = false;
  if (topic_publisher_ptr_) {
    topic_publisher_ptr_->reset();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  spawn_cmd_time_ = std::nullopt;
  subscriber_ptr_->reset();
  RCLCPP_INFO(this->get_logger(), "Test - %zu is done, resetting..", test_iteration_count_);
}

void ReactionAnalyzerNode::writeResultsToFile()
{
  // create csv file
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << node_params_.output_file_path;
  if (!node_params_.output_file_path.empty() && node_params_.output_file_path.back() != '/') {
    ss << "/";  // Ensure the path ends with a slash
  }
  if (node_running_mode_ == RunningMode::PlanningControl) {
    ss << "planning_control-";
  } else {
    ss << "perception_planning-";
  }

  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  ss << "-reaction-results.csv";

  // parse the results
  std::ofstream file(ss.str());
  // Check if the file was opened successfully
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << ss.str() << std::endl;
    return;
  }

  size_t test_count = 0;
  for (const auto & pipeline_map : pipeline_map_vector_) {
    test_count++;
    // convert pipeline_map to vector of tuples
    const auto sorted_results_vector = convertPipelineMap(pipeline_map);
    file << "Test " << test_count << "\n";
    for (size_t i = 1; i < sorted_results_vector.size(); ++i) {
      const auto & [pipeline_header_time, reactions] = sorted_results_vector[i];

      for (const auto & reaction : reactions) {
        if (i == 1) {

        }
      }
      file << "Min,Max,Mean,Median,StdDev\n";
      break;
    }

//    // parse test results
//    file << node << ",";
//    for (double duration : durations) {
//      file << duration << ",";
//    }
//
//    // calculate stats
//    const double min = *std::min_element(durations.begin(), durations.end());
//    const double max = *std::max_element(durations.begin(), durations.end());
//
//    std::vector<double> sorted_data = durations;
//    std::sort(sorted_data.begin(), sorted_data.end());
//    const double median =
//      sorted_data.size() % 2 == 0
//        ? (sorted_data[sorted_data.size() / 2 - 1] + sorted_data[sorted_data.size() / 2]) / 2
//        : sorted_data[sorted_data.size() / 2];
//
//    const double mean =
//      std::accumulate(sorted_data.begin(), sorted_data.end(), 0.0) / sorted_data.size();
//    const double sq_sum = std::inner_product(
//      sorted_data.begin(), sorted_data.end(), sorted_data.begin(), 0.0, std::plus<>(),
//      [&](double a, double b) { return (a - mean) * (b - mean); });
//    double std_dev = std::sqrt(sq_sum / sorted_data.size());
//
//    // parse stats
//    file << min << "," << max << "," << mean << "," << median << "," << std_dev << "\n";
  }
  file.close();
  RCLCPP_INFO(this->get_logger(), "Results written to: %s", ss.str().c_str());
}

}  // namespace reaction_analyzer

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>

RCLCPP_COMPONENTS_REGISTER_NODE(reaction_analyzer::ReactionAnalyzerNode)
