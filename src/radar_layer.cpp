#include "radar_layer/radar_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

namespace radar_layer
{
RadarLayer::RadarLayer()
{
  costmap_ = NULL;
}

RadarLayer::~RadarLayer()
{}

void RadarLayer::onInitialize()
{
  double transform_tolerance;

  min_bound = std::numeric_limits<double>::lowest();
  max_bound = std::numeric_limits<double>::max();

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter(
    "observation_sources",
    rclcpp::ParameterValue(std::string("")));

  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter(name_ + "." + "observation_sources", topics_string);
  node->get_parameter("transform_tolerance", transform_tolerance);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  rolling_window_ = layered_costmap_->isRolling();

  global_frame_ = layered_costmap_->getGlobalFrameID();

  default_value_ = nav2_costmap_2d::FREE_SPACE;

  RadarLayer::matchSize();

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // now we need to split the topics based on whitespace which we can use a
  // stringstream for
  std::stringstream ss(topics_string);

  std::string source;

  while (ss >> source) {
    std::string topic, sensor_frame, data_type;

    declareParameter(
      source + "." + "topic",
      rclcpp::ParameterValue(source));
    declareParameter(
      source + "." + "sensor_frame",
      rclcpp::ParameterValue(std::string("")));
    declareParameter(
      source + "." + "data_type",
      rclcpp::ParameterValue(std::string("ObstacleArray")));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(
      name_ + "." + source + "." + "sensor_frame",
      sensor_frame);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);

    if (!(data_type == "ObstacleArray")) {
      RCLCPP_FATAL(
        logger_,
        "Only topics that use obstacle array are currently supported");
      throw std::runtime_error(
              "Only topics that use obstacle array are currently supported");
    }

    RCLCPP_INFO(
      logger_,
      "Creating an observation buffer for source %s, topic %s, frame %s",
      source.c_str(),
      topic.c_str(),
      sensor_frame.c_str());

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    detection_buffers_.push_back(
      std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray>(
        new nav2_dynamic_msgs::msg::ObstacleArray()));

    obstacle_buffers_.push_back(
      std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray>(
        new nav2_dynamic_msgs::msg::ObstacleArray()));

    // base message filter subscriber, takes message type RadarTracks and
    // subscribes the current node, which is a lifecyclenode, to the topic
    auto sub = std::make_shared<message_filters::Subscriber<nav2_dynamic_msgs::msg::ObstacleArray,
        rclcpp_lifecycle::LifecycleNode>>(
      node,
      topic,
      custom_qos_profile,
      sub_opt);
    sub->unsubscribe();

    // A tf2_ros message filters which takes the topic from the subscriber
    // message filter and checks if theres a transform to targer frame within
    // the transform tolerance,
    // (https://docs.ros2.org/foxy/api/tf2_ros/classtf2__ros_1_1MessageFilter.html)
    // TODO: what if message type is not stamped with a TF frame?
    auto filter =
      std::make_shared<tf2_ros::MessageFilter<nav2_dynamic_msgs::msg::ObstacleArray>>(
      *sub,
      *tf_,
      global_frame_,
      50,
      node->get_node_logging_interface(),
      node->get_node_clock_interface(),
      transform_tolerance_);

    filter->registerCallback(
      std::bind(
        &RadarLayer::obstacleCallback, this,
        std::placeholders::_1,
        detection_buffers_.back(),
        obstacle_buffers_.back()));

    observation_subscribers_.push_back(sub);

    observation_notifiers_.push_back(filter);
    observation_notifiers_.back()->setTolerance(
      rclcpp::Duration::from_seconds(
        0.05));
  }

  RCLCPP_INFO(logger_, "RadarLayer::global_frame_ %s", global_frame_.c_str());
  dyn_params_handler_ =
    node->add_on_set_parameters_callback(
    std::bind(
      &RadarLayer::
      dynamicParametersCallback,
      this,
      std::placeholders::_1));
}

void RadarLayer::updateBounds(
  double robot_x,
  double robot_y,
  double robot_yaw,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  geometry_msgs::msg::PointStamped point_in_global_frame;
  geometry_msgs::msg::PointStamped point_in_radar_frame;
  

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  resetMaps();
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // Don't minimally bound this layer
  touch(max_bound, max_bound, min_x, min_y, max_x, max_y);
  touch(min_bound, min_bound, min_x, min_y, max_x, max_y);

  for (std::vector<std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray>>::
    const_iterator it = obstacle_buffers_.begin();
    it != obstacle_buffers_.end(); ++it)
  {
    const std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray> & obstacle_array =
      *it;
    int number_of_objects =
      obstacle_array->obstacles.size();



    geometry_msgs::msg::TransformStamped radar_to_global_transform;
    double dx;
    double dy;
    double x_x;
    double x_y;
    double y_x;
    double y_y;

    if(number_of_objects > 0){
      radar_to_global_transform = tf_->lookupTransform(obstacle_array->header.frame_id, global_frame_, clock_->now(), rclcpp::Duration::from_seconds(2.0));

      tf2::Quaternion q(
        radar_to_global_transform.transform.rotation.x,
        radar_to_global_transform.transform.rotation.y,
        radar_to_global_transform.transform.rotation.z,
        radar_to_global_transform.transform.rotation.w);
    
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      dx = radar_to_global_transform.transform.translation.x;
      dy = radar_to_global_transform.transform.translation.y;

      const double cos_roll = std::cos(roll);
      const double sin_roll = std::sin(roll);

      const double cos_pitch = std::cos(pitch);
      const double sin_pitch = std::sin(pitch);

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);

      x_x = cos_yaw*cos_pitch;
      x_y = sin_yaw*cos_pitch;

      y_x = cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll;
      y_y = sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll;
    }

    for (size_t i = 0; i < number_of_objects; i++) {
      // double length = obstacle_array->obstacles[i].size.x*10;
      // double width = obstacle_array->obstacles[i].size.y*10;
      double min_probability = 0.1;

      double length = 2 * sqrt(-2*log(min_probability) * obstacle_array->obstacles[i].position_covariance.x);
      double width = 2 * sqrt(-2*log(min_probability) * obstacle_array->obstacles[i].position_covariance.y);

      int length_in_grid = int(length / resolution_);
      int width_in_grid = int(width / resolution_);
    
      int number_of_time_steps = 3.0;
      double sample_time = 1.0;

      Eigen::MatrixXd xs(length_in_grid, width_in_grid);
      Eigen::MatrixXd ys(length_in_grid, width_in_grid);
      std::vector<geometry_msgs::msg::PointStamped> points_in_obstacle_frame(length_in_grid * width_in_grid);
      std::vector<geometry_msgs::msg::PointStamped> points_in_global_frame(length_in_grid * width_in_grid);
      std::vector<int> x_index(length_in_grid * width_in_grid);
      std::vector<int> y_index(length_in_grid * width_in_grid);

      for (int k = 0; k < number_of_time_steps; ++k) {

        Eigen::VectorXd mean = projectMean(obstacle_array->obstacles[i], sample_time, k);
        Eigen::MatrixXd covariance = projectCovariance(obstacle_array->obstacles[i], sample_time, k);
        Eigen::MatrixXd inv_covariance = Eigen::MatrixXd::Zero(2, 2);
        inv_covariance(0, 0) = 1/covariance(0, 0);
        inv_covariance(1, 1) = 1/covariance(1, 1);

        double sqrt_2_pi_det_covariance = sqrt(2 * M_PI * covariance(0, 0) * covariance(1, 1));

        rclcpp::Time start_time_ = clock_->now();
        
        ///////////////////////////////////////////////////////////////

        unsigned int point_in_obstacle_frame_index = 0;
        for (int x_i = 0; x_i < length_in_grid; x_i++) {
            for (int y_i = 0; y_i < width_in_grid; y_i++, point_in_obstacle_frame_index++) {

                double dx = - length / 2 + x_i * resolution_;
                double dy = - width / 2 + y_i * resolution_;

                xs(x_i, y_i) = mean(0) + dx;
                ys(x_i, y_i) = mean(1) + dy;
                points_in_obstacle_frame[point_in_obstacle_frame_index].header.stamp = obstacle_array->header.stamp;
                points_in_obstacle_frame[point_in_obstacle_frame_index].header.frame_id = obstacle_array->header.frame_id;
                points_in_obstacle_frame[point_in_obstacle_frame_index].point.x = obstacle_array->obstacles[i].position.x + dx;
                points_in_obstacle_frame[point_in_obstacle_frame_index].point.y = obstacle_array->obstacles[i].position.y + dy;
                points_in_obstacle_frame[point_in_obstacle_frame_index].point.z = 0;
                x_index[point_in_obstacle_frame_index] = x_i;
                y_index[point_in_obstacle_frame_index] = y_i;
          }
        }

        Eigen::MatrixXd probabilities = getProbabilityBatch(mean, 
           inv_covariance, 
           sqrt_2_pi_det_covariance, 
           xs, ys);

        bool batch_transform_success = batchTransform2DPoints(x_x,
           x_y,
           y_x,
           y_y,
           dx,
           dy,
           points_in_obstacle_frame, 
           points_in_global_frame, 
           global_frame_, 
           transform_tolerance_);
        
        // bool batch_transform_success = batchTransformPoints(points_in_obstacle_frame, 
        //    points_in_global_frame, 
        //    global_frame_, 
        //    transform_tolerance_);

        if(batch_transform_success){
          for (size_t i = 0; i < points_in_global_frame.size(); i++) {
            unsigned int mx, my;

            if (worldToMap(points_in_global_frame[i].point.x, points_in_global_frame[i].point.y, mx, my)) {
              unsigned int index = getIndex(mx, my);
              uint8_t current_cost = costmap_[index];
              costmap_[index] = std::max(current_cost, uint8_t(LETHAL_OBSTACLE * probabilities(x_index[i], y_index[i]) * sqrt_2_pi_det_covariance));
            }
          }
        }

        ////////////////////////////////////////////////////////

        // for (int x_i = 0; x_i < length_in_grid; x_i++) {
        //   for (int y_i = 0; y_i < width_in_grid; y_i++) {

        //     point_in_radar_frame.point.x = mean(0) - length / 2 + x_i * resolution_;
        //     point_in_radar_frame.point.y = mean(1) - width / 2 + y_i * resolution_;
        //     double probability = getProbabilty(mean, inv_covariance, sqrt_2_pi_det_covariance, point_in_radar_frame.point.x, point_in_radar_frame.point.y);

        //     transformPoint(
        //       obstacle_array->header,
        //       obstacle_array->obstacles[i],
        //       point_in_global_frame,
        //       -length / 2 + x_i * resolution_,
        //       -width / 2 + y_i * resolution_);

        //     double px = point_in_global_frame.point.x;
        //     double py = point_in_global_frame.point.y;

        //     // now we need to compute the map coordinates for the observation
        //     unsigned int mx, my;

        //     if (!worldToMap(px, py, mx, my)) {
        //       continue;
        //     }

        //     unsigned int index = getIndex(mx, my);
        //     uint8_t current_cost = costmap_[index];
        //     // RCLCPP_INFO(logger_, "cost: %f", current_cost);
        //     costmap_[index] = std::max(current_cost, uint8_t(LETHAL_OBSTACLE * probability * sqrt_2_pi_det_covariance));

        //   }
        // }

        /////////////////////////////////////////////////////////////

        rclcpp::Duration comp_time_ = clock_->now() - start_time_;

        RCLCPP_INFO(logger_, "Duration %f", comp_time_.seconds());
      }

    }
  }
}

void RadarLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (!enabled_) {
    return;
  }

  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default: // Nothing
      break;
  }
}

void RadarLayer::activate()
{
  for (auto & notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
}

void RadarLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

void RadarLayer::reset()
{
  resetMaps();
}

rcl_interfaces::msg::SetParametersResult RadarLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

void RadarLayer::obstacleCallback(
  nav2_dynamic_msgs::msg::ObstacleArray::ConstSharedPtr message,
  const std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray> & detection_buffer,
  const std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray> & obstacle_buffer)
{
  *detection_buffer = *message;
  findUuid(obstacle_buffer, detection_buffer);
}

void RadarLayer::findUuid(
  const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles,
  const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr detections)
{

  int number_of_obstacles = obstacles->obstacles.size();
  int number_of_detections = detections->obstacles.size();
  std::vector<std::pair<size_t, size_t>> matched_indices;

  rclcpp::Time detection_time(detections->header.stamp.sec, detections->header.stamp.nanosec);
  rclcpp::Time obstacle_time(obstacles->header.stamp.sec, obstacles->header.stamp.nanosec);
  rclcpp::Duration duration = detection_time - obstacle_time;
  double dt = duration.seconds() + duration.nanoseconds() / 1e9f;

  if (number_of_detections > 0) { //If there are detections
    if (number_of_obstacles == 0) { //Empty Obstacle List, likely first detection
      *obstacles = *detections;
    } else {
      obstacles->header.stamp = detections->header.stamp;
      //TODO: Vectorize this nested for loop for efficiency (Alex)
      for (size_t i = 0; i < number_of_detections; i++) {
        for (size_t j = 0; j < number_of_obstacles; j++) {
          if (memcmp(
              detections->obstacles[i].uuid.uuid.data(),
              obstacles->obstacles[j].uuid.uuid.data(), 16) == 0)
          {
            RCLCPP_DEBUG(logger_, "Matching UUIDs Found");
            updateGaussian(obstacles->obstacles[j], detections->obstacles[i], dt);
            matched_indices.push_back({i, j});
            break;
          }
        }
      }

      removeUnmatchedObstacles(number_of_obstacles, matched_indices, obstacles); 
      addUnmatchedDetections(number_of_detections, matched_indices, obstacles, detections); 

    }
  }
}

void RadarLayer::removeUnmatchedObstacles(int number_of_obstacles, 
  std::vector<std::pair<size_t, size_t>> matched_indices, 
  const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles){

  std::vector<size_t> unmatched_obstacles = findUnmatchedIndices(number_of_obstacles, matched_indices, false);

  if(unmatched_obstacles.size()>0){
    RCLCPP_DEBUG(logger_, "Unmatched Obstacles Found");
    std::vector<size_t> sorted_unmatched_obstacles = unmatched_obstacles;
    std::sort(sorted_unmatched_obstacles.begin(), sorted_unmatched_obstacles.end(), std::greater<size_t>());
    for (size_t i = 0; i < sorted_unmatched_obstacles.size(); i++) {
      obstacles->obstacles.erase(obstacles->obstacles.begin() + i);
    }
  }
      
}

void RadarLayer::addUnmatchedDetections(int number_of_detections, 
  std::vector<std::pair<size_t, size_t>> matched_indices, 
  const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles,
  const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr detections){
  
  std::vector<size_t> unmatched_detections = findUnmatchedIndices(number_of_detections, matched_indices, true);
      
  if(unmatched_detections.size()>0){
    RCLCPP_DEBUG(logger_, "Unmatched Detections Found");
    for (size_t i = 0; i < unmatched_detections.size(); i++) {
      obstacles->obstacles.push_back(detections->obstacles[unmatched_detections[i]]);
    }
  }
}

void RadarLayer::updateGaussian(nav2_dynamic_msgs::msg::Obstacle & obstacle,
  const nav2_dynamic_msgs::msg::Obstacle & detection,
  double dt){ 

  Eigen::Vector2d obstacle_position_mean(obstacle.position.x, obstacle.position.y);
  Eigen::Vector2d obstacle_velocity_mean(obstacle.velocity.x, obstacle.velocity.y);

  Eigen::Vector2d detection_position_mean(detection.position.x, detection.position.y);
  Eigen::Vector2d detection_velocity_mean(detection.velocity.x, detection.velocity.y);

  Eigen::MatrixXd obstacle_position_covariance = Eigen::MatrixXd::Zero(2, 2);
  obstacle_position_covariance(0, 0) = obstacle.position_covariance.x;
  obstacle_position_covariance(1, 1) = obstacle.position_covariance.y;

  Eigen::MatrixXd obstacle_velocity_covariance = Eigen::MatrixXd::Zero(2, 2);
  obstacle_velocity_covariance(0, 0) = obstacle.velocity_covariance.x;
  obstacle_velocity_covariance(1, 1) = obstacle.velocity_covariance.y;

  Eigen::MatrixXd detection_position_covariance = Eigen::MatrixXd::Zero(2, 2);
  detection_position_covariance(0, 0) = detection.position_covariance.x;
  detection_position_covariance(1, 1) = detection.position_covariance.y;

  Eigen::MatrixXd detection_velocity_covariance = Eigen::MatrixXd::Zero(2, 2);
  detection_velocity_covariance(0, 0) = detection.velocity_covariance.x;
  detection_velocity_covariance(1, 1) = detection.velocity_covariance.y;

  Eigen::MatrixXd R = (obstacle_position_covariance + dt*dt*obstacle_velocity_covariance).inverse();
  Eigen::MatrixXd new_position_covariance = (R + detection_position_covariance.inverse()).inverse();
  Eigen::VectorXd new_position_mean = new_position_covariance * (R * (obstacle_position_mean + dt*obstacle_velocity_mean) + detection_position_covariance.inverse() * detection_position_mean);

  Eigen::MatrixXd new_velocity_covariance = (obstacle_velocity_covariance.inverse() + detection_velocity_covariance.inverse()).inverse();
  Eigen::VectorXd new_velocity_mean = new_velocity_covariance * (obstacle_velocity_covariance.inverse() * obstacle_velocity_mean + detection_velocity_covariance.inverse() * detection_velocity_mean);

  obstacle.position.x = new_position_mean(0); 
  obstacle.position.y = new_position_mean(1);
  obstacle.velocity.x = new_velocity_mean(0);
  obstacle.velocity.y = new_velocity_mean(1);
  obstacle.position_covariance.x = new_position_covariance(0, 0);
  obstacle.position_covariance.y = new_position_covariance(1, 1);
  obstacle.velocity_covariance.x = new_velocity_covariance(0, 0);
  obstacle.velocity_covariance.y = new_velocity_covariance(1, 1);

}

void RadarLayer::getObstacleProbabilty(nav2_dynamic_msgs::msg::Obstacle & obstacle){

    Eigen::Vector2d mean(obstacle.position.x, obstacle.position.y);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(2, 2);
    covariance(0, 0) = obstacle.position_covariance.x;
    covariance(1, 1) = obstacle.position_covariance.y;
    Eigen::MatrixXd inv_covariance = covariance.inverse();
    double sqrt_2_pi_det_covariance = sqrt(2 * M_PI * covariance.determinant());
    
    double x, y;
    double probability;

    for (unsigned int my = 0; my < size_y_; my++) {
      for (unsigned int mx = 0; mx < size_x_; mx++) {
        mapToWorld(mx, my, x, y);
        probability = getProbabilty(mean, inv_covariance, sqrt_2_pi_det_covariance, x, y);
        unsigned int index = getIndex(x, y);
        uint8_t current_cost = costmap_[index];
        costmap_[index] = std::max(current_cost, uint8_t(LETHAL_OBSTACLE * probability * sqrt_2_pi_det_covariance));
      }
    }
}

Eigen::MatrixXd RadarLayer::getProbabilityBatch(
    const Eigen::VectorXd& mean, 
    const Eigen::MatrixXd& inv_covariance,
    double sqrt_2_pi_det_covariance,
    const Eigen::MatrixXd& xs,
    const Eigen::MatrixXd& ys) {

    // Flatten the matrices into vectors for easier handling in batch operations
    Eigen::VectorXd flat_xs = Eigen::Map<const Eigen::VectorXd>(xs.data(), xs.size());
    Eigen::VectorXd flat_ys = Eigen::Map<const Eigen::VectorXd>(ys.data(), ys.size());

    // Stack these vectors into a single 2-row matrix (inputs)
    Eigen::MatrixXd inputs(2, flat_xs.size());
    inputs.row(0) = flat_xs;
    inputs.row(1) = flat_ys;

    // Subtract the mean from each column (broadcasting)
    Eigen::MatrixXd differences = inputs.colwise() - mean;

    // Calculate the Mahalanobis distance for each point
    Eigen::MatrixXd mahalanobis = (inv_covariance * differences).cwiseProduct(differences).colwise().sum();

    // Compute probabilities using the Gaussian formula
    Eigen::MatrixXd probabilities = (1 / sqrt_2_pi_det_covariance) * (-0.5 * mahalanobis.array()).exp();

    // Reshape probabilities back into the original grid shape
    Eigen::MatrixXd grid_probabilities = Eigen::Map<Eigen::MatrixXd>(probabilities.data(), xs.rows(), xs.cols());

    return grid_probabilities;
}


double RadarLayer::getProbabilty( const Eigen::MatrixXd & mean,
  const Eigen::MatrixXd & inv_covariance,
  double & sqrt_2_pi_det_covariance,
  double x, double y){

    Eigen::Vector2d input(x, y);
    Eigen::Vector2d difference = input-mean;
    double b = difference.transpose()*inv_covariance*difference;

    double probability = (1/sqrt_2_pi_det_covariance)*exp(-b/2);

    return probability;

}

std::vector<size_t> RadarLayer::findUnmatchedIndices(
    size_t number_of_elements,
    const std::vector<std::pair<size_t, size_t>>& matched_indices,
    bool check_first_index) {

    std::set<size_t> matched_set;
    for (const auto& pair : matched_indices) {
        matched_set.insert(check_first_index ? pair.first : pair.second);
    }

    std::vector<size_t> unmatched_indices;
    for (size_t index = 0; index < number_of_elements; ++index) {
        if (matched_set.find(index) == matched_set.end()) {
            unmatched_indices.push_back(index);
        }
    }

    return unmatched_indices;
}

bool RadarLayer::batchTransform2DPoints(
    double x_x,
    double x_y,
    double y_x,
    double y_y,
    double dx,
    double dy,
    const std::vector<geometry_msgs::msg::PointStamped>& input_points,
    std::vector<geometry_msgs::msg::PointStamped>& output_points,
    const std::string& target_frame,
    const tf2::Duration& timeout) const
{
    bool all_transformed = true;

    // // Number of points
    // size_t n = input_points.size();

    // // Create matrices for input and output coordinates
    // Eigen::MatrixXd input_coords(3, n);  // Using a homogeneous coordinate system for easy matrix multiplication
    // Eigen::MatrixXd output_coords(3, n);

    // // Fill input matrix
    // for (size_t i = 0; i < n; ++i) {
    //     input_coords(0, i) = input_points[i].point.x;
    //     input_coords(1, i) = input_points[i].point.y;
    //     input_coords(2, i) = 1;  // Homogeneous coordinate
    // }

    // // Transformation matrix
    // Eigen::Matrix3d transform_matrix;
    // transform_matrix << x_x, x_y, -dx,
    //                     y_x, y_y, -dy,
    //                     0,   0,   1;

    // // Perform the matrix multiplication
    // output_coords = transform_matrix * input_coords;

    // // Assign results back to output_points
    // for (size_t i = 0; i < n; ++i) {
    //     output_points[i].point.x = output_coords(0, i);
    //     output_points[i].point.y = output_coords(1, i);
    //     output_points[i].point.z = 0;  // Setting z to zero as specified
    // }

    for (size_t i = 0; i < input_points.size(); i++) {
      output_points[i].header.stamp = input_points[i].header.stamp;
      output_points[i].header.frame_id = target_frame;
      output_points[i].point.x = input_points[i].point.x*x_x + input_points[i].point.y*x_y - dx;
      output_points[i].point.y = input_points[i].point.x*y_x + input_points[i].point.y*y_y - dy;
      output_points[i].point.z = 0;
    }

    return all_transformed;

}

bool RadarLayer::batchTransformPoints(
    const std::vector<geometry_msgs::msg::PointStamped>& input_points,
    std::vector<geometry_msgs::msg::PointStamped>& output_points,
    const std::string& target_frame,
    const tf2::Duration& timeout) const
{
    bool all_transformed = true;
    for (size_t i = 0; i < input_points.size(); i++) {
        try {
            tf_->transform(input_points[i], output_points[i], target_frame, timeout);
            output_points[i].header.stamp = input_points[i].header.stamp;
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(logger_, "Exception in batchTransformPoints: %s", ex.what());
            all_transformed = false;
        }
    }
    return all_transformed;
}

bool RadarLayer::transformPoint(
  const std_msgs::msg::Header obstacle_header,
  const nav2_dynamic_msgs::msg::Obstacle & obstacle,
  geometry_msgs::msg::PointStamped & out_point,
  double dx = 0.0,
  double dy =
  0.0) const
{
  geometry_msgs::msg::PointStamped point_in_obstacle_frame;

  point_in_obstacle_frame.header.stamp = obstacle_header.stamp;
  point_in_obstacle_frame.header.frame_id = obstacle_header.frame_id;
  point_in_obstacle_frame.point.x = obstacle.position.x + dx;
  point_in_obstacle_frame.point.y = obstacle.position.y + dy;
  point_in_obstacle_frame.point.z = 0;


  try {
    tf_->transform(
      point_in_obstacle_frame,
      out_point,
      global_frame_,
      transform_tolerance_);
    out_point.header.stamp = point_in_obstacle_frame.header.stamp;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

Eigen::VectorXd RadarLayer::projectMean(
  nav2_dynamic_msgs::msg::Obstacle obstacle,
  double sample_time,
  int time_steps
)
{
  Eigen::VectorXd position(2);
  Eigen::VectorXd velocity(2);
  Eigen::VectorXd position_projected(2);

  position(0) = obstacle.position.x;
  position(1) = obstacle.position.y;
  velocity(0) = obstacle.velocity.x;
  velocity(1) = obstacle.velocity.y;

  position_projected = position + time_steps * sample_time * velocity;

  return position_projected;
}

Eigen::MatrixXd RadarLayer::projectCovariance(
  nav2_dynamic_msgs::msg::Obstacle obstacle,
  double sample_time,
  int time_steps
)
{
  Eigen::MatrixXd position_covariance(2, 2);
  Eigen::MatrixXd velocity_covariance(2, 2);
  Eigen::MatrixXd position_covariance_projected(2, 2);

  position_covariance(0, 0) = obstacle.position_covariance.x;
  position_covariance(1, 1) = obstacle.position_covariance.y;
  velocity_covariance(0, 0) = obstacle.velocity_covariance.x;
  velocity_covariance(1, 1) = obstacle.velocity_covariance.y;

  position_covariance_projected = position_covariance +
    pow(time_steps * sample_time, 2) * velocity_covariance;

  return position_covariance_projected;
}

} // namespace radar_layer

// This is the macro allowing a radar_layer::RadarLayer class
// to be registered in order to be dynamically loadable of base type
// nav2_costmap_2d::CostmapLayer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(radar_layer::RadarLayer, nav2_costmap_2d::Layer)
