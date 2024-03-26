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
      rclcpp::ParameterValue(std::string("RadarTrack")));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(
      name_ + "." + source + "." + "sensor_frame",
      sensor_frame);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);

    if (!(data_type == "RadarTracks")) {
      RCLCPP_FATAL(
        logger_,
        "Only topics that use radar tracks are currently supported");
      throw std::runtime_error(
              "Only topics that use radar tracks are currently supported");
    }

    RCLCPP_INFO(
      logger_,
      "Creating an observation buffer for source %s, topic %s, frame %s",
      source.c_str(),
      topic.c_str(),
      sensor_frame.c_str());

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    radar_observation_buffers_.push_back(
      std::shared_ptr<radar_msgs::msg::RadarTracks>(
        new radar_msgs::msg::RadarTracks()));

    // base message filter subscriber, takes message type RadarTracks and
    // subscribes the current node, which is a lifecyclenode, to the topic
    auto sub = std::make_shared<message_filters::Subscriber<radar_msgs::msg::RadarTracks,
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
      std::make_shared<tf2_ros::MessageFilter<radar_msgs::msg::RadarTracks>>(
      *sub,
      *tf_,
      global_frame_,
      50,
      node->get_node_logging_interface(),
      node->get_node_clock_interface(),
      transform_tolerance_);

    filter->registerCallback(
      std::bind(
        &RadarLayer::radarCallback, this,
        std::placeholders::_1,
        radar_observation_buffers_.back()));

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

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  resetMaps();
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // Don't minimally bound this layer
  touch(max_bound, max_bound, min_x, min_y, max_x, max_y);
  touch(min_bound, min_bound, min_x, min_y, max_x, max_y);

  for (std::vector<std::shared_ptr<radar_msgs::msg::RadarTracks>>::
    const_iterator it = radar_observation_buffers_.begin();
    it != radar_observation_buffers_.end(); ++it)
  {
    const std::shared_ptr<radar_msgs::msg::RadarTracks> & radar_observation =
      *it;
    int number_of_objects =
      radar_observation->tracks.size();

    for (size_t i = 0; i < number_of_objects; i++) {
      double length = radar_observation->tracks[i].size.x;
      double width = radar_observation->tracks[i].size.y;

      int length_in_grid = int(length / resolution_);
      int width_in_grid = int(width / resolution_);


      for (int x_i = 0; x_i < length_in_grid; x_i++) {
        for (int y_i = 0; y_i < width_in_grid; y_i++) {
          transformPoint(
            radar_observation->header,
            radar_observation->tracks[i],
            point_in_global_frame,
            -length / 2 + x_i * resolution_,
            -width / 2 + y_i * resolution_);

          double px = point_in_global_frame.point.x;
          double py = point_in_global_frame.point.y;

          // now we need to compute the map coordinates for the observation
          unsigned int mx, my;

          if (!worldToMap(px, py, mx, my)) {
            continue;
          }
          unsigned int index = getIndex(mx, my);
          costmap_[index] = LETHAL_OBSTACLE;
        }
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

void RadarLayer::radarCallback(
  radar_msgs::msg::RadarTracks::ConstSharedPtr message,
  const std::shared_ptr<radar_msgs::msg::RadarTracks> & buffer)
{
  *buffer = *message;
}

bool RadarLayer::transformPoint(
  const std_msgs::msg::Header radar_header,
  const radar_msgs::msg::RadarTrack & radar_track,
  geometry_msgs::msg::PointStamped & out_point,
  double dx = 0.0,
  double dy =
  0.0) const
{
  geometry_msgs::msg::PointStamped point_in_radar_frame;

  point_in_radar_frame.header.stamp = radar_header.stamp;
  point_in_radar_frame.header.frame_id = radar_header.frame_id;
  point_in_radar_frame.point.x = radar_track.position.x + dx;
  point_in_radar_frame.point.y = radar_track.position.y + dy;
  point_in_radar_frame.point.z = 0;


  try {
    tf_->transform(
      point_in_radar_frame,
      out_point,
      global_frame_,
      transform_tolerance_);
    out_point.header.stamp = point_in_radar_frame.header.stamp;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}
} // namespace radar_layer

// This is the macro allowing a radar_layer::RadarLayer class
// to be registered in order to be dynamically loadable of base type
// nav2_costmap_2d::CostmapLayer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(radar_layer::RadarLayer, nav2_costmap_2d::Layer)
