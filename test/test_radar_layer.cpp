#define CATCH_CONFIG_MAIN // This tells Catch to provide a main() - only do
                          // this in one cpp file
#include <math.h>

#include <catch2/catch.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_dynamic_msgs/msg/obstacle.hpp"
#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "radar_layer/radar_layer.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"

class TestRadarLayerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  TestRadarLayerNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("test_radar_layer_node"), options_(options) {}

  // Lifecycle Methods
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &)
  {

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

    auto qos = rclcpp::QoS(10).deadline(rclcpp::Duration(1, 0));
    obstacle_array_publisher_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>(
      "/tracking", qos);

    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/marker", 10);

    // initialize with generic costmap parameters
    this->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    this->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    this->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    this->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    this->declare_parameter(
      "unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    this->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    this->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    this->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));

    // Configure the node here (e.g., create publishers, subscribers, etc.)
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &)
  {
    obstacle_array_publisher_->on_activate();
    marker_array_publisher_->on_activate();
    // Activate the node here (e.g., start publishing, etc.)
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    obstacle_array_publisher_->on_deactivate();
    marker_array_publisher_->on_deactivate();
    // Deactivate the node here (e.g., stop publishing, etc.)
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &)
  {
    obstacle_array_publisher_.reset();
    marker_array_publisher_.reset();
    // Cleanup the node here (e.g., release resources, etc.)
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & /* state */)
  {
    // Handle node shutdown here (e.g., additional cleanup)
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  unique_identifier_msgs::msg::UUID generateRandomUUID()
  {
    unique_identifier_msgs::msg::UUID uuid;
    std::mt19937 gen(std::random_device{}());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
    return uuid;
  }

  // Custom Methods
  void publishTf(double x, double y, double yaw, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.frame_id = "odom";
    tf_stamped.header.stamp = stamp;
    tf_stamped.child_frame_id = "base_link";
    tf_stamped.transform.translation.x = x;
    tf_stamped.transform.translation.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    tf_stamped.transform.rotation.x = q.getX();
    tf_stamped.transform.rotation.y = q.getY();
    tf_stamped.transform.rotation.z = q.getZ();
    tf_stamped.transform.rotation.w = q.getW();
    tf_broadcaster_->sendTransform(tf_stamped);
  }

  void publishObstacleArray(
    double px, double py, double pz,
    double vx, double vy, double vz,
    double sx, double sy, double sz,
    double cov_px, double cov_py, double cov_pz,
    double cov_vx, double cov_vy, double cov_vz,
    const rclcpp::Time & stamp)
  {
    nav2_dynamic_msgs::msg::ObstacleArray obstacle_array;
    obstacle_array.header.frame_id = "base_link";
    obstacle_array.header.stamp = stamp;

    nav2_dynamic_msgs::msg::Obstacle obstacle;
    obstacle.uuid = generateRandomUUID();

    obstacle.position.x = px;
    obstacle.position.y = py;
    obstacle.position.z = pz;

    obstacle.velocity.x = vx;
    obstacle.velocity.y = vy;
    obstacle.velocity.z = vz;

    obstacle.size.x = sx;
    obstacle.size.y = sy;
    obstacle.size.z = sz;

    obstacle.position_covariance[0] = cov_px;
    obstacle.position_covariance[4] = cov_py;
    obstacle.position_covariance[8] = cov_pz;

    obstacle.velocity_covariance[0] = cov_vx;
    obstacle.velocity_covariance[4] = cov_vy;
    obstacle.velocity_covariance[8] = cov_vz;

    obstacle_array.obstacles.push_back(obstacle);

    obstacle_array_publisher_->publish(obstacle_array);
  }

  void publishMarkerArray(
    double px, double py, double pz,
    double vx, double vy, double vz,
    double sx, double sy, double sz,
    double cov_px, double cov_py, double cov_pz,
    double cov_vx, double cov_vy, double cov_vz,
    const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = stamp;
    marker.type = 1;

    marker.pose.position.x = px;
    marker.pose.position.y = py;
    marker.pose.position.z = pz + sz / 2;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;

    marker.color.r = 0;
    marker.color.b = 0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = "odom";
    arrow_marker.header.stamp = stamp;
    arrow_marker.type = 0;
    arrow_marker.id = 1;

    arrow_marker.pose = marker.pose;

    arrow_marker.scale.x = vx;
    arrow_marker.scale.y = 0.5;
    arrow_marker.scale.z = 0.5;

    arrow_marker.color.r = 1.0;
    arrow_marker.color.b = 1.0;
    arrow_marker.color.g = 0;
    arrow_marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
    marker_array.markers.push_back(arrow_marker);

    marker_array_publisher_->publish(marker_array);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav2_dynamic_msgs::msg::ObstacleArray>>
  obstacle_array_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  marker_array_publisher_;
  rclcpp::NodeOptions options_;
};

struct ROS2Fixture
{
  ROS2Fixture() {rclcpp::init(0, nullptr);}
  ~ROS2Fixture() {rclcpp::shutdown();}
};

class ROSTestWrapper
{
public:
  ROSTestWrapper() {}

  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spin_thread;
  std::shared_ptr<TestRadarLayerNode> test_node;

  void Setup()
  {
    test_node = std::make_shared<TestRadarLayerNode>(rclcpp::NodeOptions());
    executor.add_node(test_node->get_node_base_interface());

    // Lifecycle Setup
    test_node->configure();
    test_node->activate();

    spin_thread = std::thread(
      [&]
      {executor.spin();});
  }

  void Teardown()
  {
    test_node->deactivate();

    test_node->cleanup();

    test_node->shutdown();

    executor.remove_node(test_node->get_node_base_interface());

    executor.cancel();

    if (spin_thread.joinable()) {
      spin_thread.join();
    }
  }
};

/* Test that a radar object creates cost*/
TEST_CASE_METHOD(ROS2Fixture, "Costmap assertions", "[radar_layer]")
{
  SECTION("Obstacle creates cost in costmap")
  {
    ROSTestWrapper test_wrapper;
    test_wrapper.Setup();

    auto node = test_wrapper.test_node;

    node->declare_parameter("radar_layer.enabled", rclcpp::ParameterValue(true));
    node->declare_parameter("radar_layer.minimum_probability", rclcpp::ParameterValue(0.1));
    node->declare_parameter("radar_layer.number_of_time_steps", rclcpp::ParameterValue(1));
    node->declare_parameter("radar_layer.observation_sources", std::string("radar"));
    node->declare_parameter("radar_layer.sample_time", rclcpp::ParameterValue(0.1));
    node->declare_parameter("radar_layer.stamp_footprint", rclcpp::ParameterValue(false));
    node->declare_parameter("radar_layer.combination_method", rclcpp::ParameterValue(1));
    node->declare_parameter("radar_layer.radar.topic", std::string("/tracking"));

    tf2_ros::Buffer tf(node->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface());
    tf.setCreateTimerInterface(timer_interface);
    tf2_ros::TransformListener tf_listener(tf);

    nav2_costmap_2d::LayeredCostmap layers("odom", false, false);
    std::shared_ptr<radar_layer::RadarLayer> rlayer = std::make_shared<radar_layer::RadarLayer>();
    rlayer->initialize(&layers, "radar_layer", &tf, node, nullptr);
    std::shared_ptr<nav2_costmap_2d::Layer> rpointer(rlayer);
    layers.addPlugin(rpointer);
    rpointer->activate();

    layers.resizeMap(1000, 1000, 0.1, 0, 0, false);

    node->publishTf(0, 0, 0, node->get_clock()->now());

    for (int i = 0; i < 1; i++) {
      node->publishObstacleArray(
        30.0, 30.0, 0.0,
        0.0, 0.0, 0.0,
        2.0, 2.0, 2.0,
        0.1, 0.1, 0.0,
        0.1, 0.1, 0.1,
        node->get_clock()->now());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    layers.updateMap(0, 0, 0);
    nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

    std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_publisher_ =
      std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(node, costmap, "odom", "costmap", true);
    costmap_publisher_->on_activate();
    unsigned int x0, y0, xn, yn;
    layers.getBounds(&x0, &xn, &y0, &yn);
    costmap_publisher_->updateBounds(x0, xn, y0, yn);

    unsigned int mx, my;
    rlayer->worldToMap(30.0, 30.0, mx, my);
    int cost = costmap->getCost(mx, my);
    costmap_publisher_->publishCostmap();
    rpointer->deactivate();
    costmap_publisher_->on_deactivate();

    REQUIRE(cost > 250);

    test_wrapper.Teardown();
  }

  SECTION("Covariance scaling factor modifies how cost is spread around mean")
  {
    ROSTestWrapper test_wrapper;
    test_wrapper.Setup();

    auto node = test_wrapper.test_node;

    node->declare_parameter("radar_layer.enabled", rclcpp::ParameterValue(true));
    node->declare_parameter("radar_layer.minimum_probability", rclcpp::ParameterValue(0.1));
    node->declare_parameter("radar_layer.number_of_time_steps", rclcpp::ParameterValue(1));
    node->declare_parameter("radar_layer.observation_sources", std::string("radar"));
    node->declare_parameter("radar_layer.sample_time", rclcpp::ParameterValue(0.1));
    node->declare_parameter("radar_layer.stamp_footprint", rclcpp::ParameterValue(false));
    node->declare_parameter("radar_layer.combination_method", rclcpp::ParameterValue(1));
    node->declare_parameter("radar_layer.covariance_scaling_factor", rclcpp::ParameterValue(1.0));
    node->declare_parameter("radar_layer.radar.topic", std::string("/tracking"));

    tf2_ros::Buffer tf(node->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface());
    tf.setCreateTimerInterface(timer_interface);
    tf2_ros::TransformListener tf_listener(tf);

    nav2_costmap_2d::LayeredCostmap layers("odom", false, false);
    std::shared_ptr<radar_layer::RadarLayer> rlayer = std::make_shared<radar_layer::RadarLayer>();
    rlayer->initialize(&layers, "radar_layer", &tf, node, nullptr);
    std::shared_ptr<nav2_costmap_2d::Layer> rpointer(rlayer);
    layers.addPlugin(rpointer);
    rpointer->activate();

    layers.resizeMap(1000, 1000, 0.1, 0, 0, false);

    node->publishTf(0, 0, 0, node->get_clock()->now());

    // arbitrarily publish an obstacle at x = 30.0 and y = 30.0 and wait a second
    for (int i = 0; i < 1; i++) {
      node->publishObstacleArray(
        30.0, 30.0, 0.0,
        0.0, 0.0, 0.0,
        2.0, 2.0, 2.0,
        0.1, 0.1, 0.0,
        0.1, 0.1, 0.1,
        node->get_clock()->now());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    layers.updateMap(0, 0, 0);
    nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

    std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_publisher_ =
      std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(node, costmap, "odom", "costmap", true);
    costmap_publisher_->on_activate();
    unsigned int x0, y0, xn, yn;
    layers.getBounds(&x0, &xn, &y0, &yn);
    costmap_publisher_->updateBounds(x0, xn, y0, yn);

    unsigned int mx, my, mx_x, my_x, mx_y, my_y;
    rlayer->worldToMap(30.0, 30.0, mx, my);      // mx, my: indicies corresponding to centroid of object in occupancy grid
    rlayer->worldToMap(31.0, 30.0, mx_x, my_x);  // mx_x, my_x: indicies corresponding to centroid of object, offset by 1m in X direction in occupancy grid
    rlayer->worldToMap(30.0, 31.0, mx_y, my_y);  // mx_y, my_y: indicies corresponding to centroid of object, offset by 1m in Y direction in occupancy grid
    int cost_0 = costmap->getCost(mx, my);       // cost at centroid
    int cost_0_x = costmap->getCost(mx_x, my_x); // cost at centroid offset by 1m in x
    int cost_0_y = costmap->getCost(mx_y, my_y); // cost at centroid offset by 1m in y
    costmap_publisher_->publishCostmap();

    node->set_parameter(rclcpp::Parameter("radar_layer.covariance_scaling_factor", 10.0));

    layers.updateMap(0, 0, 0);
    int cost_1 = costmap->getCost(mx, my);
    int cost_1_x = costmap->getCost(mx_x, my_x);
    int cost_1_y = costmap->getCost(mx_y, my_y);

    costmap_publisher_->publishCostmap();

    rpointer->deactivate();
    costmap_publisher_->on_deactivate();

    REQUIRE(cost_1 == cost_0);
    test_wrapper.Teardown();
  }
}
