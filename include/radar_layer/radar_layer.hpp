#ifndef RADAR_LAYER_HPP_
#define RADAR_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "radar_msgs/msg/radar_track.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav2_dynamic_msgs/msg/obstacle.hpp"
#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include <Eigen/Dense>
#include <cmath>


namespace radar_layer
{
/**
 * @class RadarLayer
 * @brief Takes in radar data to populate into 2D costmap
 */

class RadarLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
   * @brief A constructor
   */
  RadarLayer();

  /**
   * @brief A destructor
   */
  virtual ~RadarLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Update the bounds of the master costmap by this layer's update
   *dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j);

  Eigen::VectorXd projectMean(
    nav2_dynamic_msgs::msg::Obstacle obstacle,
    double sample_time,
    int time_steps);

  Eigen::MatrixXd projectCovariance(
    nav2_dynamic_msgs::msg::Obstacle obstacle,
    double sample_time,
    int time_steps);

  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate();

  /**
   * @brief Activate the layer
   */
  virtual void activate();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable()
  {
    return true;
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief  A callback to handle buffering ObstacleArray messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void obstacleCallback(
    nav2_dynamic_msgs::msg::ObstacleArray::ConstSharedPtr message,
    const std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray> & detection_buffer,
    const std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray> & obstacle_buffer);

  void findUuid(
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr detections);

  void removeUnmatchedObstacles(
    int number_of_obstacles,
    std::vector<std::pair<size_t, size_t>> matched_indices,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles);

  void addUnmatchedDetections(
    int number_of_detections,
    std::vector<std::pair<size_t, size_t>> matched_indices,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr detections);

  std::vector<size_t> findUnmatchedIndices(
    size_t number_of_elements,
    const std::vector<std::pair<size_t, size_t>> & matched_indicies,
    bool check_first_index);

  void updateGaussian(
    nav2_dynamic_msgs::msg::Obstacle & obstacle,
    const nav2_dynamic_msgs::msg::Obstacle & detection,
    double dt);

  double getProbabilty(
    const Eigen::MatrixXd & mean,
    const Eigen::MatrixXd & inv_covariance,
    double & sqrt_2_pi_det_covariance,
    double x, double y);

  bool batchTransformPoints(
    const std::vector<geometry_msgs::msg::PointStamped> & input_points,
    std::vector<geometry_msgs::msg::PointStamped> & output_points,
    const std::string & target_frame,
    const tf2::Duration & timeout) const;

  bool batchTransform2DPoints(
    double x_x,
    double x_y,
    double y_x,
    double y_y,
    double dx,
    double dy,
    const std::vector<geometry_msgs::msg::PointStamped> & input_points,
    std::vector<geometry_msgs::msg::PointStamped> & output_points,
    const std::string & target_frame,
    const tf2::Duration & timeout) const;

  Eigen::MatrixXd getProbabilityBatch(
    const Eigen::VectorXd & mean,
    const Eigen::MatrixXd & inv_covariance,
    double sqrt_2_pi_det_covariance,
    const Eigen::MatrixXd & xs,
    const Eigen::MatrixXd & ys);

  void getObstacleProbabilty(nav2_dynamic_msgs::msg::Obstacle & obstacle);

  bool transformPoint(
    const std_msgs::msg::Header obstacle_frame,
    const nav2_dynamic_msgs::msg::Obstacle & obstacle_track,
    geometry_msgs::msg::PointStamped & out_point,
    double dx,
    double dy)
  const;

  void predictiveCost(nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array, int number_of_objects);
  void stampFootprint(nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array, int number_of_objects);

  void getTransformCoefficients(std::string source_frame, 
  std::string target_frame, double & dx, double & dy, double & x_x, double & x_y, double & y_x, double & y_y);

  void populateGrid(double x_0,
    double y_0,
    double length,
    double width,
    int obstacle_index,
    std::vector<geometry_msgs::msg::PointStamped> & points_in_obstacle_frame,
    std::vector<geometry_msgs::msg::PointStamped> & points_in_global_frame,
    Eigen::MatrixXd & xs,
    Eigen::MatrixXd & ys,
    std::vector<int> x_index,
    std::vector<int> y_index,
    nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array);

private:
  /// @brief Used to store observations from radar sensors
  std::vector<std::shared_ptr<radar_msgs::msg::RadarTracks>>
  radar_observation_buffers_;

  /// @brief Used to store observations from obstackle tracking
  std::vector<std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray>>
  obstacle_buffers_;

  /// @brief Used to store observations from obstackle tracking
  std::vector<std::shared_ptr<nav2_dynamic_msgs::msg::ObstacleArray>>
  detection_buffers_;

  /// @brief Used for the observation message filters
  std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>>
  observation_subscribers_;

  /// @brief Used to make sure that transforms are available for each sensor
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>>
  observation_notifiers_;

  /// @brief Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;

  bool rolling_window_;
  bool stamp_footprint_;
  int combination_method_;
  double min_bound;
  double max_bound;
  double min_probability_;
  int number_of_time_steps_;
  double sample_time_;
  std::string global_frame_;

  tf2::Duration transform_tolerance_;
};
} // namespace radar_layer

#endif  // RADAR_LAYER_HPP_
