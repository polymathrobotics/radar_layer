#ifndef RADAR_LAYER_HPP_
#define RADAR_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "radar_msgs/msg/radar_track.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav2_dynamic_msgs/msg/obstacle.hpp"
#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include <Eigen/Dense>
#include <cmath>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

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

  /**
   * @brief A function to match the uuid of existing obstacles to new detections
   * @param obstacles vector of stored obstacles
   * @param detections vector of new detections
   */
  void findUuid(
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr detections);

  /**
   * @brief A function to remove obstacles that are no longer exist
   * @param number_of_obstacles number of obstacles stored
   * @param matched_indicies vector pair associated obstacles with detections
   * @param obstacles vector of stored obstacles
   */
  void removeUnmatchedObstacles(
    int number_of_obstacles,
    std::vector<std::pair<size_t, size_t>> matched_indices,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles);

  /**
   * @brief A function to add new detections
   * @param number_of_detections number of detections found
   * @param matched_indicies vector pair associated obstacles with detections
   * @param obstacles vector of stored obstacles
   * @param detections vector of new detections
   */
  void addUnmatchedDetections(
    int number_of_detections,
    std::vector<std::pair<size_t, size_t>> matched_indices,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacles,
    const nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr detections);

  /**
   * @brief A function to find unmatched obstacles or detections, returns index of unmatched obstacle or detection
   * @param number_of_elements number of obstacles or detections
   * @param matched_indicies vector pair associated obstacles with detections
   * @param check_first_index flag to check obstacle or detection. True checks for unmatched detections, False checks for unmatched obstacles
   */
  std::vector<size_t> findUnmatchedIndices(
    size_t number_of_elements,
    const std::vector<std::pair<size_t, size_t>> & matched_indicies,
    bool check_first_index);

  /**
   * @brief A function to update the position and velocity gaussian of a stored obstacl
   * @param obstacle obstacle to be updated
   * @param detection current associated detection
   * @param dt time between detections
   */
  void updateGaussian(
    nav2_dynamic_msgs::msg::Obstacle & obstacle,
    const nav2_dynamic_msgs::msg::Obstacle & detection,
    double dt);

  /**
   * @brief A function to get the probability of occupancy of a single 2D point
   * @param mean x, y position of a obstacle
   * @param inv_covariance inverse covariance of th obstacl
   * @param sqrt_2_pi_det_covariance square root of 2 pi times the determinant of the covariance
   * @param x x coordinate of interest
   * @param y y coordinate of interest
   */
  double getProbabilty(
    const Eigen::MatrixXd & mean,
    const Eigen::MatrixXd & inv_covariance,
    double & sqrt_2_pi_det_covariance,
    double x, double y);

  /**
   * @brief A function to transform a group of points into another frame
   * @param input_points a vector of input points to be transformed
   * @param output_points a vector to store the transformed points
   * @param target_frame frame to transform the points to
   * @param timeout transform timeout
   */
  bool batchTransformPoints(
    const std::vector<geometry_msgs::msg::PointStamped> & input_points,
    std::vector<geometry_msgs::msg::PointStamped> & output_points,
    const std::string & target_frame,
    const tf2::Duration & timeout) const;

  /**
   * @brief A function to transform a group of points into another frame
   * @param input_points a vector of input points to be transformed
   * @param output_points a vector to store the transformed points
   * @param target_frame frame to transform the points to
   * @param timeout transform timeout
   */
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

  /**
   * @brief A function to get the probability of occupancy of a batch of points
   * @param mean x, y position of an obstacle
   * @param inv_covariance inverse covariance of the obstacle
   * @param sqrt_2_pi_det_covariance square root of 2 pi times the determinant of the covariance
   * @param xs vector of x coordinates of interest
   * @param ys vector of y coordinates of interest
   */
  Eigen::MatrixXd getProbabilityBatch(
    const Eigen::VectorXd & mean,
    const Eigen::MatrixXd & inv_covariance,
    double sqrt_2_pi_det_covariance,
    const Eigen::MatrixXd & xs,
    const Eigen::MatrixXd & ys);

  /**
   * @brief A function to populate costmap using gaussian projection
   * @param obstacle_array array of stored obstacles
   * @param number_of_objects number of obstacles stored
   */
  void predictiveCost(
    nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array,
    int number_of_objects);

  /**
   * @brief A function to stamp the footprint of the obstacles into the costmap as lethal zones
   * @param obstacle_array array of stored obstacles
   * @param number_of_objects number of obstacles stored
   */
  void stampFootprint(
    nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array,
    int number_of_objects);

  /**
   * @brief A function to precalculate coefficients for 2D transforms
   * @param source_frame frame to transform from
   * @param target_frame frame to transform to
   * @param dx change in x in source frame
   * @param dy change in y in source frame
   * @param x_x scalar to transform x in source frame to x in target frame
   * @param x_y scalar to transform x in source frame to y in target frame
   * @param y_x scalar to transform y in source frame to x in target frame
   * @param y_y scalar to transform y in source frame to y in target frame
   */
  void getTransformCoefficients(
    std::string source_frame,
    std::string target_frame,
    double & dx,
    double & dy,
    double & x_x,
    double & x_y,
    double & y_x,
    double & y_y);

  /**
   * @brief A function to populate vectors to evaluate gaussian distributions of obstacles
   * @param x_0 x of centroid of obstacle in sensor frame
   * @param y_0 y of centroid of obstacle in sensor frame
   * @param length length of grid to be populated in [m]
   * @param width width of grid to be populated in [m]
   * @param points_in_obstacle_frame vector to store points in the obstacle frame
   * @param xs matrix to store x coordinate
   * @param ys matrix to store y coordinate
   * @param x_index vector to store x index of cell in occupancy grid
   * @param y_index vector to store y index of cell in occupancy grid
   * @param obstacle_array array of stored obstacles
   */
  void populateGrid(
    double x_0,
    double y_0,
    double length,
    double width,
    std::vector<geometry_msgs::msg::PointStamped> & points_in_obstacle_frame,
    Eigen::MatrixXd & xs,
    Eigen::MatrixXd & ys,
    std::vector<int> & x_index,
    std::vector<int> & y_index,
    nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array);

  /**
   * @brief A function to project the mean of an obstacle through time
   * @param obstacle obstacle to project through time
   * @param sample_time sample time to project mean
   * @param time_steps number of timesteps to project mean
   */
  Eigen::VectorXd projectMean(
    nav2_dynamic_msgs::msg::Obstacle obstacle,
    double sample_time,
    int time_steps);

  /**
   * @brief A function to project the covariance of an obstacle through time
   * @param obstacle obstacle to project through time
   * @param sample_time sample time to project covariance
   * @param time_steps number of timesteps to project covariance
   */
  Eigen::MatrixXd projectCovariance(
    nav2_dynamic_msgs::msg::Obstacle obstacle,
    double sample_time,
    int time_steps);

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

  /// @brief Used to bound probability to include in costmap
  double min_probability_;

  /// @brief Used for projection of mean and covariance
  int number_of_time_steps_;

  /// @brief Used for projection of mean and covariance
  double sample_time_;

  /// @brief name of global frame
  std::string global_frame_;

  /// @brief Whether to use the stamp footprint method or not
  bool stamp_footprint_;

  tf2::Duration transform_tolerance_;
  bool rolling_window_;
  int combination_method_;
  double min_bound;
  double max_bound;
};
} // namespace radar_layer

#endif  // RADAR_LAYER_HPP_
