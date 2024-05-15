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

  void predictiveCost(
    nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array,
    int number_of_objects);

  void stampAndProjectFootprint(
    nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array,
    int number_of_objects);

  void getTransformCoefficients(
    std::string source_frame,
    std::string target_frame,
    double & dx,
    double & dy,
    double & x_x,
    double & x_y,
    double & y_x,
    double & y_y);

  void inflateObstacle(
    int min_x, int max_x, int min_y, int max_y, double ratio,
    std::vector<int> cells_to_inflate_x, std::vector<int> cells_to_inflate_y);

  void populateGrid(
    double x_0,
    double y_0,
    double length,
    double width,
    int obstacle_index,
    std::vector<geometry_msgs::msg::PointStamped> & points_in_obstacle_frame,
    std::vector<geometry_msgs::msg::PointStamped> & points_in_global_frame,
    Eigen::MatrixXd & xs,
    Eigen::MatrixXd & ys,
    std::vector<int> & x_index,
    std::vector<int> & y_index,
    nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr obstacle_array);

  //Taken from inflation_layer with no inscribed radius
  inline unsigned char computeCost(double distance, double ratio) const
  {
    unsigned char cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else {
      // make sure cost falls off by Euclidean distance
      double factor = exp(-1.0 * cost_scaling_factor_ / ratio * (distance * resolution_));
      cost = static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  inline unsigned char costLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_costs_[dx * cache_length_ + dy];
  }

  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline double distanceLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }

  void enqueue(
    unsigned int index, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y);

  int generateIntegerDistances();

  void computeCaches(double ratio);

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
  bool inflate_obstacle_;
  int combination_method_;
  double min_bound;
  double max_bound;
  double min_probability_;
  int number_of_time_steps_;
  double sample_time_;
  double inflation_radius_;
  double cost_scaling_factor_;
  unsigned int cell_inflation_radius_, size_x_, size_y_;
  std::string global_frame_;
  std::vector<std::vector<nav2_costmap_2d::CellData>> inflation_cells_;
  std::vector<std::vector<int>> distance_matrix_;

  unsigned int cache_length_;
  unsigned int cached_cell_inflation_radius_;
  std::vector<unsigned char> cached_costs_;
  std::vector<double> cached_distances_;
  std::vector<bool> seen_;

  tf2::Duration transform_tolerance_;
};
} // namespace radar_layer

#endif  // RADAR_LAYER_HPP_
