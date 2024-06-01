# Polymath Radar_layer
The radar_layer package is a costmap plugin for Nav2 Costmap 2d to turn radar tracks into data that can be used in a costmap. It is expected the radar track includes the object's centroid position, velocity, position covariance, velocity covariance, and planar size of the detected object. This package allows the cost to be placed into the costmap in two different ways:

### Footprint Stamping
This is the simplest method where a rectangle determined by the x and y size of the obstacle, is placed into the costmap as a lethal obstacle, around the obstacle's centroid. This placed lethal obstacle can then be inflated with the inflation layer for planning purposes 

### 2D Gaussian Process
The cost is distributed as a 2D gaussian, where the centroid position is the highest cost and the cost surrounding the centroid decreases like a 2D normal distribution, based on the position covariance. 

Should the radar track also have velocity, the user may choose to project the gaussian distributed costmap into the future in order to plan to avoid a moving object in the future. The original Gaussian's mean is projected forward with the velocity, and the projected covariance is spread and scaled down as a function of the velocity covariance.

The algorithm used in this package is based on the following work:

- H. Guo, Z. Huang, Q. Ho, M. Ang, D. Rus [**Autonomous Navigation in Dynamic Environments with Multi-Modal Perception Uncertainties**](https://ieeexplore.ieee.org/document/9561965). IEEE International Conference on Robotics and Automation, 2021.

```bibtex
@article{guo2023autonomous,
      title={Autonomous Navigation in Dynamic Environments with Multi-Modal Perception Uncertainties}, 
      author={Hongliang Guo and Zefan Huang and Qiheng Ho and Marcelo Ang and Daniela Rus},
      year={2021},
      journal = {IEEE International Conference on Robotics and Automation}
}
```
## Topics
This costmap layer expects the radar tracks to be published in the [ObstacleArray](https://github.com/ros-navigation/navigation2_dynamic/blob/master/nav2_dynamic_msgs/msg/ObstacleArray.msg) format defined by the navigation2_dynamic package

## Configuration

| Parameter | Description | 
|-----|----|
| `enabled` | Whether it is enabled. | 
| `combination_method` | Enum for method to add data to master costmap. Must be 0, 1 or 2, default to 1 | 
| `observation_sources` | namespace of sources of data | 
| `minimum_probability` | minimum probability to place in costmap | 
| `number_of_time_steps` | number of time steps to propogate gaussian distribution of obstacle |
| `stamp_footprint` | Whether to use stamp footprint method or not | 
| `sample_time` |  sample time to propogate gaussian distribution of obstacle |
| `<data source>.topic` |  Topic of data | 

Example fully-described XML with default parameter values:

```
costmap:
  costmap:
    ros__parameters:
      footprint: "[[-1.0,-0.3],[-1.0,0.3],[3.5,0.3],[3.5,-0.3]]"
      footprint_padding: 0.0
      transform_tolerance: 2.0

      update_frequency: 20.0
      publish_frequency: 10.0

      global_frame: odom
      robot_base_frame: base_link

      width: 100
      height: 100
      origin_x: -50.0
      origin_y: -50.0
      resolution: 0.2
      rolling_window: true

      track_unknown_space: false
      unknown_cost_value: 25
      use_maximum: true

      plugins: ["radar_layer"]

      radar_layer:
        plugin: "radar_layer/RadarLayer"
        enabled: True
        number_of_time_steps: 10
        sample_time: 0.1
        minimum_probability: 0.15
        observation_sources: "radar"
        stamp_footprint: True
        radar:
          topic: /tracking

      always_send_full_costmap: True
```