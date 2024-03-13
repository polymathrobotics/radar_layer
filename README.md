# radar_layer
Plugin for Nav2 Costmap 2d to convert radar messages into obstacles on a costmap

In this plugin

1) radarCallback will fill an observation buffer with objects
2) updateBounds will fill in the costmap with the object footprint from the observation buffer
3) updateCosts will update the shared costmap with the values depending on the update style