/*
 *    Filename: aerial-mapper-grid-map.h
 *  Created on: Oct 9, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef AERIAL_MAPPER_GRID_MAP_H_
#define AERIAL_MAPPER_GRID_MAP_H_


#include <Eigen/Dense>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/publisher.h>
#include <ros/ros.h>

namespace grid_map {

struct Settings {
  double center_easting;
  double center_northing;
  double delta_easting;
  double delta_northing;
  double resolution;
};

class AerialGridMap{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AerialGridMap(const Settings& settings);

  void publishUntilShutdown();

  void publishOnce();

  grid_map::GridMap* getMutable() {
    return &map_;
  }

private:

  void initialize();

  grid_map::GridMap map_;

  Settings settings_;
  ros::NodeHandle node_handle_;
  ros::Publisher pub_grid_map_;
};


} // namespace grid_map

#endif  // AERIAL_MAPPER_GRID_MAP_H_
