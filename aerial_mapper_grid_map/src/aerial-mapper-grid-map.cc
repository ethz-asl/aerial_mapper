/*
 *    Filename: aerial-mapper-grid-map.cc
 *  Created on: Oct 9, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "aerial-mapper-grid-map/aerial-mapper-grid-map.h"

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace grid_map {

AerialGridMap::AerialGridMap(const Settings& settings)
    : settings_(settings),
      node_handle_{},
      pub_grid_map_(
          node_handle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true)) {
  initialize();
}

void AerialGridMap::initialize() {
  // Create grid map.
  map_ = grid_map::GridMap({"ortho", "elevation", "elevation_angle",
                            "num_observations", "elevation_angle_first_view",
                            "delta", "observation_index",
                            "observation_index_first"});
  map_.setFrameId("world");
  map_.setGeometry(
      grid_map::Length(settings_.delta_easting, settings_.delta_northing),
      settings_.resolution,
      grid_map::Position(settings_.center_easting, settings_.center_northing));
  ROS_INFO(
      "Created map with size %f x %f m (%i x %i cells).\n The center of the "
      "map is located at (%f, %f) in the %s frame.",
      map_.getLength().x(), map_.getLength().y(), map_.getSize()(0),
      map_.getSize()(1), map_.getPosition().x(), map_.getPosition().y(),
      map_.getFrameId().c_str());
  map_["ortho"].setConstant(255);
  map_["elevation"].setConstant(NAN);
  map_["elevation_angle"].setConstant(0.0);
  map_["elevation_angle_first_view"].setConstant(NAN);
  map_["num_observations"].setConstant(0);
  map_["observation_index"].setConstant(NAN);
  map_["observation_index_first"].setConstant(NAN);
  map_["delta"].setConstant(NAN);
}

void AerialGridMap::publishUntilShutdown() {
  ros::Rate r(0.1);  // 10 hz
  ros::NodeHandle node_handle;
  ros::Publisher pub_grid_map =
      node_handle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  while (true) {
    map_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    pub_grid_map.publish(message);
    ros::spinOnce();
    r.sleep();
  }
}

void AerialGridMap::publishOnce() {
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  pub_grid_map_.publish(message);
  ros::spinOnce();
}

}  // namespace grid_map
