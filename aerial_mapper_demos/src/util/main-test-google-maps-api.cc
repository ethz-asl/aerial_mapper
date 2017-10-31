/*
 *    Filename: main-test-google-maps-api.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// NON-SYSTEM
#include <aerial-mapper-google-maps-api/google-maps-api.h>
#include <gflags/gflags.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "main_test_google_maps_api");
  ros::Time::init();

  const double zurich_long = 8.589189;
  const double zurich_lat = 47.386345;
  const int zoom = 16;
  std::shared_ptr<GoogleMapsApi> map_zurich;
  map_zurich.reset(new GoogleMapsApi(zurich_long, zurich_lat, zoom));
  cv::imshow("Google map", map_zurich->getMap());
  cv::waitKey(0);

  return 0;
}
