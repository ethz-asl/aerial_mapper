#include "fw_online_digital_elevation_map_node/ros-callback-sync.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "online_digital_elevation_map_node");
  RosCallbackSync app;
  app.registerSubscriberAndPublisher();
  app.runAndJoin();

  return 0;
}
