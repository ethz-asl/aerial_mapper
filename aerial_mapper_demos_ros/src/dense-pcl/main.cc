#include "aerial-mapper-demos-ros/dense-pcl/ros-callback-sync.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

#ifdef adsf
  ros::init(argc, argv, "online_planar_rectifcation_node");
  RosCallbackSync app;
  app.registerSubscriberAndPublisher();
  app.runAndJoin();
#endif
  return 0;
}
