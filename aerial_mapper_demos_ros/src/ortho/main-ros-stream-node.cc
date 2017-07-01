#include <aerial-mapper-demos-ros/ortho/ros-callback-sync.h>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
#ifdef asdf
  ros::init(argc, argv, "online_orthomosaic_node");
  RosCallbackSync app;
  app.registerSubscriberAndPublisher();
  app.runAndJoin();
#endif
  return 0;
}
