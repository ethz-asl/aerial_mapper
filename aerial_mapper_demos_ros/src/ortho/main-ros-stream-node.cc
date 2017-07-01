#include <aerial-mapper-ros/

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "online_orthomosaic_node");
  RosCallbackSync app;
  app.registerSubscriberAndPublisher();
  app.runAndJoin();

  return 0;
}
