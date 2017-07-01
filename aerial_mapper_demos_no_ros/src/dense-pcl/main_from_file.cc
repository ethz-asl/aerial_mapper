#include <aerial-mapper-dense-pcl/dense-pcl-planar-rectification.h>

#include <opencv2/core/core.hpp>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "online_planar_rectifcation_node");

  std::unique_ptr<FwOnlinePlanarRectification> online_planar_rectification_;

  Eigen::Vector3d origin(464980, 5.27226e+06, 414.087);
  std::string ncameras_yaml_path_filename =
      "/home/timo/calibration/camera_fixed_wing.yaml";
  online_planar_rectification_.reset(new FwOnlinePlanarRectification(
                                       ncameras_yaml_path_filename, origin));

  // Load text file.
  std::vector<kindr::minimal::QuatTransformation> T_G_Bs;
  std::string file = "/tmp/opt_poses.txt";
  std::cout << "OPEN: " << file << std::endl;
  std::ifstream infile(file);
  //int64_t ts;
  double x,y,z,qw,qx,qy,qz;
  while( infile >> x >> y >> z >> qw >> qx >> qy >> qz ){
    std::cout << qw << " " << qx << " " << qy << " " << qz << std::endl;
    aslam::Quaternion q(qw,qx,qy,qz);
    Eigen::Vector3d t(x,y,z);
    t = t - origin;
    aslam::Transformation T(q,t);
    T_G_Bs.push_back(T);
    if( infile.eof() ) break;
  }
  std::cout << "T_G_Bs.size() = " << T_G_Bs.size() << std::endl;

  // Load images.
  std::vector<cv::Mat> images;
  for (size_t i = 0u; i < T_G_Bs.size(); ++i) {
    std::string filename = "/tmp/opt_image_" + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    images.push_back(image);
  }

  size_t skip = 0u;
  for (size_t i = 0u; i < images.size(); ++i) {
    std::cout << i << "/" << images.size() << std::endl;
    cv::Mat image = images[i];

    if (++skip % 10== 0) {
      std::cout << "i = " << i << std::endl;
      kindr::minimal::QuatTransformation T_G_B = T_G_Bs[i];
      cv::imshow("Image", image);
      cv::waitKey(0);
      online_planar_rectification_->addFrame(T_G_B, image);
    }
  }
  std::cout << "Finished!" << std::endl;

  return 0;
}
