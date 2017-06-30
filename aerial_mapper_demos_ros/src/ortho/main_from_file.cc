/*
 *    Filename: main_from_file.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// SYSTEM
#include <fstream>

// THIRDPARTY
#include <aslam/pipeline/visual-npipeline.h>

// PACKAGE
#include <aerial-mapper-ortho/ortho-forward-homography.h>
#include <aerial-mapper-ros/ortho/ros-callback-sync.h>


int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "online_orthomosaic_node");

  std::string base =
      "/media/timo/scnd/catkin_ws_aerial_mapper/src/aerial_mapper/data/";
  std::unique_ptr<FwOnlineOrthomosaic> online_orthomosaic_;
  std::string ncameras_yaml_path_filename = base + "camera_fixed_wing.yaml";
  online_orthomosaic_.reset(
      new FwOnlineOrthomosaic(ncameras_yaml_path_filename));

  // Load text file.
  std::vector<kindr::minimal::QuatTransformation> T_G_Bs;
  std::string file = base + "opt_poses.txt";
  std::cout << "OPEN: " << file << std::endl;
  std::ifstream infile(file);
  // int64_t ts;
  double x, y, z, qw, qx, qy, qz;
  while (infile >> x >> y >> z >> qw >> qx >> qy >> qz) {
    std::cout << qw << " " << qx << " " << qy << " " << qz << std::endl;
    aslam::Quaternion q(qw, qx, qy, qz);
    Eigen::Vector3d t(x, y, z);
    aslam::Transformation T(q, t);
    T_G_Bs.push_back(T);
    if (infile.eof()) break;
  }
  std::cout << "T_G_Bs.size() = " << T_G_Bs.size() << std::endl;

  // Load images.
  std::vector<cv::Mat> images;
  for (size_t i = 0u; i < T_G_Bs.size(); ++i) {
    std::string filename = base + "opt_image_" + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("Image", image);
    cv::waitKey(1);
    images.push_back(image);
  }

  enum Mode { Incremental, Batch };
  Mode mode;
  mode = Mode::Batch;

  if (mode == Mode::Incremental) {
    for (size_t i = 0u; i < images.size(); ++i) {
      std::cout << i << "/" << images.size() << std::endl;
      cv::Mat image = images[i];
      kindr::minimal::QuatTransformation T_G_B = T_G_Bs[i];
      online_orthomosaic_->updateOrthomosaic(T_G_B, image);
    }
  } else if (mode == Mode::Batch) {
    online_orthomosaic_->batch(T_G_Bs, images);
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}
