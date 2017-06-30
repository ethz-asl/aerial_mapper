/*
 *    Filename: aerial-mapper-io.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-io/aerial-mapper-io.h"

// SYSTEM
#include <fstream>
#include <iostream>

// NON-SYSTEM
#include <glog/logging.h>

namespace io {

AerialMapperIO::AerialMapperIO() {}

void AerialMapperIO::loadPosesFromFile(const PoseFormat& format,
                                       const std::string& filename,
                                       Poses* T_G_Bs) {
  CHECK(T_G_Bs);
  switch (format) {
    case PoseFormat::Standard:
      loadPosesFromFileStandard(filename, T_G_Bs);
    case PoseFormat::COLMAP:
      LOG(FATAL) << "Not yet implemented!";
    case PoseFormat::PIX4D:
      LOG(FATAL) << "Not yet implemented!";
  }
}

void AerialMapperIO::loadPosesFromFileStandard(const std::string& filename,
                                               Poses* T_G_Bs) {
  CHECK(T_G_Bs);
  std::ifstream infile(filename);
  double x, y, z, qw, qx, qy, qz;
  while (infile >> x >> y >> z >> qw >> qx >> qy >> qz) {
    aslam::Quaternion q(qw, qx, qy, qz);
    Eigen::Vector3d t(x, y, z);
    aslam::Transformation T(q, t);
    T_G_Bs->push_back(T);
    if (infile.eof()) {
      break;
    }
  }
  CHECK(T_G_Bs->size() > 0) << "No poses loaded.";
  LOG(INFO) << "T_G_Bs->size() = " << T_G_Bs->size();
}

void AerialMapperIO::loadImagesFromFile(const std::string& filename_base,
                                        size_t num_poses, Images* images) {
  CHECK(images);
  for (size_t i = 0u; i < num_poses; ++i) {
    const std::string& filename = filename_base + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("Image", image);
    cv::waitKey(1);
    images->push_back(image);
  }
  CHECK(images->size() > 0) << "No images loaded.";
  LOG(INFO) << "images->size() = " << images->size();
}

void AerialMapperIO::loadCameraRigFromFile(
    const std::string& filename_ncameras_yaml,
    std::shared_ptr<aslam::NCamera> ncameras) {
  ncameras = aslam::NCamera::loadFromYaml(filename_ncameras_yaml);
  CHECK(ncameras) << "Could not load the camera calibration from: "
                  << filename_ncameras_yaml;
}

}  // namespace io
