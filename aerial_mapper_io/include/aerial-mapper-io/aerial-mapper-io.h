/*
 *    Filename: aerial-mapper-io.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef AERIAL_MAPPER_IO_H_
#define AERIAL_MAPPER_IO_H_

// NON-SYSTEM
#include <aslam/pipeline/visual-npipeline.h>
#include <Eigen/Core>
#include <opencv2/highgui/highgui.hpp>



typedef kindr::minimal::QuatTransformation Pose;
typedef std::vector<Pose> Poses;
typedef std::vector<cv::Mat> Images;


namespace io {

enum PoseFormat { Standard, COLMAP, PIX4D };

class AerialMapperIO {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AerialMapperIO();

  void loadPosesFromFile(const PoseFormat& format,
                         const std::string& filename,
                         Poses* T_G_Bs);
  void loadPosesFromFileStandard(const std::string& filename, Poses* T_G_Bs);
  void loadImagesFromFile(const std::string& filename_base, size_t num_poses,
                          Images* images);
  void loadCameraRigFromFile();
};
}  // namespace io
#endif  // namespace AERIAL_MAPPER_IO_H_
