/*
 *    Filename: densifier.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef DENSIFIER_H_
#define DENSIFIER_H_

// NON-SYSTEM
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/node_handle.h>

#include "aerial-mapper-dense-pcl/block-matching-bm.h"
#include "aerial-mapper-dense-pcl/block-matching-sgbm.h"
#include "aerial-mapper-dense-pcl/common.h"

namespace stereo {

class Densifier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Densifier(const BlockMatchingParameters& block_matching_params,
            const cv::Size& image_dimension);

  void computePointCloud(const StereoRigParameters& stereo_pair,
                         const RectifiedStereoPair& rectified_stereo_pair,
                         DensifiedStereoPair* densified_stereo_pair,
                         sensor_msgs::PointCloud2& point_cloud_ros);

  void computeDisparityMap(const RectifiedStereoPair& rectified_stereo_pair,
                           DensifiedStereoPair* densified_stereo_pair) {
    block_matcher_->computeDisparityMap(rectified_stereo_pair,
                                        densified_stereo_pair);
  }

 private:
  static constexpr int kPositionX = 0;
  static constexpr int kPositionY = 4;
  static constexpr int kPositionZ = 8;
  static constexpr int kPositionIntensity = 12;
  static constexpr float kInvalidPoint =
      std::numeric_limits<float>::quiet_NaN();
  static constexpr int kMaxInvalidDisparity = 1;
  static constexpr size_t kSizeOfFloat = sizeof(float);
  static constexpr size_t kSizeOfUint32T = sizeof(uint32_t);

  std::unique_ptr<BlockMatchingBase> block_matcher_;
  const cv::Size image_resolution_;
};

}  // namespace stereo

#endif  // DENSIFIER_H_
