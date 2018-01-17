/*
 *    Filename: block-matching-sgbm.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef BLOCK_MATCHING_SGBM_H_
#define BLOCK_MATCHING_SGBM_H_

// NON-SYSTEM
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PACKAGE
#include "aerial-mapper-dense-pcl/block-matching-base.h"
#include "aerial-mapper-dense-pcl/common.h"

namespace stereo {

/// Wrapper for opencv-implementation of semi-global block-matching (SGBM).
class BlockMatchingSGBM : public BlockMatchingBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BlockMatchingSGBM(const BlockMatchingParameters::SGBM& sgbm_params) {
    sgbm_ = cv::StereoSGBM::create(0, 0, 0);
    sgbm_->setMinDisparity(sgbm_params.min_disparity);
    sgbm_->setNumDisparities(sgbm_params.num_disparities);
    sgbm_->setPreFilterCap(sgbm_params.pre_filter_cap);
    sgbm_->setUniquenessRatio(sgbm_params.uniqueness_ratio);
    sgbm_->setSpeckleWindowSize(sgbm_params.speckle_window_size);
    sgbm_->setSpeckleRange(sgbm_params.speckle_range);
    sgbm_->setDisp12MaxDiff(sgbm_params.disp_12_max_diff);
    sgbm_->setP1(sgbm_params.p1);
    sgbm_->setP2(sgbm_params.p2);
    sgbm_->setBlockSize(sgbm_params.block_size);
  }

  void computeDisparityMap(const RectifiedStereoPair& rectified_stereo_pair,
                           DensifiedStereoPair* densified_stereo_pair);

 private:
  cv::Ptr<cv::StereoSGBM> sgbm_;
  BlockMatchingParameters::SGBM sgbm_parameters_;

  static constexpr int kMaxInvalidDisparity = 1;
};

}  // namespace stereo

#endif  // BLOCK_MATCHING_SGBM_H_
