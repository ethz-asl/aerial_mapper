/*
 *    Filename: block-matching-bm.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef BLOCK_MATCHING_BM_H_
#define BLOCK_MATCHING_BM_H_

// NON-SYSTEM
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PACKAGE
#include "aerial-mapper-dense-pcl/block-matching-base.h"
#include "aerial-mapper-dense-pcl/common.h"

namespace stereo {

/// Wrapper for opencv-implementation of block-matching (BM).
class BlockMatchingBM : public BlockMatchingBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BlockMatchingBM(const BlockMatchingParameters::BM& bm_params) {
    bm_ = cv::StereoBM::create(0, 0);
    bm_->setMinDisparity(bm_params.min_disparity);
    bm_->setNumDisparities(bm_params.num_disparities);
    bm_->setPreFilterCap(bm_params.pre_filter_cap);
    bm_->setPreFilterCap(bm_params.pre_filter_size);
    bm_->setUniquenessRatio(bm_params.uniqueness_ratio);
    bm_->setTextureThreshold(bm_params.texture_threshold);
    bm_->setSpeckleWindowSize(bm_params.speckle_window_size);
    bm_->setSpeckleRange(bm_params.speckle_range);
    bm_->setBlockSize(bm_params.block_size);
  }

  void computeDisparityMap(const RectifiedStereoPair& rectified_stereo_pair,
                           DensifiedStereoPair* densified_stereo_pair) const;

 private:
  cv::Ptr<cv::StereoBM> bm_;
  BlockMatchingParameters::BM bm_parameters_;
  static constexpr int kMaxInvalidDisparity = 1;
};

}  // namespace stereo

#endif  // BLOCK_MATCHING_BM_H_
