/*
 *    Filename: block-matching-base.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef BLOCK_MATCHING_BASE_H_
#define BLOCK_MATCHING_BASE_H_

// NON-SYSTEM
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/node_handle.h>

#include "aerial-mapper-dense-pcl/common.h"

namespace stereo {

/// Base class for blockmatching methods.
class BlockMatchingBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BlockMatchingBase() {}

  virtual void computeDisparityMap(
      const RectifiedStereoPair& rectified_stereo_pair,
      DensifiedStereoPair* densified_stereo_pair) = 0;
};

}  // namespace stereo

#endif  // BLOCK_MATCHING_BASE_H_
