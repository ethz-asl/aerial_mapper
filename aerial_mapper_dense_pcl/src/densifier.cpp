/*
 *    Filename: densifier.cpp
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-dense-pcl/densifier.h"

#include <ros/ros.h>

namespace stereo {

Densifier::Densifier(const BlockMatchingParameters& block_matching_params,
                     const cv::Size& image_resolution)
    : image_resolution_(image_resolution) {
  if (block_matching_params.use_BM) {
    block_matcher_.reset(new BlockMatchingBM(block_matching_params.bm));
  } else {
    block_matcher_.reset(new BlockMatchingSGBM(block_matching_params.sgbm));
  }
}

void Densifier::computePointCloud(
    const StereoRigParameters& stereo_pair,
    const RectifiedStereoPair& rectified_stereo_pair,
    DensifiedStereoPair* densified_stereo_pair,
    sensor_msgs::PointCloud2& point_cloud_ros) {
  CHECK(densified_stereo_pair);
  CHECK_EQ(image_resolution_, densified_stereo_pair->disparity_map.size());
  densified_stereo_pair->point_cloud.create(
      densified_stereo_pair->disparity_map.size());
  densified_stereo_pair->point_cloud.setTo(
      cv::Vec3f(kInvalidPoint, kInvalidPoint, kInvalidPoint));

  // Compute the stereo projection matrix Q.
  const double baseline = rectified_stereo_pair.baseline;
  CHECK_NE(baseline, 0.0);
  const Eigen::Matrix3d K = stereo_pair.K;
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  const Eigen::Matrix4d Q =
      (Eigen::Matrix4d() << 1, 0, 0, -cx, 0, fx / fy, 0, -cy * (fx / fy), 0, 0,
       0, fx, 0, 0, 1.0 / baseline, 0.0).finished();

  // Declare some pointers for faster loop execution.
  float* disparity_map_ptr;
  const unsigned char* pixel_intensity_ptr;
  int point_offset = 0;
  for (int v = 0; v < image_resolution_.height; ++v) {
    disparity_map_ptr = densified_stereo_pair->disparity_map.ptr<float>(v);
    pixel_intensity_ptr =
        rectified_stereo_pair.image_left.ptr<unsigned char>(v);
    for (int u = 0; u < image_resolution_.width; ++u) {
      point_offset += point_cloud_ros.point_step;
      bool point_valid = false;
      if (disparity_map_ptr[u] > kMaxInvalidDisparity) {
        // w = (1 / baseline) * disparity
        const double w = Q(3, 2) * disparity_map_ptr[u];
        CHECK_NE(w, 0.0);
        // x = (u - cx) * baseline / disparity
        // y = (fx / fy * v - cy) * baseline / disparity
        // z = (fx * baseline) / disparity
        // Point defined in rectified frame 1.
        const Eigen::Vector3d point_r1(
            (u + Q(0, 3)) / w, (Q(1, 1) * v + Q(1, 3)) / w, (Q(2, 3)) / w);

        // Point defined in world/global frame.
        const Eigen::Vector3d point_G(rectified_stereo_pair.R_G_C * point_r1 +
                                      stereo_pair.t_G_C1);
        const float x = point_G(0);
        const float y = point_G(1);
        const float z = point_G(2);
        if (!std::isinf(z)) {
          // Copy 3D point to ros message.
          memcpy(&(point_cloud_ros.data[kPositionX + point_offset]), &x,
                 kSizeOfFloat);
          memcpy(&(point_cloud_ros.data[kPositionY + point_offset]), &y,
                 kSizeOfFloat);
          memcpy(&(point_cloud_ros.data[kPositionZ + point_offset]), &z,
                 kSizeOfFloat);
          const uint8_t gray = pixel_intensity_ptr[u];
          const uint32_t rgb = (gray << 16) | (gray << 8) | gray;
          memcpy(&(point_cloud_ros.data[kPositionIntensity + point_offset]),
                 &rgb, kSizeOfUint32T);

          // Todo(hitimo): Consider pre-allocation...
          densified_stereo_pair->point_cloud_eigen.push_back(point_G);
          densified_stereo_pair->point_cloud_intensities.push_back(gray);
          point_valid = true;
        }
      }
      if (!point_valid) {
        memcpy(&point_cloud_ros.data[kPositionX + point_offset], &kInvalidPoint,
               kSizeOfFloat);
        memcpy(&point_cloud_ros.data[kPositionY + point_offset], &kInvalidPoint,
               kSizeOfFloat);
        memcpy(&point_cloud_ros.data[kPositionZ + point_offset], &kInvalidPoint,
               kSizeOfFloat);
        memcpy(&point_cloud_ros.data[kPositionIntensity + point_offset],
               &kInvalidPoint, kSizeOfFloat);
      }
    }
  }
}

}  // namespace stereo
