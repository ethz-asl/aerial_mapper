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
#include <aerial-mapper-utils/utils-nearest-neighbor.h>

typedef kindr::minimal::QuatTransformation Pose;
typedef std::vector<Pose> Poses;
typedef cv::Mat Image;
typedef std::vector<Image> Images;

namespace io {

enum PoseFormat { Standard, COLMAP, PIX4D, ROS };

class AerialMapperIO {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AerialMapperIO();

  void loadPosesFromFile(const PoseFormat& format, const std::string& filename,
                         Poses* T_G_Bs);
  void loadPosesFromFileStandard(const std::string& filename, Poses* T_G_Bs);
  void loadImagesFromFile(const std::string& filename_base, size_t num_poses,
                          Images* images, bool load_colored_images = false);

  void loadImagesFromFile(const std::string& directory,
                          std::vector<std::string> image_names, Images* images,
                          bool load_colored_images = false);

  aslam::NCamera::Ptr loadCameraRigFromFile(
      const std::string& filename_ncameras_yaml);

  void loadPosesFromFileRos(const std::string& filename, Poses* T_G_Bs,
                            std::vector<int64_t>* timestamps_ns);
  void loadPix4dPosesFromFile(const std::string& filename_poses, Poses* poses,
                              std::vector<std::string>* image_names);

  void loadPointCloudFromFile(
      const std::string& filename_point_cloud,
      AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud_xyz,
      std::vector<int>* point_cloud_intensities);

  void loadPointCloudFromFile(
      const std::string& filename_point_cloud,
      AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud_xyz);

  void subtractOriginFromPoses(const Eigen::Vector3d& origin, Poses* T_G_Bs);

  void writeDataToDEMGeoTiffColor(const cv::Mat& ortho_image,
                                  const Eigen::Vector2d& xy,
                                  const std::string& geotiff_filename);

  void toGeoTiff(const cv::Mat& orthomosaic, const Eigen::Vector2d& xy,
                 const std::string& geotiff_filename);

  void toStandardFormat(const std::string& directory,
                        const std::string& filename_vi_imu_poses,
                        const std::string& filename_blender_id_time,
                        const std::string& new_directory);

  void convertFromSimulation();

  void exportPix4dGeofile(const Poses& T_G_Cs, const Images& images);
};
}  // namespace io
#endif  // namespace AERIAL_MAPPER_IO_H_
