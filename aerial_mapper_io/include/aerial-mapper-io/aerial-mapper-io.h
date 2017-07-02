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

enum PoseFormat { Standard, COLMAP, PIX4D };

class AerialMapperIO {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AerialMapperIO();

  void loadPosesFromFile(const PoseFormat& format,
                         const std::string& filename,
                         Poses* T_G_Bs);
  void loadPosesFromFileStandard(const std::string& filename,
                                 Poses* T_G_Bs);
  void loadImagesFromFile(const std::string& filename_base,
                          size_t num_poses,
                          Images* images);
  aslam::NCamera::Ptr loadCameraRigFromFile(
       const std::string& filename_ncameras_yaml);

  void loadPointCloudFromFile(
      const std::string& filename_point_cloud,
      Aligned<std::vector, Eigen::Vector3d>::type* point_cloud_xyz,
      std::vector<int>* point_cloud_intensities);

  void loadPointCloudFromFile(
      const std::string& filename_point_cloud,
      Aligned<std::vector, Eigen::Vector3d>::type* point_cloud_xyz) {}
};
}  // namespace io
#endif  // namespace AERIAL_MAPPER_IO_H_

#ifdef sadf
/// Load pointcloud from ros message.
void loadPointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const Eigen::Vector3d& origin,
                    Aligned<std::vector, Eigen::Vector3d>::type* pointcloud);

/// Load pointcloud from file.
void loadPointcloud(const std::string& filename,
                    Aligned<std::vector, Eigen::Vector3d>::type* pointcloud);

void writeDataToDEMGeoTiff(const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
                           const std::string filename);

void writeDataToDEMGeoTiffColor(const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
                                const std::string filename);
#endif




#ifdef asdf
from dsm

/// Load pointcloud from ros message.
void loadPointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const Eigen::Vector3d& origin,
                    Aligned<std::vector, Eigen::Vector3d>::type* pointcloud);

/// Load pointcloud from file.
void loadPointcloud(const std::string& filename,
                    Aligned<std::vector, Eigen::Vector3d>::type* pointcloud);

void writeDataToDEMGeoTiff(const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
                           const std::string filename);

void writeDataToDEMGeoTiffColor(const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
                                const std::string filename);


// output into files:
// [A] GeoTiff
const std::string base =
    "geotiff_color_" + std::to_string(FLAGS_DEM_color_palette) +
    "_radius_" + std::to_string(FLAGS_DEM_interpolation_radius) +
    "_resolution_" + std::to_string(FLAGS_DEM_resolution);
const std::string filename_maximum = FLAGS_DEM_output_folder + base + "_maximum.tiff";
const std::string filename_minimum = FLAGS_DEM_output_folder + base + "_minimum.tiff";
const std::string filename_mean = FLAGS_DEM_output_folder + base + "_mean.tiff";
const std::string filename_idw = FLAGS_DEM_output_folder + base + "_idw.tiff";
writeDataToDEMGeoTiffColor(ortho_image_maximum, top_left, filename_maximum);
writeDataToDEMGeoTiffColor(ortho_image_minimum, top_left, filename_minimum);
writeDataToDEMGeoTiffColor(ortho_image_mean, top_left, filename_mean);
writeDataToDEMGeoTiffColor(ortho_image_idw, top_left, filename_idw);

//    // [B] USGS DEM ASCII
//    // TODO(hitimo): Implement ASCII grid.
//    if (FLAGS_DEM_save_cv_mat_height_map) {
//      cv::imwrite("/tmp/height_map.jpeg",height_map);
//      cv::FileStorage file("/tmp/height_map.mat", cv::FileStorage::WRITE);
//      std::cout << "height_map.cols=" << height_map.cols
//                << "height_map.rows=" << height_map.rows << std::endl;
//      file << "height_map" <<  height_map;
//      file.release();
//    }
}

void Dsm::writeDataToDEMGeoTiffColor(
const cv::Mat& ortho_image, const Eigen::Vector2d& xy, const std::string geotiff_filename) {
GDALAllRegister();
std::string name = "GTiff";
GDALDriver *poDriver;
char **papszMetadata;
poDriver = GetGDALDriverManager()->GetDriverByName(name.c_str());
CHECK(poDriver != NULL);

papszMetadata = poDriver->GetMetadata();
if( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) ) {
VLOG(3) << "Driver " << name.c_str() << " supports Create() method.";
}
if ( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATECOPY, FALSE ) ) {
VLOG(3) << "Driver " << name.c_str() << " supports CreateCopy() method.";
}

GDALDataset* poDstDS;
char **papszOptions = NULL;

int height = ortho_image.rows;
int width = ortho_image.cols;
poDstDS = poDriver->Create(geotiff_filename.c_str(), width, height, 3, GDT_Byte, papszOptions );

double adfGeoTransform[6] = {xy(0), 1.0, 0.0, xy(1), 0.0, -1.0};
OGRSpatialReference oSRS;
char *pszSRS_WKT = NULL;
//  GDALRasterBand *poBand;
//  GByte abyRaster[512*512];
poDstDS->SetGeoTransform(adfGeoTransform );

std::string projection_cs =
  "UTM " + std::to_string(FLAGS_DEM_UTM_code) + " (WGS84) in northern hemisphere.";
oSRS.SetProjCS(projection_cs.c_str());
oSRS.SetWellKnownGeogCS("WGS84");
oSRS.SetUTM(FLAGS_DEM_UTM_code, TRUE);
oSRS.exportToWkt(&pszSRS_WKT);
poDstDS->SetProjection(pszSRS_WKT);
CPLFree(pszSRS_WKT);

int nx_offset = 0;
int ny_offset = 0;
int nx_size = width;
int ny_size = height;
int nBufxSize = width;
int nBufySize = height;
unsigned char pdata[width * height];
unsigned char pdata2[width * height];
unsigned char pdata3[width * height];

for (size_t y = 0u; y < static_cast<size_t>(height); ++y) {
for (size_t x = 0u; x < static_cast<size_t>(width); ++x) {
CHECK(x < static_cast<size_t>(width));
CHECK(y < static_cast<size_t>(height));
cv::Vec3b tmp = ortho_image.at<cv::Vec3b>(y,x);
// TODO(hitimo): Fix color bands.
pdata[x + y * width]=tmp(2);
pdata2[x + y * width]=tmp(0);
pdata3[x + y * width]=tmp(1);
}
}
poDstDS->GetRasterBand(1)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size, ny_size,
                                  pdata, nBufxSize, nBufySize, GDT_Byte, 0, 0 );
poDstDS->GetRasterBand(2)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size, ny_size,
                                  pdata2, nBufxSize, nBufySize, GDT_Byte, 0, 0 );
poDstDS->GetRasterBand(3)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size, ny_size,
                                  pdata3, nBufxSize, nBufySize, GDT_Byte, 0, 0 );
// Once we're done, close properly the dataset.
GDALClose( (GDALDatasetH) poDstDS );
VLOG(3) << "Closing the dataset.";
}

void Dsm::loadPointcloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const Eigen::Vector3d& origin,
    Aligned<std::vector, Eigen::Vector3d>::type* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  for (size_t i = 0u; i < cloud.size(); ++i) {
    pointcloud->emplace_back(
        Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z) + origin);
  }
  VLOG(3) << "Pointcloud size: " << pointcloud->size();
}

void Dsm::loadPointcloud(
    const std::string& filename,
    Aligned<std::vector, Eigen::Vector3d>::type* pointcloud) {
  VLOG(3) << "Loading pointcloud from file: " << filename;
  std::ifstream infile(filename);
  if (infile) {
    while (!infile.eof()) {
      Eigen::Vector3d point;
      infile >> point(0) >> point(1) >> point(2);
      pointcloud->emplace_back(point);
    }
  }
  VLOG(3) << "Pointcloud size: " << pointcloud->size();
}


#endif


