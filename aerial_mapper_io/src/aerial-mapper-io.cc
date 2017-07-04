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

#include <cstdlib>
#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>

namespace io {

AerialMapperIO::AerialMapperIO() {}

void AerialMapperIO::loadPosesFromFile(const PoseFormat& format,
                                       const std::string& filename,
                                       Poses* T_G_Bs) {
  CHECK(T_G_Bs);
  CHECK(!filename.empty()) << "Empty filename";
  LOG(INFO) << "Loading body poses via format: " <<
               static_cast<int>(format);
  switch (format) {
    case PoseFormat::Standard:
      loadPosesFromFileStandard(filename, T_G_Bs);
    break;
    case PoseFormat::COLMAP:
      LOG(FATAL) << "Not yet implemented!";
    break;
    case PoseFormat::PIX4D:
      LOG(FATAL) << "Not yet implemented!";
    break;
  }
}

void AerialMapperIO::loadPosesFromFileStandard(
    const std::string& filename, Poses* T_G_Bs) {
  CHECK(T_G_Bs);
  CHECK(!filename.empty()) << "Empty filename";
  LOG(INFO) << "Loading body poses from: " << filename;
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

void AerialMapperIO::loadImagesFromFile(
    const std::string& filename_base, size_t num_poses, Images* images) {
  CHECK(images);
  LOG(INFO) << "Loading images from directory+prefix: " << filename_base;
  for (size_t i = 0u; i < num_poses; ++i) {
    const std::string& filename =
        filename_base + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("Image", image);
    cv::waitKey(1);
    images->push_back(image);
  }
  CHECK(images->size() > 0) << "No images loaded.";
  LOG(INFO) << "images->size() = " << images->size();
}

aslam::NCamera::Ptr AerialMapperIO::loadCameraRigFromFile(
    const std::string& filename_ncameras_yaml) {
  CHECK(filename_ncameras_yaml != "");
  aslam::NCamera::Ptr ncameras;
  LOG(INFO) << "Loading camera calibration from: "
            << filename_ncameras_yaml;
  ncameras = aslam::NCamera::loadFromYaml(filename_ncameras_yaml);
  CHECK(ncameras) << "Could not load the camera calibration from: "
                  << filename_ncameras_yaml;
  std::cout << "leaving" << std::endl;
  return ncameras;
}

void AerialMapperIO::subtractOriginFromPoses(const Eigen::Vector3d& origin,
                                             Poses* T_G_Bs) {
  CHECK(T_G_Bs);
  CHECK(T_G_Bs->size() > 0u);
  for (size_t i = 0u; i < T_G_Bs->size(); ++i) {
    (*T_G_Bs)[i].getPosition() = (*T_G_Bs)[i].getPosition() - origin;
  }
}

void AerialMapperIO::loadPointCloudFromFile(
    const std::string& filename_point_cloud,
    Aligned<std::vector, Eigen::Vector3d>::type* point_cloud_xyz) {
  CHECK(filename_point_cloud != "");
  CHECK(point_cloud_xyz);
  LOG(INFO) << "Loading pointcloud from: " << filename_point_cloud;
  std::ifstream infile(filename_point_cloud);
  double x, y, z;
  int intensity;
  while (infile >> x >> y >> z >> intensity) {
    if (z > -100) {
      point_cloud_xyz->push_back(Eigen::Vector3d(x, y, z));
    }
    if (infile.eof()) break;
  }
  CHECK(point_cloud_xyz->size() > 0);
}

void AerialMapperIO::loadPointCloudFromFile(
    const std::string& filename_point_cloud,
    Aligned<std::vector, Eigen::Vector3d>::type* point_cloud_xyz,
    std::vector<int>* point_cloud_intensities) {
  CHECK(filename_point_cloud != "");
  CHECK(point_cloud_xyz);
  CHECK(point_cloud_intensities);
  LOG(INFO) << "Loading pointcloud from: " << filename_point_cloud;
  std::ifstream infile(filename_point_cloud);
  double x, y, z;
  int intensity;
  while (infile >> x >> y >> z >> intensity) {
    if (z > -100) {
      point_cloud_xyz->push_back(Eigen::Vector3d(x, y, z));
      point_cloud_intensities->push_back(intensity);
    }
    if (infile.eof()) break;
  }
  CHECK(point_cloud_xyz->size() == point_cloud_intensities->size());
  CHECK(point_cloud_xyz->size() > 0);
}

void AerialMapperIO::writeDataToDEMGeoTiffColor(
    const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
    const std::string& geotiff_filename) {
  GDALAllRegister();
  std::string name = "GTiff";
  GDALDriver* poDriver;
  char** papszMetadata;
  poDriver = GetGDALDriverManager()->GetDriverByName(name.c_str());
  CHECK(poDriver != NULL);

  papszMetadata = poDriver->GetMetadata();
  if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE)) {
    VLOG(3) << "Driver " << name.c_str() << " supports Create() method.";
  }
  if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE)) {
    VLOG(3) << "Driver " << name.c_str() << " supports CreateCopy() method.";
  }

  GDALDataset* poDstDS;
  char** papszOptions = NULL;

  int height = ortho_image.rows;
  int width = ortho_image.cols;
  poDstDS = poDriver->Create(geotiff_filename.c_str(), width, height, 3,
                             GDT_Byte, papszOptions);

  double adfGeoTransform[6] = {xy(0), 1.0, 0.0, xy(1), 0.0, -1.0};
  OGRSpatialReference oSRS;
  char* pszSRS_WKT = NULL;
  //  GDALRasterBand *poBand;
  //  GByte abyRaster[512*512];
  poDstDS->SetGeoTransform(adfGeoTransform);

  int UTM = 32;
  std::string projection_cs = "UTM " + std::to_string(UTM) +
                              " (WGS84) in northern hemisphere.";
  oSRS.SetProjCS(projection_cs.c_str());
  oSRS.SetWellKnownGeogCS("WGS84");
  oSRS.SetUTM(UTM, TRUE);
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
      cv::Vec3b tmp = ortho_image.at<cv::Vec3b>(y, x);
      // TODO(hitimo): Fix color bands.
      pdata[x + y * width] = tmp(2);
      pdata2[x + y * width] = tmp(0);
      pdata3[x + y * width] = tmp(1);
    }
  }
  poDstDS->GetRasterBand(1)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size,
                                      ny_size, pdata, nBufxSize, nBufySize,
                                      GDT_Byte, 0, 0);
  poDstDS->GetRasterBand(2)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size,
                                      ny_size, pdata2, nBufxSize, nBufySize,
                                      GDT_Byte, 0, 0);
  poDstDS->GetRasterBand(3)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size,
                                      ny_size, pdata3, nBufxSize, nBufySize,
                                      GDT_Byte, 0, 0);
  // Once we're done, close properly the dataset.
  GDALClose((GDALDatasetH)poDstDS);
  VLOG(3) << "Closing the dataset.";
}

}  // namespace io

