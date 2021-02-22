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

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

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
//    case PoseFormat::ROS:
//      loadPosesFromFileRos(filename, T_G_Bs);
//    break;
  }
}

void AerialMapperIO::loadPosesFromFileRos(
    const std::string& filename, Poses* T_G_Bs,
    std::vector<int64_t>* timestamps_ns) {
  CHECK(T_G_Bs);
  CHECK(!filename.empty()) << "Empty filename";
  CHECK(timestamps_ns);
  LOG(INFO) << "Loading body poses from: " << filename;
  std::ifstream infile(filename);
  double x, y, z, qw, qx, qy, qz;
  double timestamp_ns;
  // time, field.position.x, field.position.y, field.position.z,
  // field.orientation.x, field.orientation.y, field.orientation.z, field.orientation.w
  while (infile >> timestamp_ns >> x >> y >> z >> qx >> qy >> qz >> qw) {
    //std::cout << "timestamp_ns = " << timestamp_ns << std::endl;
    geometry_msgs::Pose pose;
    geometry_msgs::Point t;
    t.x = x;
    t.y = y;
    t.z = z;
    pose.position =  t;

    geometry_msgs::Quaternion q;
    q.x = qx;
    q.y = qy;
    q.z = qz;
    q.w = qw;

    pose.orientation = q;
    pose.position = t;

    kindr::minimal::QuatTransformation T_G_B;
    tf::poseMsgToKindr(pose, &T_G_B);

    timestamps_ns->push_back(timestamp_ns);
    T_G_Bs->push_back(T_G_B);

    if (infile.eof()) {
      break;
    }
  }
  CHECK(T_G_Bs->size() > 0) << "No poses loaded.";
  CHECK(T_G_Bs->size() == timestamps_ns->size());
  LOG(INFO) << "T_G_Bs->size() = " << T_G_Bs->size();
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

void AerialMapperIO::convertFromSimulation() {
  std::string directory = "/tmp/to_convert/";
  std::string filename_vi_imu_poses = "vi_imu_poses.csv";
  std::string filename_blender_id_time = "blender_id_time.csv";
  std::string new_directory = "/tmp/simulation2/";
  toStandardFormat(directory, filename_vi_imu_poses,
                   filename_blender_id_time, new_directory);
}

void AerialMapperIO::toStandardFormat(
    const std::string& directory,
    const std::string& filename_vi_imu_poses,
    const std::string& filename_blender_id_time,
    const std::string& new_directory) {

  // Load all available body poses with corresponding timestamps.
  Poses T_G_Bs;
  std::vector<int64_t> timestamps_ns;
  const std::string& path_filename_vi_imu_poses =
      directory + filename_vi_imu_poses;
  LOG(INFO) << "Loading from " << path_filename_vi_imu_poses;
  loadPosesFromFileRos(path_filename_vi_imu_poses, &T_G_Bs,
                                  &timestamps_ns);

  // Load the names and timestamps of the images.
  const std::string& path_filename_blender_id_time =
     directory + filename_blender_id_time;
  std::ifstream infile(path_filename_blender_id_time);
  double id, image_name;
  std::vector<std::string> image_names;
  std::vector<int64_t> image_timestamps;
  LOG(INFO) << "Loading from " << path_filename_blender_id_time;
  while (infile >> id >> image_name) {
    const std::string& image_name_string =
        std::to_string(static_cast<int64_t>(image_name));
    int64_t image_timestamp = static_cast<int64_t>(image_name) - 1;
    image_names.push_back(image_name_string);
    image_timestamps.push_back(image_timestamp);
    if (infile.eof()) {break;}
  }

  // Loop over image timestamps and find the corresponding poses.
  Poses T_G_Bs_associated;
  CHECK(timestamps_ns.size() == T_G_Bs.size());
  for (size_t i = 0u; i < image_timestamps.size(); ++i) {
    bool found_pose_for_image = false;
    Pose T_G_B;
    for (size_t j = 0u; j < timestamps_ns.size(); ++j) {
      if (timestamps_ns[j] == image_timestamps[i]) {
        found_pose_for_image = true;
        T_G_B = T_G_Bs[j];
      }
    }
    CHECK(found_pose_for_image);
    T_G_Bs_associated.push_back(T_G_B);
  }
  CHECK(T_G_Bs_associated.size() == image_timestamps.size());
  CHECK(T_G_Bs_associated.size() == image_names.size());

  // Save body poses to file with standard naming.ca
  std::string path_filename_poses = new_directory + "opt_poses.txt";
  std::ofstream fs;
  fs.open(path_filename_poses.c_str());
  for (const Pose& T_G_B : T_G_Bs_associated) {
    const Eigen::Vector3d& t = T_G_B.getPosition();
    const aslam::Quaternion& q = T_G_B.getRotation();
    fs << t(0) << " " << t(1) << " " << t(2) << " "
       << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
  }
  fs.close();

  // Load images from file.
  Images images;
  const std::string directory_images = directory + "cam0/";
  loadImagesFromFile(directory_images, image_names, &images);

  // Save with standard naming.
  for (size_t i = 0u; i < images.size(); ++i) {
    const std::string image_filename = new_directory + "image_"
        + std::to_string(i) + ".jpg";
    cv::imwrite(image_filename, images[i]);
  }
}

void AerialMapperIO::loadImagesFromFile(
    const std::string& filename_base, size_t num_poses, Images* images,
    bool load_colored_images) {
  CHECK(images);
  LOG(INFO) << "Loading images from directory+prefix: " << filename_base;
  for (size_t i = 0u; i < num_poses; ++i) {
    const std::string& filename =
        filename_base + std::to_string(i) + ".jpg";
    cv::Mat image;
    if (!load_colored_images) {
      image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    } else {
      image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    }
    cv::imshow("Image", image);
    cv::waitKey(1);
    images->push_back(image);
  }
  CHECK(images->size() > 0) << "No images loaded.";
  LOG(INFO) << "Number of images loaded: " << images->size();
}

void AerialMapperIO::loadImagesFromFile(
    const std::string& directory, std::vector<std::string> image_names,
    Images* images, bool load_colored_images) {
  CHECK(images);
  LOG(INFO) << "Loading images from directory: " << directory;
  for (const std::string image_name : image_names) {
    const std::string& filename =
        directory + image_name + ".png";
    cv::Mat image;
    if (!load_colored_images) {
      image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    } else {
      image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    }
    cv::imshow("Image", image);
    cv::waitKey(1);
    images->push_back(image);
  }
  CHECK(images->size() > 0) << "No images loaded.";
  LOG(INFO) << "Number of images loaded: " << images->size();
}

aslam::NCamera::Ptr AerialMapperIO::loadCameraRigFromFile(
    const std::string& filename_ncameras_yaml) {
  CHECK(filename_ncameras_yaml != "");
  aslam::NCamera::Ptr ncameras(new aslam::NCamera());
  LOG(INFO) << "Loading camera calibration from: "
            << filename_ncameras_yaml;
  ncameras->deserializeFromFile(filename_ncameras_yaml);
  CHECK(ncameras) << "Could not load the camera calibration from: "
                  << filename_ncameras_yaml;
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

void AerialMapperIO::exportPix4dGeofile(const Poses& T_G_Cs,
                                        const Images& images) {
  CHECK(images.size() == T_G_Cs.size());
  std::string output_directory = "/tmp/pix4d";
  std::string filename = output_directory + "/geofile.txt";
  LOG(INFO) << "Writing geofile in " << filename;
  std::ofstream fs;
  fs.open(filename.c_str());

  size_t georeference_every_nth_image = 1;
  size_t image_number = 0u;
  size_t i = 0u;
  for (const Image& image : images) {
    ++image_number;
    if (image_number % georeference_every_nth_image == 0) {
      std::stringstream ss;
      ss << std::setw(10) << std::setfill('0') << image_number;
      std::string image_filename =
          output_directory + "/image_" + ss.str() + ".jpeg";
      cv::imwrite(image_filename, image);
      cv::imshow("image", image);
      cv::waitKey(100);
      LOG(INFO) << "Saved image " << image_filename;
      Eigen::Vector3d xyz = T_G_Cs[i].getPosition();

      fs << "image_"
         << ss.str()
         << ".jpeg "
         << std::setprecision(15) << xyz(0) << " "
         << std::setprecision(15) << xyz(1) << " "
         << std::setprecision(15) << xyz(2) << std::endl;
      ++i;
    }
  }
  fs.close();
}

void AerialMapperIO::loadPointCloudFromFile(
    const std::string& filename_point_cloud,
    AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud_xyz) {
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
    AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud_xyz,
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

void AerialMapperIO::toGeoTiff(    const cv::Mat& orthomosaic,
                                   const Eigen::Vector2d& xy,
                                   const std::string& geotiff_filename) {


  // Create a geotiff with GDAL.
  GDALAllRegister();

  //  const char *pszFormat = "GTiff";
  std::string name = "GTiff";
  GDALDriver *poDriver;
  char **papszMetadata;
  poDriver = GetGDALDriverManager()->GetDriverByName(name.c_str());
  //  if( poDriver == NULL )
  //    exit( 1 );
  papszMetadata = poDriver->GetMetadata();
  if( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
    printf( "Driver %s supports Create() method.\n", name.c_str() );
  if( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATECOPY, FALSE ) )
    printf( "Driver %s supports CreateCopy() method.\n", name.c_str() );

  GDALDataset* poDstDS;
  char **papszOptions = NULL;

  int height = orthomosaic.rows;
  int width = orthomosaic.cols;
  std::cout << "height = " << height << std::endl;
  std::cout << "width = " << width << std::endl;
  poDstDS = poDriver->Create(geotiff_filename.c_str(), width, height,
                             1, GDT_Byte, papszOptions );

  //double adfGeoTransform[6] = {464736.27, 1.0, 0.0, 5272359.16, 0.0, -1.0};
  double adfGeoTransform[6] = {464499.00, 1.0, 0.0, 5.2727e+06, 0.0, -1.0};
  //{ 444720, 30, 0, 3751320, 0, -30 };
  OGRSpatialReference oSRS;
  char *pszSRS_WKT = NULL;
  GDALRasterBand *poBand;
  GByte abyRaster[512*512];
  poDstDS->SetGeoTransform(adfGeoTransform );
  //  oSRS.SetUTM( 11, TRUE );
  //  oSRS.SetWellKnownGeogCS( "NAD27" );

  oSRS.SetProjCS( "UTM 32 (WGS84) in northern hemisphere." );
  oSRS.SetWellKnownGeogCS( "WGS84" );
  oSRS.SetUTM(32, TRUE );
  oSRS.exportToWkt( &pszSRS_WKT );
  poDstDS->SetProjection( pszSRS_WKT );

  CPLFree( pszSRS_WKT );
  // poBand = poDstDS->GetRasterBand(1);
  //  poBand->RasterIO( GF_Write, 0, 0, 512, 512,
  //                    abyRaster, 512, 512, GDT_Byte, 0, 0 );

  int nx_offset = 0;
  int ny_offset = 0;
  int nx_size = width;
  int ny_size = height;
  int nBufxSize = width;
  int nBufySize = height;
  unsigned char pdata[width * height];


  for (size_t y = 0u; y < height; ++y) {
    for (size_t x = 0u; x < width; ++x) {
      CHECK(x < width);
      CHECK(y < height);
//      std::cout << "x = " << x << ", y = " << y << ", access. y*width+x = " << y*width+x << std::endl;
      pdata[x + y * width]= orthomosaic.at<uchar>(y,x);
    }
  }

  //  for (int n=0; n<width; n++) {
  //    for (int m=0; m<height; m++) {
  //      std::cout << "m = " << m << ", n = " << n << ", access. n*width+m = " << n*width+m << std::endl;

  //    }
  //  }
  poDstDS->GetRasterBand(1)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size, ny_size,
                                      pdata, nBufxSize, nBufySize, GDT_Byte, 0, 0 );
  /* Once we're done, close properly the dataset */
  GDALClose( (GDALDatasetH) poDstDS );
  std::cout << "closing the dataset." << std::endl;
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

