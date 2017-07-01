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

namespace io {

AerialMapperIO::AerialMapperIO() {}

void AerialMapperIO::loadPosesFromFile(const PoseFormat& format,
                                       const std::string& filename,
                                       Poses* T_G_Bs) {
  CHECK(T_G_Bs);
  switch (format) {
    case PoseFormat::Standard:
      loadPosesFromFileStandard(filename, T_G_Bs);
    case PoseFormat::COLMAP:
      LOG(FATAL) << "Not yet implemented!";
    case PoseFormat::PIX4D:
      LOG(FATAL) << "Not yet implemented!";
  }
}

void AerialMapperIO::loadPosesFromFileStandard(const std::string& filename,
                                               Poses* T_G_Bs) {
  CHECK(T_G_Bs);
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

void AerialMapperIO::loadImagesFromFile(const std::string& filename_base,
                                        size_t num_poses, Images* images) {
  CHECK(images);
  for (size_t i = 0u; i < num_poses; ++i) {
    const std::string& filename = filename_base + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("Image", image);
    cv::waitKey(1);
    images->push_back(image);
  }
  CHECK(images->size() > 0) << "No images loaded.";
  LOG(INFO) << "images->size() = " << images->size();
}

void AerialMapperIO::loadCameraRigFromFile(
    const std::string& filename_ncameras_yaml,
    std::shared_ptr<aslam::NCamera> ncameras) {
  ncameras = aslam::NCamera::loadFromYaml(filename_ncameras_yaml);
  CHECK(ncameras) << "Could not load the camera calibration from: "
                  << filename_ncameras_yaml;
}

void AerialMapperIO::loadPointCloudFromFile(
    const std::string& filename_point_cloud,
    Aligned<std::vector, Eigen::Vector3d>::type* point_cloud_xyz,
    std::vector<int>* point_cloud_intensities) {
  CHECK(point_cloud_xyz);
  CHECK(point_cloud_intensities);
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

}  // namespace io

// ###########################################################
// ############################################################

#ifdef asdf
void OrthomosaicBackprojection::loadCameraRig(
    std::string ncameras_yaml_path_filename) {
  ncameras_ = aslam::NCamera::loadFromYaml(ncameras_yaml_path_filename);
  CHECK(ncameras_) << "Could not load the camera calibration from: "
                   << ncameras_yaml_path_filename;
}

#include <vi-map-dmap/table-file-io.h>
#include <vi-map-dmap/view.h>
#include <vi-map-dmap/vi-map-dmap.h>

void OrthomosaicBackprojection::loadPosesAndImagesFromViMap(
    std::string path_vi_map, aslam::TransformationVector* T_G_Cs,
    std::vector<cv::Mat>* images) {
  CHECK_NOTNULL(T_G_Cs);
  CHECK_NOTNULL(images);
  vi_map_dmap::init();
  VLOG(1) << "Load the VI-Map from " << path_vi_map;
  vi_map_dmap::table_file_io::loadAllTablesFromDisk(path_vi_map);
  std::unique_ptr<vi_map_dmap::View> view;
  view.reset(new vi_map_dmap::View());
  std::unique_ptr<viwls_graph::VIMap> map;
  map.reset(new viwls_graph::VIMap);
  map->shallowCopyFrom(view->map());
  CHECK(map);
  VLOG(1) << "VI-Map successfuly loaded.";

  VLOG(1) << "Get all missions";

  viwls_graph::MissionIdList all_missions;
  map->getAllMissionIds(&all_missions);

  VLOG(1) << "Get all vertices.";
  pose_graph::VertexIdList all_vertices;
  for (const viwls_graph::MissionId& mission_id : all_missions) {
    pose_graph::VertexIdList mission_vertices;
    map->getAllVertexIdsInMission(mission_id, &mission_vertices);
    all_vertices.insert(all_vertices.end(), mission_vertices.begin(),
                        mission_vertices.end());
  }
  VLOG(1) << "No vertices: " << all_vertices.size();

  VLOG(1) << "Get Camera rig.";
  ncameras_ =
      map->getVertex(all_vertices[0]).getVisualNFrame().getNCameraShared();
  const aslam::Transformation& T_B_C =
      ncameras_->get_T_C_B(kFrameIdx).inverse();

  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    cv::Mat image_raw;
    const viwls_graph::Vertex& vertex = map->getVertex(vertex_id);
    map->getCvMatResource(vertex, kFrameIdx,
                          viwls_graph::ResourceType::kRawImage, &image_raw);
    T_G_Cs->emplace_back(vertex.get_T_M_I() * T_B_C);
    images->emplace_back(image_raw);
    CHECK(T_G_Cs->size() == images->size());
  }
  VLOG(1) << "Extracted " << T_G_Cs->size() << " poses.";
}

void OrthomosaicBackprojection::loadPosesAndImagesFromPix4d(
    std::string filename_poses, std::string filename_images,
    std::string ncameras_yaml_path_filename,
    aslam::TransformationVector* T_G_Cs, std::vector<cv::Mat>* images) {
  CHECK_NOTNULL(T_G_Cs);
  CHECK_NOTNULL(images);

  std::vector<std::string> image_names;
  loadPosesFromPix4d(filename_poses, T_G_Cs, &image_names);
  CHECK(T_G_Cs->size() == image_names.size());
  loadImages(filename_images, image_names, images);
  CHECK(T_G_Cs->size() == images->size());
  initializeCameraRig(ncameras_yaml_path_filename);
}

void OrthomosaicBackprojection::loadImages(
    const std::string& image_directory,
    const std::vector<std::string>& image_names, std::vector<cv::Mat>* images) {
  CHECK_NOTNULL(images);
  CHECK(image_names.size());
  images->clear();
  for (std::string image_name : image_names) {
    VLOG(100) << "Loading image: " << image_directory + image_name;
    cv::Mat image =
        cv::imread(image_directory + image_name, CV_LOAD_IMAGE_GRAYSCALE);
    // cv::Mat img_tmp;
    // cv::equalizeHist(image, img_tmp);
    images->push_back(image);
    //  images->push_back(image);
    cv::imshow("last_image", image);
    cv::waitKey(1);
  }
  CHECK(images->size() == image_names.size());
}

void OrthomosaicBackprojection::loadPosesFromPix4d(
    std::string filename_poses, aslam::TransformationVector* poses,
    std::vector<std::string>* image_names) {
  VLOG(3) << "Loading poses from file: " << filename_poses;
  std::ifstream infile(filename_poses);
  std::string header;
  std::getline(infile, header);
  VLOG(3) << "Header: " << header;

  poses->clear();
  image_names->clear();
  if (infile) {
    std::string image_string;
    double latitude, longitude, altitude, omega, phi, kappa;
    while (infile >> image_string >> longitude >> latitude >> altitude >>
           omega >> phi >> kappa) {
      double cw = cos(omega * M_PI / 180.0);
      double cp = cos(phi * M_PI / 180.0);
      double ck = cos(kappa * M_PI / 180.0);
      double sw = sin(omega * M_PI / 180.0);
      double sp = sin(phi * M_PI / 180.0);
      double sk = sin(kappa * M_PI / 180.0);

      Eigen::Matrix3d R_match_conventions, R;
      R_match_conventions(0, 0) = 1.0;
      R_match_conventions(1, 1) = -1.0;
      R_match_conventions(2, 2) = -1.0;

      R << cp* ck, cw* sk + sw* sp* ck, sw* sk - cw* sp* ck, -cp* sk,
          cw* ck - sw* sp* sk, sw* ck + cw* sp* sk, sp, -sw* cp, cw* cp;

      double northing, easting;
      char utm_zone[10];
      UTM::LLtoUTM(latitude, longitude, northing, easting, utm_zone);
      const Eigen::Vector3d t(easting, northing, altitude);

      Eigen::Matrix4d T_;
      T_.block<3, 3>(0, 0) = (R_match_conventions * R).transpose();
      T_.block<3, 1>(0, 3) = t;
      aslam::Transformation T(T_);
      poses->push_back(T);

      VLOG(100) << "----------------------";
      VLOG(100) << image_string;
      VLOG(100) << std::endl << T.getTransformationMatrix();
      image_names->push_back(image_string);
    }
  }
  CHECK(poses->size() == image_names->size());
  VLOG(3) << "Number of poses: " << poses->size();
}

void OrthomosaicBackprojection::loadHeightMap(const std::string filename,
                                              cv::Mat* height_map) {
  CHECK_NOTNULL(height_map);
  cv::FileStorage file("/tmp/height_map.mat", cv::FileStorage::READ);
  file["height_map"] >> *height_map;
  file.release();
  std::cout << "height_map.cols =" << height_map->cols << std::endl;
  std::cout << "height_map.rows = " << height_map->rows << std::endl;
  //  for (size_t i = 0u; i < height_map.cols; ++i) {
  //      for (size_t j = 0u; j < height_map.rows; ++j) {
  //        std::cout << "i/j: " << i << "/" << j << ", " <<
  //                     height_map.at<double>(j,i) << std::endl;
  //      }
  //    }
}

#include <cstdlib>
#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>

exportOrthoBatchBackwardAsGeoTiff() {
  bool create_geotiff = true;
  if (create_geotiff) {
    // Create a geotiff with GDAL.
    GDALAllRegister();
    std::string name = "GTiff";
    GDALDriver* poDriver;
    char** papszMetadata;
    poDriver = GetGDALDriverManager()->GetDriverByName(name.c_str());
    papszMetadata = poDriver->GetMetadata();
    if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE)) {
      printf("Driver %s supports Create() method.\n", name.c_str());
    }
    if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE)) {
      printf("Driver %s supports CreateCopy() method.\n", name.c_str());
    }

    std::string geotiff_filename = "/tmp/test_geo3.tiff";
    GDALDataset* poDstDS;
    char** papszOptions = NULL;

    int height = orthomosaic.rows;
    int width = orthomosaic.cols;
    poDstDS = poDriver->Create(geotiff_filename.c_str(), width, height, 1,
                               GDT_Byte, papszOptions);

    double adfGeoTransform[6] = {
        464499.00,  1.0, 0.0,
        5272750.00, 0.0, -1.0};  //{ 444720, 30, 0, 3751320, 0, -30 };
    OGRSpatialReference oSRS;
    char* pszSRS_WKT = NULL;
    GDALRasterBand* poBand;
    GByte abyRaster[512 * 512];
    poDstDS->SetGeoTransform(adfGeoTransform);
    oSRS.SetProjCS("UTM 32 (WGS84) in northern hemisphere.");
    oSRS.SetWellKnownGeogCS("WGS84");
    oSRS.SetUTM(32, TRUE);
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

    for (size_t y = 0u; y < height; ++y) {
      for (size_t x = 0u; x < width; ++x) {
        CHECK(x < width);
        CHECK(y < height);
        LOG(INFO) << "x = " << x << ", y = " << y
                  << ", access. y*width+x = " << y * width + x;
        pdata[x + y * width] = orthomosaic.at<cv::Vec3b>(y, x)[0];
      }
    }
    poDstDS->GetRasterBand(1)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size,
                                        ny_size, pdata, nBufxSize, nBufySize,
                                        GDT_Byte, 0, 0);
    GDALClose((GDALDatasetH)poDstDS);
    LOG(INFO) << "Closing the dataset.";
  }
}

void FwOnlineDigitalElevationMap::writeDataToDEMGeoTiffColor(
    const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
    const std::string geotiff_filename) {
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

  std::string projection_cs = "UTM " + std::to_string(FLAGS_DEM_UTM_code) +
                              " (WGS84) in northern hemisphere.";
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

// DSM
// output into files:
// [A] GeoTiff
//    const std::string base =
//        "geotiff_color_" + std::to_string(FLAGS_DEM_color_palette) +
//        "_radius_" + std::to_string(FLAGS_DEM_interpolation_radius) +
//        "_resolution_" + std::to_string(FLAGS_DEM_resolution);
//    const std::string filename_maximum = FLAGS_DEM_output_folder + base +
//    "_maximum.tiff";
//    const std::string filename_minimum = FLAGS_DEM_output_folder + base +
//    "_minimum.tiff";
//    const std::string filename_mean = FLAGS_DEM_output_folder + base +
//    "_mean.tiff";
//    const std::string filename_idw = FLAGS_DEM_output_folder + base +
//    "_idw.tiff";
//    writeDataToDEMGeoTiffColor(ortho_image_maximum, top_left,
//    filename_maximum);
//    writeDataToDEMGeoTiffColor(ortho_image_minimum, top_left,
//    filename_minimum);
//    writeDataToDEMGeoTiffColor(ortho_image_mean, top_left, filename_mean);
//    writeDataToDEMGeoTiffColor(ortho_image_idw, top_left, filename_idw);

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

void FwOnlineDigitalElevationMap::loadPointcloud(
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

void FwOnlineDigitalElevationMap::loadPointcloud(
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
