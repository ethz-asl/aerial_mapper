#include "fw_online_digital_elevation_map_node/fw-digital-elevation-map.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

DECLARE_string(DEM_filename_xyz);
DECLARE_string(DEM_output_folder);
DECLARE_double(DEM_interpolation_radius);
DECLARE_int32(DEM_color_palette);
DECLARE_double(DEM_resolution);
DECLARE_int32(DEM_UTM_code);
DECLARE_bool(DEM_show_output);
DECLARE_bool(DEM_save_cv_mat_height_map);
DECLARE_double(DEM_easting_min);
DECLARE_double(DEM_northing_min);
DECLARE_double(DEM_easting_max);
DECLARE_double(DEM_northing_max);

FwOnlineDigitalElevationMap::FwOnlineDigitalElevationMap(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const Eigen::Vector3d& origin) {
  Aligned<std::vector, Eigen::Vector3d>::type pointcloud;
  loadPointcloud(cloud_msg, origin, &pointcloud);
  process(pointcloud);
}

void FwOnlineDigitalElevationMap::loadPointcloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const Eigen::Vector3d& origin,
    Aligned<std::vector, Eigen::Vector3d>::type* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  for (size_t i = 0u; i < cloud.size(); ++i) {
    pointcloud->emplace_back(Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z) + origin);
  }
  VLOG(3) << "Pointcloud size: " << pointcloud->size();
}

void FwOnlineDigitalElevationMap::loadPointcloud(
    const std::string& filename, Aligned<std::vector, Eigen::Vector3d>::type* pointcloud) {
  VLOG(3) << "Loading pointcloud from file: " << filename;
  std::ifstream infile(filename);
  if(infile) {
   while (!infile.eof()){
     Eigen::Vector3d point;
     infile >> point(0) >> point(1) >> point(2);
     pointcloud->emplace_back(point);
   }
  }
  VLOG(3) << "Pointcloud size: " << pointcloud->size();
}

void FwOnlineDigitalElevationMap::process(const Aligned<std::vector,
                                          Eigen::Vector3d>::type& pointcloud) {
  CHECK(!pointcloud.empty());
  const int nameWidth = 30;
    std::cout << "**********************************************************************************************" << std::endl
              << "Starting Digital Elevation Map generation" << std::endl
              << std::left << std::setw(nameWidth) << " - Resolution: "
              << std::left << std::setw(nameWidth) << std::to_string(FLAGS_DEM_resolution) << std::endl
              << std::left << std::setw(nameWidth) << " - Interp. radius: "
              << std::left << std::setw(nameWidth) << std::to_string(FLAGS_DEM_interpolation_radius) << std::endl
              << std::left << std::setw(nameWidth) << " - Filename XYZ: "
              << std::left << std::setw(nameWidth) << FLAGS_DEM_filename_xyz << std::endl
              << std::left << std::setw(nameWidth) << " - Output folder: "
              << std::left << std::setw(nameWidth) << FLAGS_DEM_output_folder << std::endl
              << std::left << std::setw(nameWidth) << " - Color Palette: "
              << std::left << std::setw(nameWidth) << std::to_string(FLAGS_DEM_color_palette) << std::endl
              << std::left << std::setw(nameWidth) << " - UTM code: "
              << std::left << std::setw(nameWidth) << std::to_string(FLAGS_DEM_UTM_code) << std::endl;
    std::cout << "**********************************************************************************************" << std::endl;

    // Insert pointcloud in kdtree.
    PointCloud<double> cloud_kdtree;
    cloud_kdtree.pts.resize(pointcloud.size());
    Eigen::Vector2d min_xy(std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max());
    Eigen::Vector2d max_xy(std::numeric_limits<double>::min(),
                           std::numeric_limits<double>::min());
    double min_height = std::numeric_limits<double>::max();
    double max_height = std::numeric_limits<double>::min();

    std::cout << "Num points: " << pointcloud.size() << std::endl;
    for (size_t i = 0u; i < pointcloud.size(); ++i) {
      cloud_kdtree.pts[i].x = pointcloud[i](0);
      cloud_kdtree.pts[i].y = pointcloud[i](1);
      cloud_kdtree.pts[i].z = pointcloud[i](2);

      // Save boundaries.
      if (cloud_kdtree.pts[i].x < min_xy(0)) {
        min_xy(0) = cloud_kdtree.pts[i].x;
      }
      if (cloud_kdtree.pts[i].y < min_xy(1)) {
        min_xy(1) = cloud_kdtree.pts[i].y;
      }
      if (cloud_kdtree.pts[i].x > max_xy(0)) {
        max_xy(0) = cloud_kdtree.pts[i].x;
      }
      if (cloud_kdtree.pts[i].x > max_xy(1)) {
        max_xy(1) = cloud_kdtree.pts[i].y;
      }
      if (cloud_kdtree.pts[i].z < min_height) {
        min_height = cloud_kdtree.pts[i].z;
      }
      if (cloud_kdtree.pts[i].z > max_height) {
        max_height = cloud_kdtree.pts[i].z;
      }
    }
    VLOG(10) << "Minimum height = " << min_height;
    VLOG(10) << "Maximum height = " << max_height;

    VLOG(100) << "Construct a kd-tree index.";
    typedef PointCloudAdaptor<PointCloud<double> > PC2KD;
    const PC2KD pc2kd(cloud_kdtree);
    const size_t kDimensionKdTree = 2u;
    const size_t kMaxLeaf = 100u;
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<double, PC2KD>, PC2KD, kDimensionKdTree> my_kd_tree_t;
    my_kd_tree_t index(kDimensionKdTree, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf));
    index.buildIndex();

    VLOG(100) << "Define the grid.";
//    const Eigen::Vector2d bottom_left = Eigen::Vector2d(FLAGS_DEM_easting_min,
//                                                        FLAGS_DEM_northing_min);
    const Eigen::Vector2d bottom_left = min_xy;
    //const Eigen::Vector2d bottom_left(-200, -200);


//    const Eigen::Vector2d top_right = Eigen::Vector2d(FLAGS_DEM_easting_max,
//                                                      FLAGS_DEM_northing_max);
    const Eigen::Vector2d top_right = max_xy;
    // Eigen::Vector2d top_right(200,200);
    const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
    const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
    const Eigen::Vector2d top_left = bottom_left + Eigen::Vector2d(0.0, height_north);

    VLOG(100) << "Color palette.";
    palette pal = GetPalette(static_cast<palette::palettetypes>(FLAGS_DEM_color_palette));

    VLOG(100) << "Loop over all cells.";
    bool adaptive_interpolation = true;
    const double d = static_cast<double>(FLAGS_DEM_resolution);
    cv::Mat ortho_image_maximum(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat ortho_image_minimum(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat ortho_image_mean(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat ortho_image_idw(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255, 255, 255));
    double height_max = max_height;
    double height_min = min_height;
    cv::Mat height_map(height_north * d, width_east * d, cv::DataType<double>::type);
    size_t counter = 0;
    VLOG(100) << "to y: " << static_cast<size_t>(height_north * d);
    VLOG(100) << "to x: " << static_cast<size_t>(width_east * d);
    common::ProgressBar progress_bar(static_cast<size_t>(height_north * d));
    for (size_t y = 0u; y < static_cast<size_t>(height_north * d); ++y) {
      //VLOG(100) << "y = " << y;
      progress_bar.increment();
      for (size_t x = 0u; x < static_cast<size_t>(width_east * d); ++x) {
        //VLOG(10000) << "x = " << x;
        CHECK(x < width_east * d);
        CHECK(y < height_north * d);

        //VLOG(200) << "alculate the cell center.";
        const Eigen::Vector2d cell_center =
            top_left + Eigen::Vector2d(static_cast<double>(x), -static_cast<double>(y)) / d +
            Eigen::Vector2d(0.5, -0.5) / d;

        std::vector<std::pair<int, double > > indices_dists;
        nanoflann::RadiusResultSet<double, int>
            resultSet(FLAGS_DEM_interpolation_radius, indices_dists);
        const double query_pt[3] = {cell_center(0), cell_center(1), 0.0};
        index.findNeighbors(resultSet, query_pt, nanoflann::SearchParams());

        //std::cout << "resultSet.size() = "<< resultSet.size() << std::endl;
        if (adaptive_interpolation) {
          int lambda = 10;
          while (resultSet.size() == 0u) {
            nanoflann::RadiusResultSet<double, int> tmp(lambda * FLAGS_DEM_interpolation_radius,
                                                                            indices_dists);
  //            resultSet = tmp;
            index.findNeighbors(tmp, query_pt, nanoflann::SearchParams());
            lambda *= 10;
            //std::cout << "lambda=" << lambda << std::endl;
          }
        }

        bool samples_in_interpolation_radius = resultSet.size() > 0u;
        if (samples_in_interpolation_radius) {
          std::vector<double> distances;
          std::vector<double> heights;
          CHECK(resultSet.size() > 0);
          distances.clear();
          heights.clear();
          for (const std::pair<int, double>& s : resultSet.m_indices_dists) {
            distances.push_back(s.second);
            heights.push_back(cloud_kdtree.pts[s.first].z);
          }
          CHECK(distances.size() > 0u);
          CHECK(heights.size() > 0u);
          CHECK(distances.size() == heights.size());

          double maximum_height = std::numeric_limits<double>::min();
          double minimum_height = std::numeric_limits<double>::max();
          double summed_height = 0.0;
          double idw_top = 0.0;
          double idw_bottom = 0.0;
          bool idw_perfect_match = false;
          for (size_t i = 0u; i < heights.size(); ++i) {
            // Maximum height in kdtree radius.
            if (heights[i] > maximum_height) {
              maximum_height = heights[i];
            }
            // Minimum height in kdtree radius.
            if (heights[i] < minimum_height) {
              minimum_height = heights[i];
            }
            // Mean of all height in kdtree radius.
            summed_height += heights[i];

            // Inverse distance weighting.
            if (distances[i] == 0.0) {
              // Perfect match, no interpolation needed.
              idw_top = heights[i];
              idw_bottom = 1.0;
              idw_perfect_match = true;
            }
            if (!idw_perfect_match) {
              CHECK(distances[i] > 0.0);
              idw_top += heights[i] / (distances[i]);
              idw_bottom += 1.0 / (distances[i]);
            }
          }

          // Maximum height
          CHECK(height_max - height_min != 0.0);
          const double maximum_scaled_height =
              (255.0) / (height_max - height_min) * maximum_height - (255.0) / (height_max - height_min) * height_min;
          ortho_image_maximum.at<cv::Vec3b>(y, x) =
              cv::Vec3b(pal.colors[int(maximum_scaled_height)].rgbBlue,
              pal.colors[int(maximum_scaled_height)].rgbGreen,
              pal.colors[int(maximum_scaled_height)].rgbRed);

          // Minimum height
          const double minimum_scaled_height =
              (255.0) / (height_max - height_min) * minimum_height - (255.0) / (height_max-height_min) * height_min;
          ortho_image_minimum.at<cv::Vec3b>(y, x) =
              cv::Vec3b(pal.colors[int(minimum_scaled_height)].rgbBlue,
              pal.colors[int(minimum_scaled_height)].rgbGreen,
              pal.colors[int(minimum_scaled_height)].rgbRed);

          // Mean height
          CHECK(heights.size() != 0);
          const double mean_height = summed_height / double(heights.size());
          const double mean_scaled_height =
              (255.0) / (height_max - height_min) * mean_height - (255.0) / (height_max - height_min) * height_min;
          ortho_image_mean.at<cv::Vec3b>(y, x) =
              cv::Vec3b(pal.colors[int(mean_scaled_height)].rgbBlue,
              pal.colors[int(mean_scaled_height)].rgbGreen,
              pal.colors[int(mean_scaled_height)].rgbRed);

          // Inverse distance weighting.
          CHECK(idw_bottom > 0.0);
          const double idw_height = idw_top / idw_bottom;
          const double idw_scaled_height =
              (255.0) / (height_max - height_min) * idw_height - (255.0) / (height_max-height_min) * height_min;
          ortho_image_idw.at<cv::Vec3b>(y, x) =
              cv::Vec3b(pal.colors[int(idw_scaled_height)].rgbBlue,
              pal.colors[int(idw_scaled_height)].rgbGreen,
              pal.colors[int(idw_scaled_height)].rgbRed);

          height_map.at<double>(y, x) = mean_height;

        } else {
          ++counter;
          ortho_image_maximum.at<cv::Vec3b>(y, x) = ortho_image_minimum.at<cv::Vec3b>(y, x) =
              ortho_image_mean.at<cv::Vec3b>(y, x) = ortho_image_idw.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
          height_map.at<double>(y, x) = 1.0;
           //  std::cout << "y/x: " << y << "/" << x << ", mean=" << -1 << std::endl;
        }
      }
    }
    std::cout << "counter= " << counter << std::endl;
    if (FLAGS_DEM_show_output) {
      cv::imshow("DEM maximum", ortho_image_maximum);
      cv::imshow("DEM minimum", ortho_image_minimum);
      cv::imshow("DEM mean", ortho_image_mean);
      cv::imshow("DEM idw", ortho_image_idw);
      cv::waitKey(0);
    }

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

void FwOnlineDigitalElevationMap::writeDataToDEMGeoTiffColor(
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
