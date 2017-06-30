#include "fw-ortho/orthomosaic-backprojection.h"

#include "fw-ortho/gps-conversions.h"

#include <gdal/gdal.h>
#include <gdal/cpl_string.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>

#include <aslam/pipeline/undistorter-mapped.h>
#include <aslam/pipeline/undistorter.h>

#include <cstdlib>

DECLARE_string(orthomosaic_filename_poses);
DECLARE_string(orthomosaic_image_directory);
DECLARE_double(orthomosaic_easting_min);
DECLARE_double(orthomosaic_northing_min);
DECLARE_double(orthomosaic_easting_max);
DECLARE_double(orthomosaic_northing_max);
DECLARE_int32(orthomosaic_UTM_code);
DECLARE_double(orthomosaic_resolution);
DECLARE_string(orthomosaic_camera_calibration_yaml);
DECLARE_string(orthomosaic_filename_height_map);

OrthomosaicBackprojection::OrthomosaicBackprojection() {
  aslam::TransformationVector T_G_Cs;
  std::vector<cv::Mat> images;
  loadPosesAndImagesFromPix4d(
      FLAGS_orthomosaic_filename_poses,
        FLAGS_orthomosaic_image_directory,
      FLAGS_orthomosaic_camera_calibration_yaml, &T_G_Cs, &images);
  process(T_G_Cs, images);
}

OrthomosaicBackprojection::OrthomosaicBackprojection(
    std::string ncameras_yaml_path_filename,
    const aslam::TransformationVector& T_G_Bs,
    const std::vector<cv::Mat>& images) {
  loadCameraRig(ncameras_yaml_path_filename);
  CHECK(ncameras_);
  aslam::TransformationVector T_G_Cs;
  for (const aslam::Transformation& T_G_B : T_G_Bs) {
    T_G_Cs.push_back(T_G_B * ncameras_->get_T_C_B(0u).inverse());
  }
  std::vector<cv::Mat> images_undistorted;
  std::unique_ptr<aslam::MappedUndistorter> undistorter_;
  undistorter_ =
      aslam::createMappedUndistorter(ncameras_->getCameraShared(0), 1.0, 1.0,
                                     aslam::InterpolationMethod::Linear);

  std::vector<cv::Mat> images_new;
  for (const cv::Mat& image : images) {
    cv::Mat image_undistorted;
    undistorter_->processImage(image, &image_undistorted);
    images_undistorted.push_back(image_undistorted);
    images_new.push_back(image);
  }
  processBatch(T_G_Cs, images_new);
}

void OrthomosaicBackprojection::processBatch(
    const aslam::TransformationVector& T_G_Cs,
    const std::vector<cv::Mat>& images) {
  CHECK(!T_G_Cs.empty());
  CHECK(T_G_Cs.size() == images.size());

  printParams1();
  const aslam::Camera& camera = ncameras_->getCamera(kFrameIdx);

  // cv::Mat height_map;
  // loadHeightMap(FLAGS_orthomosaic_filename_height_map, &height_map);

  // Define the grid.
  const Eigen::Vector2d bottom_left = Eigen::Vector2d(
      FLAGS_orthomosaic_easting_min, FLAGS_orthomosaic_northing_min);
  const Eigen::Vector2d top_right = Eigen::Vector2d(
      FLAGS_orthomosaic_easting_max, FLAGS_orthomosaic_northing_max);
  const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
  const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
  const Eigen::Vector2d top_left =
      bottom_left + Eigen::Vector2d(0.0, height_north);

  std::cout << "images.size() = " << images.size() << std::endl;
  std::cout << "T_G_Cs.size() = " << T_G_Cs.size() << std::endl;

  // bool use_digital_elevation_map = false;
  // Iterate over all cells.
  const double d = static_cast<double>(FLAGS_orthomosaic_resolution);
  const Eigen::Vector2d& a = Eigen::Vector2d(0.5, -0.5) / d;
  const Eigen::Vector2d& b = a + top_left;
  cv::Mat orthomosaic(height_north * d, width_east * d, CV_8UC3,
                      cv::Scalar(255.0, 255.0, 255.0));
  cv::Mat orthomosaic2(height_north * d, width_east * d, CV_8UC3,
                       cv::Scalar(255.0, 255.0, 255.0));
  Eigen::MatrixXi observation_map = Eigen::MatrixXi::Zero(
      static_cast<int>(width_east * d), static_cast<int>(height_north * d));
  std::cout << "observation_map.rows()=" << observation_map.rows()
            << std::endl;
  std::cout << "observation_map.cols()=" << observation_map.cols()
            << std::endl;

  size_t y_limit = static_cast<int>(height_north * d) - 1;
  size_t x_limit = static_cast<int>(width_east * d) - 1;
  std::cout << "x_limit = " << x_limit << std::endl;
  std::cout << "y_limit =" << y_limit << std::endl;
  //  CHECK(height_map.size() == orthomosaic.size());
  //  std::cout << "height_map.size() = " << height_map.size() << std::endl;
  //  std::cout << "orthomosaic.size() = " << orthomosaic.size() << std::endl;
  const ros::Time time1 = ros::Time::now();

  for (size_t y = 0u; y < y_limit; ++y) {
    CHECK(observation_map.cols() > y);
    VLOG(1) << "[ " << y << " / " << height_north * d << " ]";
    for (size_t x = 0u; x < x_limit; ++x) {
      // VLOG(1) << "[ " << x << " / " << width_east * d << " ]";
      CHECK(observation_map.rows() > x);
      const Eigen::Vector2d& cell_center =
          b +
          Eigen::Vector2d(static_cast<double>(x),
                          -static_cast<double>(y)) / d;
      //      if (use_digital_elevation_map) {
      //        const double height_from_dem = height_map.at<double>(y, x);
      //        landmark_UTM = Eigen::Vector3d(cell_center(0), cell_center(1),
      //        height_from_dem);
      //      } else {
      const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1),
                                         414.087);
      // std::cout << "landmark_UTM = " << landmark_UTM.transpose() <<
      // std::endl;
      // VLOG(1) << "landmark_UTM = " << landmark_UTM.transpose();
      //      }
      // std::vector<std::pair<double, double > > score;

      for (size_t i = 0u; i < images.size(); ++i) {
        const Eigen::Vector3d& C_landmark =
            T_G_Cs[i].inverse().transform(landmark_UTM);
        Eigen::Vector2d keypoint;
        const aslam::ProjectionResult& projection_result =
            camera.project3(C_landmark, &keypoint);

        // Check if keypoint visible.
        const bool keypoint_visible =
            (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
            (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
            (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
            (projection_result.getDetailedStatus() !=
             aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
            (projection_result.getDetailedStatus() !=
             aslam::ProjectionResult::PROJECTION_INVALID);
        // VLOG(1) << "keypoint_visible = " << keypoint_visible;
        if (keypoint_visible) {
          //          observation_map(x,y) +=1;
          // SIC! Order of keypoint x/y.
          // double gray_value;
          // std::cout << "keypoint = " << keypoint.transpose() << std::endl;
          const int kp_y =
              std::min(static_cast<int>(std::round(keypoint(1))), 479);
          const int kp_x =
              std::min(static_cast<int>(std::round(keypoint(0))), 751);
          // VLOG(1) << "(kp_x/kp_y) = (" << kp_x << "/" << kp_y << ")";
          // std::cout << "i = " << i << std::endl;
          // cv::imshow("current image", images[i]);
          // cv::waitKey(1);
          const double gray_value = images[i].at<uchar>(kp_y, kp_x);
          // VLOG(1) << "gray_value = " << gray_value;
          orthomosaic.at<cv::Vec3b>(y, x) =
              cv::Vec3b(gray_value, gray_value, gray_value);
          //          double dist = //(T_G_Cs[i].getPosition() -
          //          landmark_UTM).norm();
          //          double dist = (Eigen::Vector2d(470.0/2.0,
          //          752.0/2.0)-keypoint).norm();
          //          //std::cout << "dist = " << dist << std::endl;
          //          std::pair<double, double> a = std::make_pair(dist,
          //          gray_value);
          //          score.push_back(a);
          break;
        }
      }
      //      double score_min = std::numeric_limits<double>::max();
      //      double gray_value_tmp = 0.0;
      //      for (size_t i = 0u; i < score.size(); ++i) {
      //        gray_value_tmp += score[i].second;
      //      }
      //      gray_value_tmp = gray_value_tmp/static_cast<double>(score.size());
      //      orthomosaic2.at<cv::Vec3b>(y, x) =
      //          cv::Vec3b(gray_value_tmp, gray_value_tmp, gray_value_tmp);
    }
    cv::imshow("Orthomosaic", orthomosaic); /*
     //cv::imshow("Orthomosaic2", orthomosaic2);*/
    cv::waitKey(1);
  }

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& d1 = time2 - time1;
  std::cout << d1 << std::endl;

  cv::imshow("Orthomosaic", orthomosaic);
  cv::waitKey(0);

  cv::imwrite("/tmp/orthomosaic_no_dem.jpeg", orthomosaic); /*
   cv::imwrite("/tmp/ortho_score.jpeg", orthomosaic2);*/

  bool create_geotiff = true;
  if (create_geotiff) {
    // Create a geotiff with GDAL.
    GDALAllRegister();

    //  const char *pszFormat = "GTiff";
    std::string name = "GTiff";
    GDALDriver* poDriver;
    char** papszMetadata;
    poDriver = GetGDALDriverManager()->GetDriverByName(name.c_str());
    //  if( poDriver == NULL )
    //    exit( 1 );
    papszMetadata = poDriver->GetMetadata();
    if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
      printf("Driver %s supports Create() method.\n", name.c_str());
    if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATECOPY, FALSE))
      printf("Driver %s supports CreateCopy() method.\n", name.c_str());

    std::string geotiff_filename = "/tmp/test_geo3.tiff";
    GDALDataset* poDstDS;
    char** papszOptions = NULL;

    int height = orthomosaic.rows;
    int width = orthomosaic.cols;
    std::cout << "height = " << height << std::endl;
    std::cout << "width = " << width << std::endl;
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
    //  oSRS.SetUTM( 11, TRUE );
    //  oSRS.SetWellKnownGeogCS( "NAD27" );

    oSRS.SetProjCS("UTM 32 (WGS84) in northern hemisphere.");
    oSRS.SetWellKnownGeogCS("WGS84");
    oSRS.SetUTM(32, TRUE);
    oSRS.exportToWkt(&pszSRS_WKT);
    poDstDS->SetProjection(pszSRS_WKT);

    CPLFree(pszSRS_WKT);
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
        std::cout << "x = " << x << ", y = " << y
                  << ", access. y*width+x = " << y* width + x << std::endl;
        pdata[x + y * width] = orthomosaic.at<cv::Vec3b>(y, x)[0];
      }
    }

    //  for (int n=0; n<width; n++) {
    //    for (int m=0; m<height; m++) {
    //      std::cout << "m = " << m << ", n = " << n << ", access. n*width+m =
    //      " << n*width+m << std::endl;

    //    }
    //  }

    poDstDS->GetRasterBand(1)->RasterIO(GF_Write, nx_offset, ny_offset, nx_size,
                                        ny_size, pdata, nBufxSize, nBufySize,
                                        GDT_Byte, 0, 0);
    /* Once we're done, close properly the dataset */
    GDALClose((GDALDatasetH)poDstDS);
    std::cout << "closing the dataset." << std::endl;
  }
}

// Checks whether the given keypoint lies inside the image box minus some
// border.
bool OrthomosaicBackprojection::withinImageBoxWithBorder(
    Eigen::Vector2d keypoint, const aslam::Camera& camera) {
  static constexpr double kMinDistanceToBorderPx = 0.0;
  CHECK_LT(2 * kMinDistanceToBorderPx, camera.imageWidth());
  CHECK_LT(2 * kMinDistanceToBorderPx, camera.imageHeight());
  return (keypoint(0) >= kMinDistanceToBorderPx) &&
         (keypoint(1) >= kMinDistanceToBorderPx) &&
         (keypoint(0) <
          static_cast<double>(camera.imageWidth()) - kMinDistanceToBorderPx) &&
         (keypoint(1) <
          static_cast<double>(camera.imageHeight()) - kMinDistanceToBorderPx);
}

void OrthomosaicBackprojection::processIncremental(
    const aslam::TransformationVector& T_G_Cs,
    const std::vector<cv::Mat>& images) {
  printParams2();

  CHECK(!T_G_Cs.empty());
  CHECK(T_G_Cs.size() == images.size());
  const aslam::Camera& camera = ncameras_->getCamera(kFrameIdx);

  // Define the grid.
  const Eigen::Vector2d bottom_left = Eigen::Vector2d(
      FLAGS_orthomosaic_easting_min, FLAGS_orthomosaic_northing_min);
  const Eigen::Vector2d top_right = Eigen::Vector2d(
      FLAGS_orthomosaic_easting_max, FLAGS_orthomosaic_northing_max);
  const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
  const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
  const Eigen::Vector2d top_left =
      bottom_left + Eigen::Vector2d(0.0, height_north);

  std::cout << "images.size() = " << images.size() << std::endl;
  std::cout << "T_G_Cs.size() = " << T_G_Cs.size() << std::endl;

  // bool use_digital_elevation_map = false;
  // Iterate over all cells.
  const double d = static_cast<double>(FLAGS_orthomosaic_resolution);
  const Eigen::Vector2d& a = Eigen::Vector2d(0.5, -0.5) / d;
  const Eigen::Vector2d& b = a + top_left;
  cv::Mat orthomosaic(height_north * d, width_east * d, CV_8UC3,
                      cv::Scalar(255.0, 255.0, 255.0));
  cv::Mat orthomosaic2(height_north * d, width_east * d, CV_8UC3,
                       cv::Scalar(255.0, 255.0, 255.0));
  Eigen::MatrixXi observation_map = Eigen::MatrixXi::Zero(
      static_cast<int>(width_east * d), static_cast<int>(height_north * d));
  std::cout << "observation_map.rows()=" << observation_map.rows() << std::endl;
  std::cout << "observation_map.cols()=" << observation_map.cols() << std::endl;

  //  size_t y_limit = static_cast<int>(height_north * d) - 1;
  //  size_t x_limit = static_cast<int>(width_east * d) - 1;
  //  std::cout << "x_limit = " << x_limit << std::endl;
  //  std::cout << "y_limit =" << y_limit << std::endl;

  // COMPUTE GROUND POINTS
  Eigen::Matrix2Xd border_keypoints_;
  border_keypoints_.resize(Eigen::NoChange, 4);
  const size_t width = ncameras_->getCameraShared(0u)->imageWidth();
  const size_t height = ncameras_->getCameraShared(0u)->imageHeight();
  border_keypoints_.col(0) = Eigen::Vector2d(0.0, 0.0);
  border_keypoints_.col(1) =
      Eigen::Vector2d(static_cast<double>(width - 1u), 0.0);
  border_keypoints_.col(2) = Eigen::Vector2d(static_cast<double>(width - 1u),
                                             static_cast<double>(height - 1u));
  border_keypoints_.col(3) =
      Eigen::Vector2d(0.0, static_cast<double>(height - 1u));

  const ros::Time time1 = ros::Time::now();
  for (size_t i = 0u; i < images.size(); ++i) {
    //    const cv::Mat& image = images[i];
    const aslam::Transformation& T_G_C = T_G_Cs[i];
    std::vector<cv::Point2f> ground_points, image_points;
    for (int border_pixel_index = 0;
         border_pixel_index < border_keypoints_.cols(); ++border_pixel_index) {
      Eigen::Vector3d C_ray;
      const Eigen::Vector2d& keypoint =
          border_keypoints_.col(border_pixel_index);
      ncameras_->getCameraShared(kFrameIdx)->backProject3(keypoint, &C_ray);
      const double scale = -(T_G_C.getPosition()(2) - 414.087) /
                           (T_G_C.getRotationMatrix() * C_ray)(2);
      const Eigen::Vector3d& G_landmark =
          T_G_C.getPosition() + scale * T_G_C.getRotationMatrix() * C_ray;
      ground_points.push_back(cv::Point2f(G_landmark(0), G_landmark(1)));
      image_points.push_back(
          cv::Point2f(border_keypoints_.col(border_pixel_index)(0),
                      border_keypoints_.col(border_pixel_index)(1)));
    }

    double x_max, x_min, y_max, y_min;
    x_max = 0;
    y_max = 0;
    x_min = 1.0e16;
    y_min = 1.0e16;
    for (const cv::Point2f& ground_point : ground_points) { /*
       std::cout << std::setprecision(15) << ground_point.x << "," <<
       ground_point.y << std::endl;*/
      if (ground_point.x > x_max) {
        x_max = ground_point.x;
      }
      if (ground_point.x < x_min) {
        x_min = ground_point.x;
      }
      if (ground_point.y > y_max) {
        y_max = ground_point.y;
      }
      if (ground_point.y < y_min) {
        y_min = ground_point.y;
      }
    }
    //  cv::imshow("FIRST IMAGE", image);
    //  cv::waitKey(0);

    // FIND CORRESPONDING GRID POINTS
    int x_max_, x_min_, y_max_, y_min_;
    //  std::cout << "b = " << b << std::endl;
    //  std::cout << "x_max " << x_max << std::endl;
    //  std::cout << "x_min " << x_min << std::endl;
    //  std::cout << "y_max " << y_max << std::endl;
    //  std::cout << "y_min " << y_min << std::endl;
    x_max_ = int(x_max) - b(0);
    x_min_ = int(x_min) - b(0);
    y_max_ = -(int(y_max) - b(1));
    y_min_ = -(int(y_min) - b(1));
    //  std::cout << "x_max_ " << x_max_ << std::endl;
    //  std::cout << "x_min_ " << x_min_ << std::endl;
    //  std::cout << "y_max_ " << y_max_ << std::endl;
    //  std::cout << "y_min_ " << y_min_ << std::endl;
    //    orthomosaic.at<cv::Vec3b>(y_max_, x_max_) = cv::Vec3b(255, 0, 0);
    //    orthomosaic.at<cv::Vec3b>(y_min_, x_min_) = cv::Vec3b(255, 255, 0);
    //    cv::imshow("Orthomosaic", orthomosaic);
    //    cv::waitKey(1);

    for (size_t x = x_min_; x <= x_max_; ++x) {
      for (size_t y = y_max_; y <= y_min_; ++y) {
        //        std::cout << "orthomosaic.at<cv::Vec3b>(y, x) "
        //                  << std::to_string(orthomosaic.at<cv::Vec3b>(y,
        //                  x)[0]) << std::endl;
        if (orthomosaic.at<cv::Vec3b>(y, x)[0] == 255 ||
            orthomosaic.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 255, 0)) {
          // std::cout << "x/y: " << x << "," << y << std::endl;
          const Eigen::Vector2d& cell_center =
              b +
              Eigen::Vector2d(static_cast<double>(x), -static_cast<double>(y)) /
                  d;
          const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1),
                                             414.087);
          const Eigen::Vector3d& C_landmark =
              T_G_Cs[i].inverse().transform(landmark_UTM);
          Eigen::Vector2d keypoint;
          const aslam::ProjectionResult& projection_result =
              camera.project3(C_landmark, &keypoint);
          const bool keypoint_visible =
              (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
              (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
              (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
              (projection_result.getDetailedStatus() !=
               aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
              (projection_result.getDetailedStatus() !=
               aslam::ProjectionResult::PROJECTION_INVALID);
          if (keypoint_visible) {
            const int kp_y =
                std::min(static_cast<int>(std::round(keypoint(1))), 479);
            const int kp_x =
                std::min(static_cast<int>(std::round(keypoint(0))), 751);
            const double gray_value = images[i].at<uchar>(kp_y, kp_x);
            orthomosaic.at<cv::Vec3b>(y, x) =
                cv::Vec3b(gray_value, gray_value, gray_value);
          } else {
            orthomosaic.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
          }
        }
      }
      //      cv::imshow("Orthomosaic", orthomosaic);
      //      cv::waitKey(1);
    }
  }  // images

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& d1 = time2 - time1;
  std::cout << d1 << std::endl;

#ifdef fadsf
  const ros::Time time1 = ros::Time::now();
  for (size_t y = 0u; y < y_limit; ++y) {
    CHECK(observation_map.cols() > y);
    for (size_t x = 0u; x < x_limit; ++x) {
      CHECK(observation_map.rows() > x);
      const Eigen::Vector2d& cell_center =
          b +
          Eigen::Vector2d(static_cast<double>(x), -static_cast<double>(y)) / d;
      const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1),
                                         414.087);
      // for (size_t i = 0u; i < images.size(); ++i) {
      size_t i = 0u;
      const Eigen::Vector3d& C_landmark =
          T_G_Cs[i].inverse().transform(landmark_UTM);
      Eigen::Vector2d keypoint;
      const aslam::ProjectionResult& projection_result =
          camera.project3(C_landmark, &keypoint);
      const bool keypoint_visible =
          (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
          (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
          (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
          (projection_result.getDetailedStatus() !=
           aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
          (projection_result.getDetailedStatus() !=
           aslam::ProjectionResult::PROJECTION_INVALID);
      if (keypoint_visible) {
        std::cout << "cell_center: " << cell_center.transpose() << std::endl;
        const int kp_y =
            std::min(static_cast<int>(std::round(keypoint(1))), 479);
        const int kp_x =
            std::min(static_cast<int>(std::round(keypoint(0))), 751);
        const double gray_value = images[i].at<uchar>(kp_y, kp_x);
        orthomosaic.at<cv::Vec3b>(y, x) =
            cv::Vec3b(gray_value, gray_value, gray_value);
        // break;
      }
      //}
    }
    cv::imshow("Orthomosaic", orthomosaic);
    cv::waitKey(1);
  }
#endif

  //  const ros::Time time2 = ros::Time::now();
  //  const ros::Duration& d1 = time2 - time1;
  //  std::cout << d1 << std::endl;

  cv::imshow("Orthomosaic", orthomosaic);
  cv::waitKey(0);

  cv::imwrite("/tmp/orthomosaic_incremental.jpeg", orthomosaic);
}

void printParams1() {
    const int nameWidth = 30;
  std::cout << "***************************************************************"
               "*******************************" << std::endl
            << "Starting Orthomosaic image generation" << std::endl << std::left
            << std::setw(nameWidth) << " - Filename poses: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_filename_poses
            << std::endl << std::left << std::setw(nameWidth)
            << " - Image directory: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_image_directory << std::endl << std::left
            << std::setw(nameWidth)
            << " - Filename camera calibration: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_camera_calibration_yaml
            << std::endl << std::left << std::setw(nameWidth)
            << " - Resolution: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_resolution << std::endl << std::left
            << std::setw(nameWidth) << " - Easting min.: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_easting_min
            << std::endl << std::left << std::setw(nameWidth)
            << " - Easting max.: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_easting_max << std::endl << std::left
            << std::setw(nameWidth) << " - Northing min.: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_northing_min
            << std::endl << std::left << std::setw(nameWidth)
            << " - Northing max.: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_northing_max << std::endl;
  std::cout << "***************************************************************"
               "*******************************" << std::endl;

}

void printParams2() {
  const int nameWidth = 30;
  std::cout << "***************************************************************"
               "*******************************" << std::endl
            << "Starting INCREMENTAL Orthomosaic image generation" << std::endl
            << std::left << std::setw(nameWidth)
            << " - Filename poses: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_filename_poses << std::endl << std::left
            << std::setw(nameWidth) << " - Image directory: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_image_directory
            << std::endl << std::left << std::setw(nameWidth)
            << " - Filename camera calibration: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_camera_calibration_yaml
            << std::endl << std::left << std::setw(nameWidth)
            << " - Resolution: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_resolution << std::endl << std::left
            << std::setw(nameWidth) << " - Easting min.: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_easting_min
            << std::endl << std::left << std::setw(nameWidth)
            << " - Easting max.: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_easting_max << std::endl << std::left
            << std::setw(nameWidth) << " - Northing min.: " << std::left
            << std::setw(nameWidth) << FLAGS_orthomosaic_northing_min
            << std::endl << std::left << std::setw(nameWidth)
            << " - Northing max.: " << std::left << std::setw(nameWidth)
            << FLAGS_orthomosaic_northing_max << std::endl;
  std::cout << "***************************************************************"
               "*******************************" << std::endl;
}
