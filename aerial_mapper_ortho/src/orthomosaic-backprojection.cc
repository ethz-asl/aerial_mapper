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

/* signum function */
int sgn(int x){
  return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}

void gbham(int xstart,int ystart,int xend,int yend)
/*--------------------------------------------------------------
 * Bresenham-Algorithmus: Linien auf Rastergeräten zeichnen
 *
 * Eingabeparameter:
 *    int xstart, ystart        = Koordinaten des Startpunkts
 *    int xend, yend            = Koordinaten des Endpunkts
 *
 * Ausgabe:
 *    void SetPixel(int x, int y) setze ein Pixel in der Grafik
 *         (wird in dieser oder aehnlicher Form vorausgesetzt)
 *---------------------------------------------------------------
 */
{
    int x, y, t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, es, el, err;

/* Entfernung in beiden Dimensionen berechnen */
   dx = xend - xstart;
   dy = yend - ystart;

/* Vorzeichen des Inkrements bestimmen */
   incx = sgn(dx);
   incy = sgn(dy);
   if(dx<0) dx = -dx;
   if(dy<0) dy = -dy;

/* feststellen, welche Entfernung größer ist */
   if (dx>dy)
   {
      /* x ist schnelle Richtung */
      pdx=incx; pdy=0;    /* pd. ist Parallelschritt */
      ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
      es =dy;   el =dx;   /* Fehlerschritte schnell, langsam */
   } else
   {
      /* y ist schnelle Richtung */
      pdx=0;    pdy=incy; /* pd. ist Parallelschritt */
      ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
      es =dx;   el =dy;   /* Fehlerschritte schnell, langsam */
   }

/* Initialisierungen vor Schleifenbeginn */
   x = xstart;
   y = ystart;
   err = el/2;
   std::cout << "x,y " << x << ", " << y << std::endl;

/* Pixel berechnen */
   for(t=0; t<el; ++t) /* t zaehlt die Pixel, el ist auch Anzahl */
   {
      /* Aktualisierung Fehlerterm */
      err -= es;
      if(err<0)
      {
          /* Fehlerterm wieder positiv (>=0) machen */
          err += el;
          /* Schritt in langsame Richtung, Diagonalschritt */
          x += ddx;
          y += ddy;
      } else
      {
          /* Schritt in schnelle Richtung, Parallelschritt */
          x += pdx;
          y += pdy;
      }
      std::cout << "x,y " << x << ", " << y << std::endl;
   }
} /* gbham() */


OrthomosaicBackprojection::OrthomosaicBackprojection() {
  aslam::TransformationVector T_G_Cs;
  std::vector<cv::Mat> images;
  loadPosesAndImagesFromPix4d(FLAGS_orthomosaic_filename_poses,
                              FLAGS_orthomosaic_image_directory,
                              FLAGS_orthomosaic_camera_calibration_yaml,
                              &T_G_Cs, &images);

//  for (const cv::Mat& image : images) {
//    cv::imshow("tmp", image);
//    cv::waitKey(1);
//  }
//  VLOG(1) << "DONE";

  process(T_G_Cs, images);

}

OrthomosaicBackprojection::OrthomosaicBackprojection(std::string ncameras_yaml_path_filename,
                                                     const aslam::TransformationVector& T_G_Bs,
                                                     const std::vector<cv::Mat>& images) {
  loadCameraRig(ncameras_yaml_path_filename);
  CHECK(ncameras_);
  aslam::TransformationVector T_G_Cs;
//  size_t k = 0;
  for (const aslam::Transformation& T_G_B : T_G_Bs) {
//    ++k;
//    if ( k == 150) {
    T_G_Cs.push_back(T_G_B * ncameras_->get_T_C_B(0u).inverse());
//    }

  }
  std::vector<cv::Mat> images_undistorted;
  std::unique_ptr<aslam::MappedUndistorter> undistorter_;
  undistorter_ = aslam::createMappedUndistorter(ncameras_->getCameraShared(0), 1.0, 1.0,
                                                aslam::InterpolationMethod::Linear);

  std::cout << "sdf" << ncameras_->getCamera(0u).getParameters() << std::endl;
  std::vector<cv::Mat> images_new;
  //HECK(false);
  //k=0;
  for (const cv::Mat& image : images) {
//    ++k;
//    if ( k == 150) {
    cv::Mat image_undistorted;
    undistorter_->processImage(image, &image_undistorted);
    images_undistorted.push_back(image_undistorted);
    images_new.push_back(image);
//    cv::imwrite("/tmp/test_undistorted.jpg", image_undistorted);

//    cv::imshow("250", image);
//    cv::waitKey(0);
    //break;
//     }
  }
  process(T_G_Cs, images_new);
}

void OrthomosaicBackprojection::loadCameraRig(std::string ncameras_yaml_path_filename) {
  ncameras_ = aslam::NCamera::loadFromYaml(ncameras_yaml_path_filename);
  CHECK(ncameras_) << "Could not load the camera calibration from: "
                   << ncameras_yaml_path_filename;
}

void OrthomosaicBackprojection::loadPosesAndImagesFromViMap(std::string path_vi_map,
                                                            aslam::TransformationVector* T_G_Cs,
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
  ncameras_ = map->getVertex(all_vertices[0]).getVisualNFrame().getNCameraShared();
  const aslam::Transformation& T_B_C = ncameras_->get_T_C_B(kFrameIdx).inverse();

  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    cv::Mat image_raw;
    const viwls_graph::Vertex& vertex = map->getVertex(vertex_id);
    map->getCvMatResource(vertex, kFrameIdx, viwls_graph::ResourceType::kRawImage, &image_raw);
    T_G_Cs->emplace_back(vertex.get_T_M_I() * T_B_C);
    images->emplace_back(image_raw);
    CHECK(T_G_Cs->size() == images->size());
  }
  VLOG(1) << "Extracted " << T_G_Cs->size() << " poses.";
}

void OrthomosaicBackprojection::loadPosesAndImagesFromPix4d(
    std::string filename_poses, std::string filename_images,
    std::string ncameras_yaml_path_filename, aslam::TransformationVector* T_G_Cs,
    std::vector<cv::Mat>* images) {
  CHECK_NOTNULL(T_G_Cs);
  CHECK_NOTNULL(images);

  std::vector<std::string> image_names;
  loadPosesFromPix4d(filename_poses, T_G_Cs, &image_names);
  CHECK(T_G_Cs->size() == image_names.size());
  loadImages(filename_images, image_names, images);
  CHECK(T_G_Cs->size() == images->size());
  initializeCameraRig(ncameras_yaml_path_filename);
}

void OrthomosaicBackprojection::loadImages(const std::string& image_directory,
                                           const std::vector<std::string>& image_names,
                                           std::vector<cv::Mat>* images) {
  CHECK_NOTNULL(images);
  CHECK(image_names.size());
  images->clear();
  for (std::string image_name : image_names) {
    VLOG(100) << "Loading image: " << image_directory + image_name;
    cv::Mat image = cv::imread(image_directory + image_name, CV_LOAD_IMAGE_GRAYSCALE);
    //cv::Mat img_tmp;
    //cv::equalizeHist(image, img_tmp);
    images->push_back(image);
  //  images->push_back(image);
    cv::imshow("last_image", image);
    cv::waitKey(1);
  }
  CHECK(images->size() == image_names.size());
}

void OrthomosaicBackprojection::loadPosesFromPix4d(std::string filename_poses,
                                                   aslam::TransformationVector* poses,
                                                   std::vector<std::string>* image_names) {
  VLOG(3) << "Loading poses from file: " << filename_poses;
  std::ifstream infile(filename_poses);
  std::string header;
  std::getline(infile, header);
  VLOG(3) << "Header: " <<  header;

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

      R << cp * ck, cw * sk + sw*sp*ck, sw*sk-cw*sp*ck,
          -cp*sk, cw*ck - sw*sp*sk, sw*ck+cw*sp*sk,
          sp, -sw*cp, cw*cp;

      double northing, easting;
      char utm_zone[10];
      UTM::LLtoUTM(latitude, longitude, northing, easting, utm_zone);
      const Eigen::Vector3d t (easting, northing, altitude);

      Eigen::Matrix4d T_;
      T_.block<3, 3>(0, 0) = (R_match_conventions * R).transpose();
      T_.block<3, 1>(0, 3) = t;
      aslam::Transformation T (T_);
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

void OrthomosaicBackprojection::process(const aslam::TransformationVector& T_G_Cs,
                                        const std::vector<cv::Mat>& images) {
  const int nameWidth = 30;
  std::cout << "**********************************************************************************************" << std::endl
            << "Starting Orthomosaic image generation" << std::endl
            << std::left << std::setw(nameWidth) << " - Filename poses: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_filename_poses << std::endl
            << std::left << std::setw(nameWidth) << " - Image directory: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_image_directory << std::endl
            << std::left << std::setw(nameWidth) << " - Filename camera calibration: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_camera_calibration_yaml << std::endl
            << std::left << std::setw(nameWidth) << " - Resolution: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_resolution << std::endl
            << std::left << std::setw(nameWidth) << " - Easting min.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_easting_min << std::endl
            << std::left << std::setw(nameWidth) << " - Easting max.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_easting_max << std::endl
            << std::left << std::setw(nameWidth) << " - Northing min.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_northing_min << std::endl
            << std::left << std::setw(nameWidth) << " - Northing max.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_northing_max << std::endl;
  std::cout << "**********************************************************************************************" << std::endl;

  CHECK(!T_G_Cs.empty());
  CHECK(T_G_Cs.size() == images.size());
  const aslam::Camera& camera = ncameras_->getCamera(kFrameIdx);

  //cv::Mat height_map;
  //loadHeightMap(FLAGS_orthomosaic_filename_height_map, &height_map);

  // Define the grid.
  const Eigen::Vector2d bottom_left = Eigen::Vector2d(FLAGS_orthomosaic_easting_min,
                                                      FLAGS_orthomosaic_northing_min);
  const Eigen::Vector2d top_right = Eigen::Vector2d(FLAGS_orthomosaic_easting_max,
                                                    FLAGS_orthomosaic_northing_max);
  const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
  const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
  const Eigen::Vector2d top_left = bottom_left + Eigen::Vector2d(0.0, height_north);

  std::cout << "images.size() = " << images.size() << std::endl;
  std::cout << "T_G_Cs.size() = " << T_G_Cs.size() << std::endl;

  // bool use_digital_elevation_map = false;
  // Iterate over all cells.
  const double d = static_cast<double>(FLAGS_orthomosaic_resolution);
  const Eigen::Vector2d& a = Eigen::Vector2d(0.5, -0.5) / d;
  const Eigen::Vector2d& b = a + top_left;
  cv::Mat orthomosaic(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255.0, 255.0, 255.0));
  cv::Mat orthomosaic2(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255.0, 255.0, 255.0));
  Eigen::MatrixXi observation_map
      = Eigen::MatrixXi::Zero(static_cast<int>(width_east * d),
                              static_cast<int>(height_north * d));
  std::cout << "observation_map.rows()=" << observation_map.rows() << std::endl;
  std::cout << "observation_map.cols()=" << observation_map.cols() << std::endl;

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
      //VLOG(1) << "[ " << x << " / " << width_east * d << " ]";
      CHECK(observation_map.rows() > x);
      const Eigen::Vector2d& cell_center =
          b + Eigen::Vector2d(static_cast<double>(x), -static_cast<double>(y)) / d;
      //      if (use_digital_elevation_map) {
      //        const double height_from_dem = height_map.at<double>(y, x);
      //        landmark_UTM = Eigen::Vector3d(cell_center(0), cell_center(1), height_from_dem);
      //      } else {
      const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1), 414.087);
      //std::cout << "landmark_UTM = " << landmark_UTM.transpose() << std::endl;
      //VLOG(1) << "landmark_UTM = " << landmark_UTM.transpose();
      //      }
      //std::vector<std::pair<double, double > > score;

      for (size_t i = 0u; i < images.size(); ++i) {
        const Eigen::Vector3d& C_landmark = T_G_Cs[i].inverse().transform(landmark_UTM);
        Eigen::Vector2d keypoint;
        const aslam::ProjectionResult& projection_result = camera.project3(C_landmark, &keypoint);

        // Check if keypoint visible.
        const bool keypoint_visible =
            (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
            (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
            (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
            (projection_result.getDetailedStatus() != aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
            (projection_result.getDetailedStatus() != aslam::ProjectionResult::PROJECTION_INVALID);
        //VLOG(1) << "keypoint_visible = " << keypoint_visible;
        if (keypoint_visible) {
//          observation_map(x,y) +=1;
          // SIC! Order of keypoint x/y.
          //double gray_value;
          //std::cout << "keypoint = " << keypoint.transpose() << std::endl;
          const int kp_y = std::min(static_cast<int>(std::round(keypoint(1))), 479);
          const int kp_x = std::min(static_cast<int>(std::round(keypoint(0))), 751);
          //VLOG(1) << "(kp_x/kp_y) = (" << kp_x << "/" << kp_y << ")";
          //std::cout << "i = " << i << std::endl;
          //cv::imshow("current image", images[i]);
          //cv::waitKey(1);
          const double gray_value = images[i].at<uchar>(kp_y, kp_x);
          //VLOG(1) << "gray_value = " << gray_value;
          orthomosaic.at<cv::Vec3b>(y, x) =
              cv::Vec3b(gray_value, gray_value, gray_value);
//          double dist = //(T_G_Cs[i].getPosition() - landmark_UTM).norm();
//          double dist = (Eigen::Vector2d(470.0/2.0, 752.0/2.0)-keypoint).norm();
//          //std::cout << "dist = " << dist << std::endl;
//          std::pair<double, double> a = std::make_pair(dist, gray_value);
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
    cv::imshow("Orthomosaic", orthomosaic);/*
    //cv::imshow("Orthomosaic2", orthomosaic2);*/
    cv::waitKey(1);
  }

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& d1 = time2 - time1;
  std::cout << d1 << std::endl;

  cv::imshow("Orthomosaic", orthomosaic);
  cv::waitKey(0);

  cv::imwrite("/tmp/orthomosaic_no_dem.jpeg", orthomosaic);/*
  cv::imwrite("/tmp/ortho_score.jpeg", orthomosaic2);*/

  bool create_geotiff = true;
  if (create_geotiff) {
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

    std::string geotiff_filename =
        "/tmp/test_geo3.tiff";
    GDALDataset* poDstDS;
    char **papszOptions = NULL;

    int height = orthomosaic.rows;
    int width = orthomosaic.cols;
    std::cout << "height = " << height << std::endl;
    std::cout << "width = " << width << std::endl;
    poDstDS = poDriver->Create(geotiff_filename.c_str(), width, height, 1, GDT_Byte, papszOptions );

    double adfGeoTransform[6] = {464499.00, 1.0, 0.0, 5272750.00, 0.0, -1.0};//{ 444720, 30, 0, 3751320, 0, -30 };
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
        std::cout << "x = " << x << ", y = " << y << ", access. y*width+x = " << y*width+x << std::endl;
        pdata[x + y * width]= orthomosaic.at<cv::Vec3b>(y,x)[0];
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
}

// Checks whether the given keypoint lies inside the image box minus some border.
bool OrthomosaicBackprojection::withinImageBoxWithBorder(
    Eigen::Vector2d keypoint, const aslam::Camera& camera) {
  static constexpr double kMinDistanceToBorderPx = 0.0;
  CHECK_LT(2 * kMinDistanceToBorderPx, camera.imageWidth());
  CHECK_LT(2 * kMinDistanceToBorderPx, camera.imageHeight());
  return (keypoint(0) >= kMinDistanceToBorderPx) &&
      (keypoint(1) >= kMinDistanceToBorderPx) &&
      (keypoint(0) < static_cast<double>(camera.imageWidth()) - kMinDistanceToBorderPx) &&
      (keypoint(1) < static_cast<double>(camera.imageHeight()) - kMinDistanceToBorderPx);
}

void  OrthomosaicBackprojection::Bresenham(int x1,
    int y1,
    int const x2,
    int const y2)
{
#ifdef adsf
    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;

    plot(x1, y1);

    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2)
        {
            if ((error >= 0) && (error || (ix > 0)))
            {
                error -= delta_x;
                y1 += iy;
            }
            // else do nothing

            error += delta_y;
            x1 += ix;

            plot(x1, y1);
        }
    }
    else
    {
        // error may go below zero
        int error(delta_x - (delta_y >> 1));

        while (y1 != y2)
        {
            if ((error >= 0) && (error || (iy > 0)))
            {
                error -= delta_y;
                x1 += ix;
            }
            // else do nothing

            error += delta_x;
            y1 += iy;

            plot(x1, y1);
        }
    }
#endif
}

void OrthomosaicBackprojection::initializeCameraRig(
    const std::string& ncameras_yaml_path_filename) {
  ncameras_ = aslam::NCamera::loadFromYaml(ncameras_yaml_path_filename);
  CHECK(ncameras_) << "Could not load the camera calibration from: "
       << ncameras_yaml_path_filename;
}

void OrthomosaicBackprojection::processIncremental(const aslam::TransformationVector& T_G_Cs,
                                                   const std::vector<cv::Mat>& images) {

  const int nameWidth = 30;
  std::cout << "**********************************************************************************************" << std::endl
            << "Starting INCREMENTAL Orthomosaic image generation" << std::endl
            << std::left << std::setw(nameWidth) << " - Filename poses: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_filename_poses << std::endl
            << std::left << std::setw(nameWidth) << " - Image directory: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_image_directory << std::endl
            << std::left << std::setw(nameWidth) << " - Filename camera calibration: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_camera_calibration_yaml << std::endl
            << std::left << std::setw(nameWidth) << " - Resolution: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_resolution << std::endl
            << std::left << std::setw(nameWidth) << " - Easting min.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_easting_min << std::endl
            << std::left << std::setw(nameWidth) << " - Easting max.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_easting_max << std::endl
            << std::left << std::setw(nameWidth) << " - Northing min.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_northing_min << std::endl
            << std::left << std::setw(nameWidth) << " - Northing max.: "
            << std::left << std::setw(nameWidth) << FLAGS_orthomosaic_northing_max << std::endl;
  std::cout << "**********************************************************************************************" << std::endl;

  CHECK(!T_G_Cs.empty());
  CHECK(T_G_Cs.size() == images.size());
  const aslam::Camera& camera = ncameras_->getCamera(kFrameIdx);

  // Define the grid.
  const Eigen::Vector2d bottom_left = Eigen::Vector2d(FLAGS_orthomosaic_easting_min,
                                                      FLAGS_orthomosaic_northing_min);
  const Eigen::Vector2d top_right = Eigen::Vector2d(FLAGS_orthomosaic_easting_max,
                                                    FLAGS_orthomosaic_northing_max);
  const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
  const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
  const Eigen::Vector2d top_left = bottom_left + Eigen::Vector2d(0.0, height_north);

  std::cout << "images.size() = " << images.size() << std::endl;
  std::cout << "T_G_Cs.size() = " << T_G_Cs.size() << std::endl;

  // bool use_digital_elevation_map = false;
  // Iterate over all cells.
  const double d = static_cast<double>(FLAGS_orthomosaic_resolution);
  const Eigen::Vector2d& a = Eigen::Vector2d(0.5, -0.5) / d;
  const Eigen::Vector2d& b = a + top_left;
  cv::Mat orthomosaic(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255.0, 255.0, 255.0));
  cv::Mat orthomosaic2(height_north * d, width_east * d, CV_8UC3, cv::Scalar(255.0, 255.0, 255.0));
  Eigen::MatrixXi observation_map
      = Eigen::MatrixXi::Zero(static_cast<int>(width_east * d),
                              static_cast<int>(height_north * d));
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
  border_keypoints_.col(1) = Eigen::Vector2d(static_cast<double>(width - 1u), 0.0);
  border_keypoints_.col(2) = Eigen::Vector2d(static_cast<double>(width - 1u),
                                             static_cast<double>(height - 1u));
  border_keypoints_.col(3) = Eigen::Vector2d(0.0, static_cast<double>(height - 1u));

  const ros::Time time1 = ros::Time::now();
  for (size_t i =0u; i < images.size(); ++i) {
//    const cv::Mat& image = images[i];
    const aslam::Transformation& T_G_C = T_G_Cs[i];
    std::vector<cv::Point2f> ground_points, image_points;
    for (int border_pixel_index = 0; border_pixel_index < border_keypoints_.cols();
         ++border_pixel_index) {
      Eigen::Vector3d C_ray;
      const Eigen::Vector2d& keypoint = border_keypoints_.col(border_pixel_index);
      ncameras_->getCameraShared(kFrameIdx)->backProject3(keypoint, &C_ray);
      const double scale = - (T_G_C.getPosition()(2)-414.087) / (T_G_C.getRotationMatrix() * C_ray)(2);
      const Eigen::Vector3d& G_landmark =
          T_G_C.getPosition() + scale * T_G_C.getRotationMatrix() * C_ray;
      ground_points.push_back(cv::Point2f(G_landmark(0), G_landmark(1)));
      image_points.push_back(cv::Point2f(border_keypoints_.col(border_pixel_index)(0),
                                         border_keypoints_.col(border_pixel_index)(1)));

    }

    double x_max, x_min, y_max, y_min;
    x_max = 0;
    y_max = 0;
    x_min = 1.0e16;
    y_min = 1.0e16;
    for (const cv::Point2f& ground_point : ground_points) {/*
      std::cout << std::setprecision(15) << ground_point.x << "," << ground_point.y << std::endl;*/
      if (ground_point.x > x_max) {x_max = ground_point.x;}
      if (ground_point.x < x_min) {x_min = ground_point.x;}
      if (ground_point.y > y_max) {y_max = ground_point.y;}
      if (ground_point.y < y_min) {y_min = ground_point.y;}
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
    x_max_ = int(x_max)-b(0);
    x_min_ = int(x_min)-b(0);
    y_max_ = -(int(y_max)-b(1));
    y_min_ = -(int(y_min)-b(1));
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
//                  << std::to_string(orthomosaic.at<cv::Vec3b>(y, x)[0]) << std::endl;
        if (orthomosaic.at<cv::Vec3b>(y, x)[0] == 255 ||
            orthomosaic.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 255, 0)) {
          //std::cout << "x/y: " << x << "," << y << std::endl;
          const Eigen::Vector2d& cell_center =
              b + Eigen::Vector2d(static_cast<double>(x), -static_cast<double>(y)) / d;
          const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1), 414.087);
          const Eigen::Vector3d& C_landmark = T_G_Cs[i].inverse().transform(landmark_UTM);
          Eigen::Vector2d keypoint;
          const aslam::ProjectionResult& projection_result = camera.project3(C_landmark, &keypoint);
          const bool keypoint_visible =
              (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
              (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
              (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
              (projection_result.getDetailedStatus() != aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
              (projection_result.getDetailedStatus() != aslam::ProjectionResult::PROJECTION_INVALID);
          if (keypoint_visible) {
            const int kp_y = std::min(static_cast<int>(std::round(keypoint(1))), 479);
            const int kp_x = std::min(static_cast<int>(std::round(keypoint(0))), 751);
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
  } // images

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
          b + Eigen::Vector2d(static_cast<double>(x), -static_cast<double>(y)) / d;
      const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1), 414.087);
      //for (size_t i = 0u; i < images.size(); ++i) {
        size_t i =0u;
        const Eigen::Vector3d& C_landmark = T_G_Cs[i].inverse().transform(landmark_UTM);
        Eigen::Vector2d keypoint;
        const aslam::ProjectionResult& projection_result = camera.project3(C_landmark, &keypoint);
        const bool keypoint_visible =
            (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
            (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
            (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
            (projection_result.getDetailedStatus() != aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
            (projection_result.getDetailedStatus() != aslam::ProjectionResult::PROJECTION_INVALID);
        if (keypoint_visible) {
          std::cout << "cell_center: " << cell_center.transpose() << std::endl;
          const int kp_y = std::min(static_cast<int>(std::round(keypoint(1))), 479);
          const int kp_x = std::min(static_cast<int>(std::round(keypoint(0))), 751);
          const double gray_value = images[i].at<uchar>(kp_y, kp_x);
          orthomosaic.at<cv::Vec3b>(y, x) =
              cv::Vec3b(gray_value, gray_value, gray_value);
          //break;
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

