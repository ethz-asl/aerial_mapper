#include "fw_ortho_from_pcl/fw-digital-elevation-map.h"

#include <memory>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

DEFINE_string(DEM_filename_xyz, "/media/timo/scnd/experiments/dem_test/cropped.xyz", "filename pointcloud");
DEFINE_string(DEM_output_folder, "/tmp/", "Result output folder.");
DEFINE_double(DEM_interpolation_radius, 1.0, "Radius of the interpolation (KD-tree)");
DEFINE_int32(DEM_color_palette, 6, "Color palette enum");
DEFINE_double(DEM_resolution, 2, "Resolution of the Digital elevation map.");
DEFINE_int32(DEM_UTM_code, 33, "UTM code used for the geotiff.");
DEFINE_bool(DEM_show_output, true, "Show the output as opencv image?");
DEFINE_bool(DEM_save_cv_mat_height_map, true, "");
DEFINE_double(DEM_easting_min, 464899.00, "");
DEFINE_double(DEM_northing_min, 5272160.00, "");
DEFINE_double(DEM_easting_max, 465056.47, "");
DEFINE_double(DEM_northing_max, 5272349.53, "");

std::unique_ptr<FwOnlineDigitalElevationMap> dem_;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "online_digital_elevation_map_node");
  ros::NodeHandle nh;

  //const Eigen::Vector3d origin(464980, 5.27226e+06, 414.087);
  //const Eigen::Vector3d origin(200.0, 200.0, 0.0);



  std::string file = "/tmp/pointcloud.txt";
  std::cout << "OPEN: " << file << std::endl;
  std::ifstream infile(file);
  //int64_t ts;
  double x,y,z;
  int intensity;
  Aligned<std::vector, Eigen::Vector3d>::type xyz;
  std::vector<int> intensities;
  double max_x = -500000;
  double max_y = -500000;
  double max_z = -500000;
  double min_x = 500000;
  double min_y = 500000;
  double min_z = 500000;
  while( infile >> x >> y >> z >> intensity) {

    if (z > -100) {
    xyz.push_back(Eigen::Vector3d(x,y,z));
    if (xyz.size () < 20) {
    std::cout << "x y z i : "
              << x << " "
              << y << " "
              << z << " "
              << double(intensity) << std::endl;
    }
    if (x > max_x) {max_x = x;}
    if (y > max_y) {max_y = y;}
    if (z > max_z) {max_z = z;}
    if (x < min_x) {min_x = x;}
    if (y < min_y) {min_y = y;}
    if (z < min_z) {min_z = z;}

    intensities.push_back(intensity);
    }
    if( infile.eof() ) break;
  }
  std::cout << std::setprecision(10) << max_x << " " << max_y << " " << max_z << std::endl;
  std::cout << std::setprecision(10) << min_x << " " << min_y << " " << min_z << std::endl;
  CHECK(xyz.size() == intensities.size());
  std::cout << "Pointcloud size = " << xyz.size() << std::endl;
  CHECK(xyz.size() > 0);

  dem_.reset(new FwOnlineDigitalElevationMap(xyz, intensities));

  //ros::spin();

  return 0;
}
