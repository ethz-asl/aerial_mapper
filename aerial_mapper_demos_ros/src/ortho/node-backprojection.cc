#include "fw-ortho/orthomosaic-backprojection.h"



DEFINE_string(orthomosaic_filename_poses,
              "/run/user/1000/gvfs/smb-share:server=192.168.1.96,share=base/000000_exchange/reduced40/calibrated_external_camera_parameters_wgs84.txt",
              "Filename of the camera poses.");
DEFINE_string(orthomosaic_image_directory,
              "/run/user/1000/gvfs/smb-share:server=192.168.1.96,share=base/000000_exchange/reduced40/images/", "Image directory.");
DEFINE_double(orthomosaic_easting_min, 464899.00, "");
DEFINE_double(orthomosaic_northing_min, 5272160.00, "");
DEFINE_double(orthomosaic_easting_max, 465056.47, "");
DEFINE_double(orthomosaic_northing_max, 5272349.53, "");
DEFINE_int32(orthomosaic_UTM_code, 32, "");
DEFINE_double(orthomosaic_resolution, 1, "");
DEFINE_string(orthomosaic_camera_calibration_yaml,
              "/run/user/1000/gvfs/smb-share:server=192.168.1.96,share=base/000000_exchange/reduced40/camera_fixed_wing.yaml", "");
DEFINE_string(orthomosaic_filename_height_map, "/tmp/height_map.jpeg","");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "online_orthomosaic_node");
  OrthomosaicBackprojection orthomosaic;

  return 0;
}
