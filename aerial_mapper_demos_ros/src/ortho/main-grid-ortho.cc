#include "fw-ortho/orthomosaic-backprojection.h"



DEFINE_string(orthomosaic_filename_poses,
              "/run/user/1000/gvfs/smb-share:server=192.168.1.96,share=base/000000_exchange/reduced40/calibrated_external_camera_parameters_wgs84.txt",
              "Filename of the camera poses.");
DEFINE_string(orthomosaic_image_directory,
              "/run/user/1000/gvfs/smb-share:server=192.168.1.96,share=base/000000_exchange/reduced40/images/", "Image directory.");
DEFINE_double(orthomosaic_easting_min, 464899.00-400, "");
DEFINE_double(orthomosaic_northing_min, 5272160.00-200, "");
DEFINE_double(orthomosaic_easting_max, 465056.47, "");
DEFINE_double(orthomosaic_northing_max, 5272349.53+400, "");
DEFINE_int32(orthomosaic_UTM_code, 32, "");
DEFINE_double(orthomosaic_resolution, 0.5, "");
DEFINE_string(orthomosaic_camera_calibration_yaml,
              "/run/user/1000/gvfs/smb-share:server=192.168.1.96,share=base/000000_exchange/reduced40/camera_fixed_wing.yaml", "");
DEFINE_string(orthomosaic_filename_height_map, "/tmp/height_map.jpeg","");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "online_orthomosaic_node");

  ros::Time::init();
  // Load text file.
  aslam::TransformationVector T_G_Bs;
  std::string file = "/tmp/opt_poses.txt";
  std::cout << "OPEN: " << file << std::endl;
  std::ifstream infile(file);
  //int64_t ts;
  double x,y,z,qw,qx,qy,qz;
  while( infile >> x >> y >> z >> qw >> qx >> qy >> qz ){
    std::cout << qw << " " << qx << " " << qy << " " << qz << std::endl;
    aslam::Quaternion q(qw,qx,qy,qz);
    Eigen::Vector3d t(x,y,z);
    aslam::Transformation T(q,t);
    T_G_Bs.push_back(T);
    if( infile.eof() ) break;
  }
  std::cout << "T_G_Bs.size() = " << T_G_Bs.size() << std::endl;

  // Load images.
  std::vector<cv::Mat> images;
  for (size_t i = 0u; i < T_G_Bs.size(); ++i) {
    std::string filename = "/tmp/opt_image_" + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("Image", image);
    cv::waitKey(1);
    images.push_back(image);
  }

  std::string ncameras_yaml_path_filename =
      "/home/timo/calibration/camera_fixed_wing.yaml";
  OrthomosaicBackprojection orthomosaic(ncameras_yaml_path_filename,
                                        T_G_Bs, images);

  return 0;
}
