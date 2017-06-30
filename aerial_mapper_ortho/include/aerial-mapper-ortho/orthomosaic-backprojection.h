#ifndef FW_ORTHOMOSAIC_BACKPROJECTION_H_
#define FW_ORTHOMOSAIC_BACKPROJECTION_H_

#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <aslam/pipeline/undistorter-mapped.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/cameras/ncamera.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/pipeline/visual-npipeline.h>
#include <vi-map-dmap/table-file-io.h>
#include <vi-map-dmap/view.h>
#include <vi-map-dmap/vi-map-dmap.h>

class OrthomosaicBackprojection {
 public:
    OrthomosaicBackprojection();
    void Bresenham(int x1,
        int y1,
        int const x2,
        int const y2);
    OrthomosaicBackprojection(std::string ncameras_yaml_path_filename,
                              const aslam::TransformationVector& T_G_Bs,
                              const std::vector<cv::Mat>& images);
    void loadCameraRig(std::string ncameras_yaml_path_filename);
    void process(const aslam::TransformationVector& T_G_Cs,
                 const std::vector<cv::Mat>& images);
    void processIncremental(const aslam::TransformationVector& T_G_Cs,
                            const std::vector<cv::Mat>& images);

    void loadPosesAndImagesFromViMap(std::string path_vi_map,
                                     aslam::TransformationVector* T_G_Cs,
                                     std::vector<cv::Mat>* images);

    void loadPosesAndImagesFromPix4d(std::string filename_poses,
                                     std::string filename_images,
                                     std::string ncameras_yaml_path_filename,
                                     aslam::TransformationVector* poses,
                                     std::vector<cv::Mat>* image_names);

    void loadPosesFromPix4d(std::string filename_poses,
                            aslam::TransformationVector* poses,
                            std::vector<std::string>* image_names);

    bool withinImageBoxWithBorder(Eigen::Vector2d keypoint,
                                  const aslam::Camera& camera);

    void loadImages(const std::string& image_directory,
                    const std::vector<std::string>& image_names,
                    std::vector<cv::Mat>* images);

    void initializeCameraRig(const std::string& ncameras_yaml_path_filename);

    void loadHeightMap(const std::string filename, cv::Mat* height_map);

    std::shared_ptr<aslam::NCamera> ncameras_;
    static constexpr size_t kFrameIdx = 0u;
};

#endif
