/*
 * Utils.h
 *
 *  Created on: Sep 25, 2014
 *      Author: andreas
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <mono_dense_reconstruction/StereoPair.h>

class Utils{
public:
static void convertCvPclToRosPCL2Msg(const cv::Mat_<cv::Vec3f>& cvPcl, const cv::Mat& cvPclIntensity, const std::string& referenceFrameName, const ros::Time& timestamp, sensor_msgs::PointCloud2& rosPcl2Msg);
static void convertDispMapToRosMsg(const cv::Mat& disparityMap, const std::string& referenceFrameName, const ros::Time& timestamp, stereo_msgs::DisparityImage& disparityRosMsg);
};
#endif /* UTILS_H_ */
