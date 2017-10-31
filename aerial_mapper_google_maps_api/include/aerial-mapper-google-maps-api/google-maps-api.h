/*
 *    Filename: google-maps-api.h
 *  Created on: Oct 31, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef GOOGLE_MAPS_API_H_
#define GOOGLE_MAPS_API_H_

// SYSTEM
#include <iostream>
#include <string>
#include <vector>

// NON-SYSTEM
#include <aerial-mapper-thirdparty/gps-conversions.h>
#include <curl/curl.h>
#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>


class GoogleMapsApi {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GoogleMapsApi(double longitude, double latitude, int zoom);

  cv::Mat getMap() const { return map_; }

 private:
  /// Retrieve the ortho as cv::Mat.
  cv::Mat curlImage(const char* image_url) const;

  /// Generate URL and retrieve map from Google Maps API.
  void createURL(double longitude, double latitude, int zoom);

  /// Convert from pixel coordinates (col, row) to UTM coordinates.
  std::pair<double, double> pixelToUtm(
      const double col, const double row, const double camera_northing,
      const double camera_easting, const double cruise_altitude,
      const Eigen::Matrix3d& pixel_direction_pre) const;

  /// Convert from UTM coordinates to pixel coordinates (col, row).
  std::pair<int, int> utmToPixel(
      const double northing, const double easting, const double camera_northing,
      const double camera_easting, const double cruise_altitude,
      const Eigen::Matrix3d& pixel_direction_pre_inverse) const;

  /// Convert from Longitude, Latitude to pixel coordinates (col, row).
  std::pair<int, int> longLatToPixel(double longitude, double latitude) const;

  std::string mapURL_;
  cv::Mat map_;
  int zoom_;
  std::pair<double, double> center_;
};

#endif // GOOGLE_MAPS_API_H_
