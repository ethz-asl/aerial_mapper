/*
 *    Filename: google-maps-api.cc
 *  Created on: Oct 31, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-google-maps-api/google-maps-api.h"

static size_t write_data(char*ptr, size_t size, size_t nmemb, void* userdata) {
  std::vector<uchar> *stream = (std::vector<uchar> *)userdata;
  size_t count = size * nmemb;
  stream->insert(stream->end(), ptr, ptr + count);
  return count;
}

GoogleMapsApi::GoogleMapsApi(double longitude, double latitude, int zoom) {
  createURL(longitude, latitude, zoom);
  zoom_ = zoom;
  center_.first = longitude;
  center_.second = latitude;
  map_ = curlImage(mapURL_.c_str());
}

void GoogleMapsApi::createURL(double longitude, double latitude, int zoom) {
  std::stringstream ss;
  ss << "http://maps.googleapis.com/maps/api/staticmap?size=640x640&center="
     << latitude << "," << longitude << "&zoom=" << zoom
     << "&sensor=false&maptype=satellite";
  mapURL_ = ss.str();
  LOG(INFO) << "Map URL is: " << mapURL_;
}

cv::Mat GoogleMapsApi::curlImage(const char* img_url) const {
  std::vector<uchar> stream;
  CURL *curl = curl_easy_init();
  curl_easy_setopt(curl, CURLOPT_URL, img_url);
  // Pass the writefunction.
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
  // Pass the stream ptr to the writefunction.
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream);
  // Timeout after 10s if curl_easy hangs.
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10);
  curl_easy_perform(curl);
  curl_easy_cleanup(curl);
  return cv::imdecode(stream, -1);
}

std::pair<int, int> GoogleMapsApi::longLatToPixel(double longitude,
                                                  double latitude) const {
  std::pair<int, int> pixel;
  double metersPerPixel =
      156543.03392 * cos(latitude * CV_PI / 180) / pow(2, zoom_);
  double northing, easting, center_northing, center_easting;
  char zone[10];
  UTM::LLtoUTM(latitude, longitude, northing, easting, zone);
  UTM::LLtoUTM(center_.second, center_.first, center_northing, center_easting,
               zone);
  pixel.first = map_.rows / 2 - (northing - center_northing) / metersPerPixel;
  pixel.second = map_.cols / 2 + (easting - center_easting) / metersPerPixel;
  return pixel;
}

std::pair<double, double> GoogleMapsApi::pixelToUtm(
    const double col, const double row, const double camera_northing,
    const double camera_easting, const double cruise_altitude,
    const Eigen::Matrix3d& pixel_direction_pre) const {
  Eigen::Vector3d pixel(col, row, 1.0);
  Eigen::Vector3d direction = pixel_direction_pre * pixel;
  CHECK(direction(2) != 0.0) << "GPS coordinates indeterminant!";
  double lambda = (-cruise_altitude) / direction(2);
  double northing = lambda * direction[0] + camera_northing;
  double easting = -lambda * direction[1] + camera_easting;
  if (lambda > 0) {
    return std::pair<double, double>(northing, easting);
  } else {
    return std::pair<double, double>(0, 0);
  }
}

std::pair<int, int> GoogleMapsApi::utmToPixel(
    const double northing, const double easting, const double camera_northing,
    const double camera_easting, const double cruise_altitude,
    const Eigen::Matrix3d& pixel_direction_pre_inverse) const {
  double lambda_d0 = northing - camera_northing;
  double lambda_d1 = camera_easting - easting;
  // FIXME
  double alt_correct = 1.0;
  double lambda_d2 = -cruise_altitude * alt_correct;
  Eigen::Vector3d lambda_d(lambda_d0, lambda_d1, lambda_d2);
  Eigen::Vector3d pixel = pixel_direction_pre_inverse * lambda_d;
  CHECK(pixel(2) != 0.0) << "Pixel coordinates indeterminant!";
  return std::pair<int, int>(static_cast<int>(pixel(0) / pixel(2)),
                             static_cast<int>(pixel(1) / pixel(2)));
}
