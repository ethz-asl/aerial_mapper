/*
 * Utils.cpp
 *
 *  Created on: Sep 25, 2014
 *      Author: andreas
 */
#include <fstream>
#include "mono_dense_reconstruction_nodes/test/Utils.h"

void Utils::convertCvPclToRosPCL2Msg(const cv::Mat_<cv::Vec3f>& cvPcl,
                                     const cv::Mat& cvPclIntensity,
                                     const std::string& referenceFrameName,
                                     const ros::Time& timestamp,
                                     sensor_msgs::PointCloud2& rosPcl2Msg) {
  // Fill in new PointCloud2 message
  rosPcl2Msg.header.stamp = timestamp;
  rosPcl2Msg.header.frame_id = referenceFrameName;
  rosPcl2Msg.height = cvPcl.rows;
  rosPcl2Msg.width = cvPcl.cols;
  rosPcl2Msg.fields.resize(4);

  rosPcl2Msg.fields[0].name = "x";
  rosPcl2Msg.fields[0].offset = 0;
  rosPcl2Msg.fields[0].count = 1;
  rosPcl2Msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;

  rosPcl2Msg.fields[1].name = "y";
  rosPcl2Msg.fields[1].offset = 4;
  rosPcl2Msg.fields[1].count = 1;
  rosPcl2Msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;

  rosPcl2Msg.fields[2].name = "z";
  rosPcl2Msg.fields[2].offset = 8;
  rosPcl2Msg.fields[2].count = 1;
  rosPcl2Msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;

  rosPcl2Msg.fields[3].name = "rgb";
  rosPcl2Msg.fields[3].offset = 12;
  rosPcl2Msg.fields[3].count = 1;
  rosPcl2Msg.fields[3].datatype = sensor_msgs::PointField::UINT32;

  rosPcl2Msg.point_step = 16;  // Length of a point in bytes
                               // float32->32bit=4byte
  rosPcl2Msg.row_step =
      rosPcl2Msg.point_step * rosPcl2Msg.width;  // Length of a row in bytes
  rosPcl2Msg.data.resize(
      rosPcl2Msg.row_step *
      rosPcl2Msg.height);  // Actual point data, size is (row_step*height)
  rosPcl2Msg.is_dense = false;  // There may be invalid points

  float badPoint = std::numeric_limits<float>::quiet_NaN();  // NaN
  const unsigned char* intensityPtr;
  int offset = 0;

  //    const double x_origin = 464980;
  //    const double y_origin = 5.27226e+06;
  //    const double z_origin = 414.087;

  //    for (int v = 0; v < cvPcl.rows; ++v) {
  //        for (int u = 0; u < cvPcl.cols; ++u, offset +=rosPcl2Msg.point_step)
  //        {
  //          cvPcl.operator()(v, u)[0] =
  //          cvPcl.operator()(v, u)[1] = cvPcl.operator()(v, u)[1] - y_origin;
  //          cvPcl.operator()(v, u)[2] = cvPcl.operator()(v, u)[2] - z_origin;
  //        }
  //    }

  std::string filename = "/tmp/pointcloud.txt";
  std::ofstream fs;
  std::ifstream ifile(filename.c_str());
  if (ifile) {
    fs.open(filename.c_str(), std::fstream::app | std::fstream::out);
  } else {
    fs.open(filename.c_str());
  }

  // std::cout << "rows/cols: " << cvPcl.rows << "/" << cvPcl.cols << std::endl;
  for (int v = 0; v < cvPcl.rows; ++v) {
    intensityPtr = cvPclIntensity.ptr<unsigned char>(v);
    for (int u = 0; u < cvPcl.cols; ++u, offset += rosPcl2Msg.point_step) {
      if (dense::isValidPoint(cvPcl.operator()(v, u))) {
        //			  std::cout << "x/y/z = " <<  cvPcl.operator()(v,
        //u)[0] << "/"
        //						<< cvPcl.operator()(v, u)[1] <<
        //"/" << cvPcl.operator()(v, u)[2] << std::endl;
        //                std::cout << "cvPcl.operator()(v, u)[0] = " <<
        //                cvPcl.operator()(v, u)[0] << std::endl;
        //				// Copy the point cloud data to the ros msg
        //data if current point is valid
        //                const double x = cvPcl.operator()(v, u)[0] - x_origin;
        //                const double y = cvPcl.operator()(v, u)[1] - y_origin;
        //                const double z = cvPcl.operator()(v, u)[2] - z_origin;
        memcpy(&rosPcl2Msg.data[offset + 0], &cvPcl.operator()(v, u)[0],
               sizeof(float));  // x
        memcpy(&rosPcl2Msg.data[offset + 4], &cvPcl.operator()(v, u)[1],
               sizeof(float));  // y
        memcpy(&rosPcl2Msg.data[offset + 8], &cvPcl.operator()(v, u)[2],
               sizeof(float));  // z

        // Add Color (gray values)
        uint8_t gray = intensityPtr[u];
        uint32_t rgb = (gray << 16) | (gray << 8) | gray;
        memcpy(&rosPcl2Msg.data[offset + 12], &rgb, sizeof(uint32_t));
        // std::cout << "gray = " << gray << ", rgb = " << rgb << std::endl;
        fs << std::setprecision(12) << cvPcl.operator()(v, u)[0] << " "
           << cvPcl.operator()(v, u)[1] << " " << cvPcl.operator()(v, u)[2]
           << " " << int(gray) << std::endl;

      } else {
        // If current point is not valid copy NaN
        memcpy(&rosPcl2Msg.data[offset + 0], &badPoint,  // x
               sizeof(float));
        memcpy(&rosPcl2Msg.data[offset + 4], &badPoint,  // y
               sizeof(float));
        memcpy(&rosPcl2Msg.data[offset + 8], &badPoint,  // z
               sizeof(float));

        memcpy(&rosPcl2Msg.data[offset + 12], &badPoint, sizeof(float));
      }
    }
  }
  fs.close();
  //	cv::waitKey(0);
}

void Utils::convertDispMapToRosMsg(
    const cv::Mat& disparityMap, const std::string& referenceFrameName,
    const ros::Time& timestamp, stereo_msgs::DisparityImage& disparityRosMsg) {
  disparityRosMsg.header.stamp = timestamp;
  disparityRosMsg.header.frame_id = referenceFrameName;
  disparityRosMsg.image.height = disparityMap.rows;
  disparityRosMsg.image.width = disparityMap.cols;
  disparityRosMsg.image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disparityRosMsg.image.step = disparityRosMsg.image.width * sizeof(float);
  disparityRosMsg.image.data.resize(disparityRosMsg.image.height *
                                    disparityRosMsg.image.step);

  // Find Maximum Disparity
  float maxDisp =
      *std::max_element(disparityMap.begin<float>(), disparityMap.end<float>());
  float minDisp =
      *std::min_element(disparityMap.begin<float>(), disparityMap.end<float>());

  int minDisparity = (int)minDisp;
  int numberOfDisparities = (int)maxDisp - (int)minDisp;

  int border = 10;
  int left = border - 1;
  int right = disparityRosMsg.image.width - border - 1;
  int top = border - 1;
  int bottom = disparityRosMsg.image.height - border - 1;

  disparityRosMsg.valid_window.x_offset = left;
  disparityRosMsg.valid_window.y_offset = top;
  disparityRosMsg.valid_window.width = right - left;
  disparityRosMsg.valid_window.height = bottom - top;

  // Disparity search range
  disparityRosMsg.min_disparity = minDisparity;
  disparityRosMsg.max_disparity = minDisparity + numberOfDisparities - 1;
  disparityRosMsg.delta_d = 1.0;

  // Create cv::Mat view onto disp_msg->image
  cv::Mat_<float> disparityImageMsg(
      disparityRosMsg.image.height, disparityRosMsg.image.width,
      reinterpret_cast<float*>(&disparityRosMsg.image.data[0]),
      disparityRosMsg.image.step);

  disparityMap.convertTo(disparityImageMsg, CV_32F);
}
