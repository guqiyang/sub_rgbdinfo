/*!
 * Author: guqiyang guqiyang@aicrobo.com
 * Maintainer: guqiyang guqiyang@aicrobo.com
 * Group: AICRobo http://www.aicrobo.com
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013~2015, AICRobo.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

/*
 * 回调函数，将图片显示出来
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // convert sensor_msgs/Image to Mat
  cv_bridge::CvImagePtr cvImgPtr;
  cv::Mat_<cv::Vec3b> img;

  try
  {
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img = cvImgPtr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  cv::imshow("opencv_show", img);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "opencv_show");
  ros::NodeHandle nh;

  //if use Kinect1.0, use topic: "/camera/depth/image_raw"
  ros::Subscriber imageRawSub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  ROS_INFO("opencv_show started...");
  ros::spin();
  return 0;
}
