/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "camera.hpp"
#include "../converters/camera_info_definitions.hpp"
#include "../helpers/driver_helpers.hpp"
#include "../tools/alvisiondefinitions.h" // for kTop...
#include "../tools/alvision.h" // for ALImage...
#include "../tools/from_any_value.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

namespace naoqi
{
namespace service
{

CameraService::CameraService( const std::string& name, const std::string& topic, const qi::SessionPtr& session, const int& camera_source_depth, const int& resolution_depth, const float& frequency_depth,
                              const int& camera_source_front, const int& resolution_front, const float& frequency_front)
  : name_(name),
  p_video_depth_(session->service("ALVideoDevice")),
  p_video_color_(session->service("ALVideoDevice")),
  camera_source_depth_(camera_source_depth),
  resolution_depth_(resolution_depth),
  colorspace_depth_( (camera_source_depth!=AL::kDepthCamera)?AL::kRGBColorSpace:AL::kRawDepthColorSpace ),
  msg_colorspace_depth_( (camera_source_depth!=AL::kDepthCamera)?"rgb8":"16UC1" ),
  cv_mat_type_depth_( (camera_source_depth!=AL::kDepthCamera)?CV_8UC3:CV_16U ),
  frequency_depth_(frequency_depth),
  topic_(topic),
  camera_source_front_(camera_source_front),
  resolution_front_(resolution_front),
  colorspace_front_( (camera_source_front!=AL::kDepthCamera)?AL::kRGBColorSpace:AL::kRawDepthColorSpace ),
  msg_colorspace_front_( (camera_source_front!=AL::kDepthCamera)?"rgb8":"16UC1" ),
  cv_mat_type_front_( (camera_source_front!=AL::kDepthCamera)?CV_8UC3:CV_16U ),
  frequency_front_(frequency_front),
  session_(session)
  {
    if ( camera_source_depth == AL::kTopCamera )
    {
      msg_frameid_depth_ = "CameraTop_optical_frame";
    }
    else if (camera_source_depth == AL::kBottomCamera )
    {
      msg_frameid_depth_ = "CameraBottom_optical_frame";
    }
    else if (camera_source_depth == AL::kDepthCamera )
    {
      msg_frameid_depth_ = "CameraDepth_optical_frame";
    }
    // Overwrite the parameters for the infrared
    else if (camera_source_depth == AL::kInfraredCamera )
    {
      // Reset to kDepth since it's the same device handle
      camera_source_depth_ = AL::kDepthCamera;
      msg_frameid_depth_ = "CameraDepth_optical_frame";
      colorspace_depth_ = AL::kInfraredColorSpace;
      msg_colorspace_depth_ = "16UC1";
      cv_mat_type_depth_ = CV_16U;
    }

    if ( camera_source_front == AL::kTopCamera )
    {
      msg_frameid_front_ = "CameraTop_optical_frame";
    }
    else if (camera_source_front == AL::kBottomCamera )
    {
      msg_frameid_front_ = "CameraBottom_optical_frame";
    }
    else if (camera_source_front == AL::kDepthCamera )
    {
      msg_frameid_front_ = "CameraDepth_optical_frame";
    }
    // Overwrite the parameters for the infrared
    else if (camera_source_front == AL::kInfraredCamera )
    {
      // Reset to kDepth since it's the same device handle
      camera_source_front_ = AL::kDepthCamera;
      msg_frameid_front_ = "CameraDepth_optical_frame";
      colorspace_front_ = AL::kInfraredColorSpace;
      msg_colorspace_front_ = "16UC1";
      cv_mat_type_front_ = CV_16U;
    }
  }

CameraService::~CameraService() {
    if (!handle_depth_.empty())
    {
      p_video_depth_.call<qi::AnyValue>("unsubscribe", handle_depth_);
      handle_depth_.clear();
      ROS_INFO(">>> [Service] camera depth unsubscribe");
    }
    if (!handle_front_.empty())
    {
      p_video_color_.call<qi::AnyValue>("unsubscribe", handle_front_);
      handle_front_.clear();
      ROS_INFO(">>> [Service] camera color unsubscribe");
    }
}

void CameraService::reset( ros::NodeHandle& nh ) {

  if (!handle_depth_.empty())
  {
    p_video_depth_.call<qi::AnyValue>("unsubscribe", handle_depth_);
    handle_depth_.clear();
    std::cout << "[Service] reset " << handle_depth_ << std::endl;
  }

  if (!handle_front_.empty())
  {
    p_video_color_.call<qi::AnyValue>("unsubscribe", handle_front_);
    handle_front_.clear();
    std::cout << "[Service] reset " << handle_front_ << std::endl;
  }

  handle_depth_ = p_video_depth_.call<std::string>(
                         "subscribeCamera",
                          name_,
                          camera_source_depth_,
                          resolution_depth_,
                          colorspace_depth_,
                          frequency_depth_
                          );

  handle_front_ = p_video_color_.call<std::string>(
                         "subscribeCamera",
                          name_,
                          camera_source_front_,
                          resolution_front_,
                          colorspace_front_,
                          frequency_front_
                          );

  service_ = nh.advertiseService(topic_, &CameraService::callback, this);

  std::cout << "[Service] rgb+depth init done" << std::endl;

}

bool CameraService::callback( pepper_clf_msgs::DepthAndColorImage::Request &req, pepper_clf_msgs::DepthAndColorImage::Response &resp )
{

    ROS_INFO(">>> [Service] camera depth+rgb called");

    // DEPTH

    qi::AnyValue image_anyvalue_d = p_video_depth_.call<qi::AnyValue>("getImageRemote", handle_depth_);
    tools::NaoqiImage image_d;

    try{
        image_d = tools::fromAnyValueToNaoqiImage(image_anyvalue_d);
    } catch (std::runtime_error& e) {
      ROS_ERROR("[Service] cannot retrieve depth image: %s", e.what());
      resp.success = false;
      return false;
    }

    cv::Mat cv_img_depth(image_d.height, image_d.width, cv_mat_type_depth_, image_d.buffer);
    resp.depth = *cv_bridge::CvImage(std_msgs::Header(), msg_colorspace_depth_, cv_img_depth).toImageMsg();
    resp.depth.header.frame_id = msg_frameid_depth_;
    resp.depth.header.stamp = ros::Time::now();

    // COLOR

    qi::AnyValue image_anyvalue_c = p_video_color_.call<qi::AnyValue>("getImageRemote", handle_front_);
    tools::NaoqiImage image_c;

    try{
        image_c = tools::fromAnyValueToNaoqiImage(image_anyvalue_c);
    } catch(std::runtime_error& e) {
      ROS_ERROR("[Service] cannot retrieve color image: %s", e.what());
      resp.success = false;
      return false;
    }

    cv::Mat cv_img_front(image_c.height, image_c.width, cv_mat_type_front_, image_c.buffer);
    resp.color = *cv_bridge::CvImage(std_msgs::Header(), msg_colorspace_front_, cv_img_front).toImageMsg();
    resp.color.header.frame_id = msg_frameid_front_;
    resp.color.header.stamp = ros::Time::now();

    // delete im; Not really neccessary
    // p_video_color_.call<qi::AnyValue>("releaseImage", handle_front_);
    // delete im; Not really neccessary
    // p_video_depth_.call<qi::AnyValue>("releaseImage", handle_depth_);

    resp.success = true;
    return true;
}
}
}
