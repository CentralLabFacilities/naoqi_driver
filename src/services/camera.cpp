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

CameraService::CameraService( const std::string& name, const std::string& topic, const qi::SessionPtr& session, const int& camera_source, const int& resolution, const float& frequency )
  : name_(name),
  p_video_( session->service("ALVideoDevice") ),
  camera_source_(camera_source),
  resolution_(resolution),
  colorspace_( (camera_source_!=AL::kDepthCamera)?AL::kRGBColorSpace:AL::kRawDepthColorSpace ),
  msg_colorspace_( (camera_source_!=AL::kDepthCamera)?"rgb8":"16UC1" ),
  cv_mat_type_( (camera_source_!=AL::kDepthCamera)?CV_8UC3:CV_16U ),
  frequency_(frequency),
  topic_(topic),
  session_(session)
{
    if ( camera_source == AL::kTopCamera )
    {
      msg_frameid_ = "CameraTop_optical_frame";
    }
    else if (camera_source == AL::kBottomCamera )
    {
      msg_frameid_ = "CameraBottom_optical_frame";
    }
    else if (camera_source_ == AL::kDepthCamera )
    {
      msg_frameid_ = "CameraDepth_optical_frame";
    }
    // Overwrite the parameters for the infrared
    else if (camera_source_ == AL::kInfraredCamera )
    {
      // Reset to kDepth since it's the same device handle
      camera_source_ = AL::kDepthCamera;
      msg_frameid_ = "CameraDepth_optical_frame";
      colorspace_ = AL::kInfraredColorSpace;
      msg_colorspace_ = "16UC1";
      cv_mat_type_ = CV_16U;
    }
}

void CameraService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &CameraService::callback, this);
}

bool CameraService::callback( pepper_clf_msgs::DepthAndColorImage::Request &req, pepper_clf_msgs::DepthAndColorImage::Response &resp )
{
    handle_ = p_video_.call<std::string>(
                           "subscribeCamera",
                            "depth_camera",
                            camera_source_,
                            resolution_,
                            colorspace_,
                            frequency_
                            );

    qi::AnyValue image_anyvalue = p_video_.call<qi::AnyValue>("getImageRemote", handle_);
    tools::NaoqiImage image;

    try{
        image = tools::fromAnyValueToNaoqiImage(image_anyvalue);
    }
    catch(std::runtime_error& e)
    {
      std::cout << "Cannot retrieve image" << std::endl;
      return false;
    }

    cv::Mat cv_img(image.height, image.width, cv_mat_type_, image.buffer);
    resp.depth = *cv_bridge::CvImage(std_msgs::Header(), msg_colorspace_, cv_img).toImageMsg();
    resp.depth.header.frame_id = msg_frameid_;

    resp.depth.header.stamp = ros::Time::now();
    p_video_.call<qi::AnyValue>("releaseImage", handle_);

    return true;
}


}
}
