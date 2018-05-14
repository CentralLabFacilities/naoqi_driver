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


#ifndef CAMERA_SERVICE_HPP
#define CAMERA_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <pepper_clf_msgs/DepthAndColorImage.h>
#include <qi/session.hpp>

#include <image_transport/image_transport.h>

#include <naoqi_driver/tools.hpp>
#include "../helpers/driver_helpers.hpp"

/*
* ALDEBARAN includes
*/
#include <qi/session.hpp>
#include <qi/anyobject.hpp>

namespace naoqi
{
namespace service
{

class CameraService
{
public:
  CameraService( const std::string& name, const std::string& topic, const qi::SessionPtr& session, const int& camera_source_depth, const int& resolution_depth, const float& frequency_depth,const int& camera_source_front, const int& resolution_front, const float& frequency_front );

  ~CameraService(){};

  std::string name() const
  {
    return name_;
  }

  std::string topic() const
  {
    return topic_;
  }

  void reset( ros::NodeHandle& nh );

  bool callback( pepper_clf_msgs::DepthAndColorImage::Request &req, pepper_clf_msgs::DepthAndColorImage::Response &resp );


private:
  const std::string name_;
  const std::string topic_;
  qi::AnyObject p_video_;

  int camera_source_depth_;
  int resolution_depth_;
  int colorspace_depth_;
  float frequency_depth_;
  std::string handle_depth_;
  std::string msg_colorspace_depth_;
  int cv_mat_type_depth_;
  std::string msg_frameid_depth_;

  int camera_source_front_;
  int resolution_front_;
  int colorspace_front_;
  float frequency_front_;
  std::string handle_front_;
  std::string msg_colorspace_front_;
  int cv_mat_type_front_;
  std::string msg_frameid_front_;

  sensor_msgs::CameraInfo camera_info_;

  const qi::SessionPtr& session_;
  ros::ServiceServer service_;
};

} // service
} // naoqi
#endif
