// Copyright (c) 2010, Willow Garage, Inc. Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__XYZ_PC_TRANSFORMER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__XYZ_PC_TRANSFORMER_HPP_

#include <vector>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rviz_common/properties/property.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC XYZPCTransformer : public PointCloudTransformer
{
public:
  uint8_t supports(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) override;

  bool transform(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    uint32_t mask,
    const Ogre::Matrix4 & transform,
    V_PointCloudPoint & points_out) override;
};

}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__XYZ_PC_TRANSFORMER_HPP_
