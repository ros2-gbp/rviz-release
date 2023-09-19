/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_DEFAULT_PLUGINS__ROBOT__MOCK_LINK_UPDATER_HPP_
#define RVIZ_DEFAULT_PLUGINS__ROBOT__MOCK_LINK_UPDATER_HPP_

#include <gmock/gmock.h>

#include <string>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "rviz_default_plugins/robot/link_updater.hpp"

class MockLinkUpdater : public rviz_default_plugins::robot::LinkUpdater
{
public:
  MOCK_CONST_METHOD5(
    getLinkTransforms, bool(
      const std::string & link_name,
      Ogre::Vector3 & visual_position,
      Ogre::Quaternion & visual_orientation,
      Ogre::Vector3 & collision_position,
      Ogre::Quaternion & collision_orientation));

  MOCK_CONST_METHOD3(
    setLinkStatus, void(
      rviz_common::properties::StatusLevel level,
      const std::string & link_name,
      const std::string & text));
};

#endif  // RVIZ_DEFAULT_PLUGINS__ROBOT__MOCK_LINK_UPDATER_HPP_
