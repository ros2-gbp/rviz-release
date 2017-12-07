/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_TRIANGLE_LIST_MARKER_H
#define RVIZ_TRIANGLE_LIST_MARKER_H

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include "rviz/default_plugin/markers/marker_base.h"

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace rviz
{

class TriangleListMarker : public MarkerBase
{
public:
  TriangleListMarker(MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node);
  ~TriangleListMarker();

  virtual S_MaterialPtr getMaterials();

protected:
  virtual void onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message);

  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr material_;
  std::string material_name_;
};

}

#endif // RVIZ_TRIANGLE_LIST_MARKER_H


