// Copyright (c) 2017, Bosch Software Innovations GmbH.
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


#include "ogre_testing_environment.hpp"

#include <string>

#include <OgreLogManager.h>

#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/render_system.hpp"

namespace rviz_default_plugins
{

void OgreTestingEnvironment::setUpOgreTestEnvironment(bool debug)
{
  if (!debug) {
    const std::string & name = "";
    auto lm = new Ogre::LogManager();
    lm->createLog(name, false, debug, true);
  }
  setUpRenderSystem();
}

void OgreTestingEnvironment::setUpRenderSystem()
{
  rviz_rendering::RenderSystem::get();
}

Ogre::RenderWindow * OgreTestingEnvironment::createOgreRenderWindow()
{
  auto test = new rviz_rendering::RenderWindow();
  return rviz_rendering::RenderSystem::get()->makeRenderWindow(test->winId(), 10, 10, 1.0);
}

}  // end namespace rviz_default_plugins
