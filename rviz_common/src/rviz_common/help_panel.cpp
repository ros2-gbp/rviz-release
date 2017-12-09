/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_common/help_panel.hpp"

#include <string>

#include <QVBoxLayout>  // NOLINT: cpplint is unable to handle the include order here
#include <QTextBrowser>  // NOLINT: cpplint is unable to handle the include order here
#include <QUrl>  // NOLINT: cpplint is unable to handle the include order here

#include <boost/filesystem.hpp>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/visualization_manager.hpp"

namespace fs = boost::filesystem;

namespace rviz
{

HelpPanel::HelpPanel(QWidget * parent)
: Panel(parent),
  browser_(NULL)
{
  QVBoxLayout * layout = new QVBoxLayout(this);
  browser_ = new QTextBrowser();
  layout->addWidget(browser_);
}

HelpPanel::~HelpPanel()
{
}

void HelpPanel::onInitialize()
{
  setHelpFile(vis_manager_->getHelpPath() );
}

void HelpPanel::setHelpFile(const QString & qfile_path)
{
  std::string file_path = qfile_path.toStdString();

  if (!fs::exists(file_path)) {
    browser_->setText("Help file '" + qfile_path + "' does not exist.");
  } else if (fs::is_directory(file_path)) {
    browser_->setText("Help file '" + qfile_path + "' is a directory, not a file.");
  } else {
    QUrl url = QUrl::fromLocalFile(qfile_path);
    if (browser_->source() == url) {
      browser_->reload();
    } else {
      browser_->setSource(url);
    }
  }
}

}  // end namespace rviz
