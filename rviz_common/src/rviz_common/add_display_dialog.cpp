/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include "./add_display_dialog.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include <QCheckBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QComboBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QDialogButtonBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QGroupBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QHeaderView>  // NOLINT: cpplint is unable to handle the include order here
#include <QLabel>  // NOLINT: cpplint is unable to handle the include order here
#include <QLineEdit>  // NOLINT: cpplint is unable to handle the include order here
#include <QPushButton>  // NOLINT: cpplint is unable to handle the include order here
#include <QTabWidget>  // NOLINT: cpplint is unable to handle the include order here
#include <QTextBrowser>  // NOLINT: cpplint is unable to handle the include order here
#include <QVBoxLayout>  // NOLINT: cpplint is unable to handle the include order here

#include "rcl/validate_topic_name.h"

#include "./display_factory.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/get_topic_names_and_types.hpp"

namespace rviz_common
{

bool validate_ros_topic(const std::string & topic_name, std::string & output_error)
{
  int validation_result;
  size_t invalid_index;
  rcl_ret_t ret = rcl_validate_topic_name(topic_name.c_str(), &validation_result, &invalid_index);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("failed to call rcl_validate_topic_name()");
  }
  if (validation_result == RCL_TOPIC_NAME_VALID) {
    return true;
  }
  const char * validation_error = rcl_topic_name_validation_result_string(validation_result);
  if (!validation_error) {
    throw std::runtime_error("failed to get the validation error reason");
  }
  output_error = std::string("topic '") + topic_name + "' is invalid because: " + validation_error;
  return false;
}

// TODO(wjwwood): use an rclcpp utility to do this
std::string get_topic_parent(const std::string & topic_name)
{
  const auto pos = topic_name.find_last_of('/');
  if (pos == std::string::npos || topic_name == "/") {
    return topic_name;
  }
  return topic_name.substr(0, pos);
}

/**
 * Return true if one topic is a subtopic of the other.
 *
 * A topic is a subtopic of another if a subset of its path exactly matches the
 * other.  For example, /camera/image_raw/compressed is a subtopic of
 * /camera/image_raw but not /camera/image.
 *
 * @param base A valid ROS topic
 *
 * @param topic A valid ROS topic
 *
 * @return True if topic is a subtopic of base.  False otherwise or if either
 *         argument is an invalid ROS topic.
 */
bool isSubtopic(const std::string & base, const std::string & topic)
{
  std::string error;
  if (!validate_ros_topic(base, error)) {
    RVIZ_COMMON_LOG_ERROR_STREAM("isSubtopic() Invalid basename: " << error);
    return false;
  }
  if (!validate_ros_topic(topic, error)) {
    RVIZ_COMMON_LOG_ERROR_STREAM("isSubtopic() Invalid topic: " << error);
    return false;
  }

  std::string query = topic;
  while (query != "/") {
    if (query == base) {
      return true;
    }
    query = get_topic_parent(query);
  }
  return false;
}

struct PluginGroup
{
  struct Info
  {
    QStringList topic_suffixes;
    QStringList datatypes;
  };

  QString base_topic;
  // Map from plugin name to plugin data
  QMap<QString, Info> plugins;
};

void getPluginGroups(
  const QMap<QString, QString> & datatype_plugins,
  QList<PluginGroup> * groups,
  std::vector<std::string> * unvisualizable,
  const std::string & node_name)
{
  std::map<std::string, std::vector<std::string>> topic_names_and_types =
    rviz_common::ros_integration::get_topic_names_and_types(node_name);

  for (const auto map_pair : topic_names_and_types) {
    QString topic = QString::fromStdString(map_pair.first);
    if (map_pair.second.empty()) {
      throw std::runtime_error("topic '" + map_pair.first + "' unexpectedly has not types.");
    }
    if (map_pair.second.size() > 1) {
      std::stringstream ss;
      ss << "topic '" << map_pair.first <<
        "' has more than one types associated, rviz will arbitrarily use the type '" <<
        map_pair.second[0] << "' -- all types for the topic:";
      for (const auto & topic_type_name : map_pair.second) {
        ss << " '" << topic_type_name << "'";
      }
      RVIZ_COMMON_LOG_WARNING(ss.str().c_str());
    }
    QString datatype = QString::fromStdString(map_pair.second[0]);

    if (datatype_plugins.contains(datatype)) {
      if (groups->size() == 0 ||
        !isSubtopic(groups->back().base_topic.toStdString(),
        topic.toStdString()))
      {
        PluginGroup pi;
        pi.base_topic = topic;
        groups->append(pi);
      }

      PluginGroup & group = groups->back();
      QString topic_suffix("raw");
      if (topic != group.base_topic) {
        // Remove base_topic and leading slash
        topic_suffix = topic.right(topic.size() - group.base_topic.size() - 1);
      }

      const QList<QString> & plugin_names =
        datatype_plugins.values(datatype);
      for (int i = 0; i < plugin_names.size(); ++i) {
        const QString & name = plugin_names[i];
        PluginGroup::Info & info = group.plugins[name];
        info.topic_suffixes.append(topic_suffix);
        info.datatypes.append(datatype);
      }
    } else {
      unvisualizable->push_back(map_pair.first);
    }
  }
}

// Dialog implementation
AddDisplayDialog::AddDisplayDialog(
  DisplayFactory * factory,
  const QStringList & disallowed_display_names,
  const QStringList & disallowed_class_lookup_names,
  QString * lookup_name_output,
  const std::string & node_name,
  QString * display_name_output,
  QString * topic_output,
  QString * datatype_output,
  QWidget * parent)
: QDialog(parent),
  factory_(factory),
  disallowed_display_names_(disallowed_display_names),
  disallowed_class_lookup_names_(disallowed_class_lookup_names),
  lookup_name_output_(lookup_name_output),
  display_name_output_(display_name_output),
  topic_output_(topic_output),
  datatype_output_(datatype_output)
{
  //***** Layout

  // Display Type group
  QGroupBox * type_box = new QGroupBox("Create visualization");

  QLabel * description_label = new QLabel("Description:");
  description_ = new QTextBrowser;
  description_->setMaximumHeight(100);
  description_->setOpenExternalLinks(true);

  DisplayTypeTree * display_tree = new DisplayTypeTree;
  display_tree->fillTree(factory);

  TopicDisplayWidget * topic_widget = new TopicDisplayWidget(node_name);
  topic_widget->fill(factory);

  tab_widget_ = new QTabWidget;
  display_tab_ = tab_widget_->addTab(display_tree, tr("By display type"));
  topic_tab_ = tab_widget_->addTab(topic_widget, tr("By topic"));

  QVBoxLayout * type_layout = new QVBoxLayout;
  type_layout->addWidget(tab_widget_);
  type_layout->addWidget(description_label);
  type_layout->addWidget(description_);

  type_box->setLayout(type_layout);

  // Display Name group
  QGroupBox * name_box = nullptr;
  if (display_name_output_) {
    name_box = new QGroupBox("Display Name");
    name_editor_ = new QLineEdit;
    QVBoxLayout * name_layout = new QVBoxLayout;
    name_layout->addWidget(name_editor_);
    name_box->setLayout(name_layout);
  }

  // Buttons
  button_box_ =
    new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal);

  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(type_box);
  if (display_name_output_) {
    main_layout->addWidget(name_box);
  }
  main_layout->addWidget(button_box_);
  setLayout(main_layout);

  // *INDENT-OFF* - uncrustify cannot deal with commas here
  //***** Connections
  connect(display_tree, SIGNAL(itemChanged(SelectionData *)),
    this, SLOT(onDisplaySelected(SelectionData *)));
  connect(display_tree, SIGNAL(itemActivated(QTreeWidgetItem *, int)),
    this, SLOT(accept()));

  connect(topic_widget, SIGNAL(itemChanged(SelectionData *)),
    this, SLOT(onTopicSelected(SelectionData *)));
  connect(topic_widget, SIGNAL(itemActivated(QTreeWidgetItem *, int)),
    this, SLOT(accept()));

  connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));
  connect(button_box_, SIGNAL(rejected()), this, SLOT(reject()));

  connect(tab_widget_, SIGNAL(currentChanged(int)),
    this, SLOT(onTabChanged(int)));
  if (display_name_output_) {
    connect(name_editor_, SIGNAL(textEdited(const QString&)),
      this, SLOT(onNameChanged()));
  }
  // *INDENT-ON*

  button_box_->button(QDialogButtonBox::Ok)->setEnabled(isValid());
}

QSize AddDisplayDialog::sizeHint() const
{
  return QSize(500, 660);
}

void AddDisplayDialog::onTabChanged(int index)
{
  Q_UNUSED(index);
  updateDisplay();
}

void AddDisplayDialog::onDisplaySelected(SelectionData * data)
{
  display_data_ = *data;
  updateDisplay();
}

void AddDisplayDialog::onTopicSelected(SelectionData * data)
{
  topic_data_ = *data;
  updateDisplay();
}

void AddDisplayDialog::updateDisplay()
{
  SelectionData * data = NULL;
  if (tab_widget_->currentIndex() == topic_tab_) {
    data = &topic_data_;
  } else if (tab_widget_->currentIndex() == display_tab_) {
    data = &display_data_;
  } else {
    RVIZ_COMMON_LOG_WARNING_STREAM("Unknown tab index: " << tab_widget_->currentIndex());
    return;
  }

  QString html = "<html><body>" + data->whats_this + "</body></html>";
  description_->setHtml(html);

  lookup_name_ = data->lookup_name;
  if (display_name_output_) {
    name_editor_->setText(data->display_name);
  }

  *topic_output_ = data->topic;
  *datatype_output_ = data->datatype;

  button_box_->button(QDialogButtonBox::Ok)->setEnabled(isValid());
}

bool AddDisplayDialog::isValid()
{
  if (lookup_name_.size() == 0) {
    setError("Select a Display type.");
    return false;
  }
  if (display_name_output_) {
    QString display_name = name_editor_->text();
    if (display_name.size() == 0) {
      setError("Enter a name for the display.");
      return false;
    }
    if (disallowed_display_names_.contains(display_name)) {
      setError("Name in use.  Display names must be unique.");
      return false;
    }
  }
  setError("");
  return true;
}

void AddDisplayDialog::setError(const QString & error_text)
{
  button_box_->button(QDialogButtonBox::Ok)->setToolTip(error_text);
}

void AddDisplayDialog::onNameChanged()
{
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(isValid());
}

void AddDisplayDialog::accept()
{
  if (isValid()) {
    *lookup_name_output_ = lookup_name_;
    if (display_name_output_) {
      *display_name_output_ = name_editor_->text();
    }
    QDialog::accept();
  }
}

DisplayTypeTree::DisplayTypeTree()
{
  setHeaderHidden(true);

  // *INDENT-OFF*
  connect(
    this, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)),
    this, SLOT(onCurrentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
  // *INDENT-ON*
}

void DisplayTypeTree::onCurrentItemChanged(
  QTreeWidgetItem * curr,
  QTreeWidgetItem * prev)
{
  Q_UNUSED(prev);
  // If display is selected, populate selection data.  Otherwise, clear data.
  SelectionData sd;
  if (curr->parent() != NULL) {
    // Leave topic and datatype blank
    sd.whats_this = curr->whatsThis(0);
    sd.lookup_name = curr->data(0, Qt::UserRole).toString();
    sd.display_name = curr->text(0);
  }
  Q_EMIT itemChanged(&sd);
}

void DisplayTypeTree::fillTree(Factory * factory)
{
  QIcon default_package_icon = loadPixmap("package://rviz/icons/default_package_icon.png");

  QStringList classes = factory->getDeclaredClassIds();
  classes.sort();

  // Map from package names to the corresponding top-level tree widget items.
  std::map<QString, QTreeWidgetItem *> package_items;

  for (int i = 0; i < classes.size(); i++) {
    QString lookup_name = classes[i];
    QString package = factory->getClassPackage(lookup_name);
    QString description = factory->getClassDescription(lookup_name);
    QString name = factory->getClassName(lookup_name);

    QTreeWidgetItem * package_item;

    std::map<QString, QTreeWidgetItem *>::iterator mi;
    mi = package_items.find(package);
    if (mi == package_items.end()) {
      package_item = new QTreeWidgetItem(this);
      package_item->setText(0, package);
      package_item->setIcon(0, default_package_icon);

      package_item->setExpanded(true);
      package_items[package] = package_item;
    } else {
      package_item = (*mi).second;
    }
    QTreeWidgetItem * class_item = new QTreeWidgetItem(package_item);

    class_item->setIcon(0, factory->getIcon(lookup_name));

    class_item->setText(0, name);
    class_item->setWhatsThis(0, description);
    // Store the lookup name for each class in the UserRole of the item.
    class_item->setData(0, Qt::UserRole, lookup_name);
  }
}

TopicDisplayWidget::TopicDisplayWidget(const std::string & node_name)
: node_name_(node_name)
{
  tree_ = new QTreeWidget;
  tree_->setHeaderHidden(true);
  tree_->setColumnCount(2);

  tree_->header()->setStretchLastSection(false);
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  tree_->header()->setResizeMode(0, QHeaderView::Stretch);
#else
  tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
#endif

  enable_hidden_box_ = new QCheckBox("Show unvisualizable topics");
  enable_hidden_box_->setCheckState(Qt::Unchecked);

  QVBoxLayout * layout = new QVBoxLayout;
  layout->setContentsMargins(QMargins(0, 0, 0, 0));

  layout->addWidget(tree_);
  layout->addWidget(enable_hidden_box_);

  // *INDENT-OFF* - uncrustify cannot deal with commas here
  connect(tree_, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)),
    this, SLOT(onCurrentItemChanged(QTreeWidgetItem *)));
  // Forward signals from tree_
  connect(tree_, SIGNAL(itemActivated(QTreeWidgetItem *, int)),
    this, SIGNAL(itemActivated(QTreeWidgetItem *, int)));
  // *INDENT-ON*

  // Connect signal from checkbox
  connect(enable_hidden_box_, SIGNAL(stateChanged(int)),
    this, SLOT(stateChanged(int)));

  setLayout(layout);
}

void TopicDisplayWidget::onCurrentItemChanged(QTreeWidgetItem * curr)
{
  // If plugin is selected, populate selection data.  Otherwise, clear data.
  SelectionData sd;
  if (curr->data(1, Qt::UserRole).isValid()) {
    QTreeWidgetItem * parent = curr->parent();
    sd.whats_this = curr->whatsThis(0);

    sd.topic = parent->data(0, Qt::UserRole).toString();
    sd.lookup_name = curr->data(0, Qt::UserRole).toString();
    sd.display_name = curr->text(0);

    QComboBox * combo = qobject_cast<QComboBox *>(tree_->itemWidget(curr, 1));
    if (combo != NULL) {
      QString combo_text = combo->currentText();
      if (combo_text != "raw") {
        sd.topic += "/" + combo_text;
      }
      sd.datatype = combo->itemData(combo->currentIndex()).toString();
    } else {
      sd.datatype = curr->data(1, Qt::UserRole).toString();
    }
  }
  Q_EMIT itemChanged(&sd);
}

void TopicDisplayWidget::onComboBoxClicked(QTreeWidgetItem * curr)
{
  tree_->setCurrentItem(curr);
}

void TopicDisplayWidget::stateChanged(int state)
{
  bool hide_disabled = state == Qt::Unchecked;
  QTreeWidgetItemIterator it(tree_, QTreeWidgetItemIterator::Disabled);
  for (; *it; ++it) {
    QTreeWidgetItem * item = *it;
    item->setHidden(hide_disabled);
  }
}

void TopicDisplayWidget::fill(DisplayFactory * factory)
{
  findPlugins(factory);

  QList<PluginGroup> groups;
  std::vector<std::string> unvisualizable;
  getPluginGroups(datatype_plugins_, &groups, &unvisualizable, node_name_);

  // Insert visualizable topics along with their plugins
  QList<PluginGroup>::const_iterator pg_it;
  for (pg_it = groups.begin(); pg_it < groups.end(); ++pg_it) {
    const PluginGroup & pg = *pg_it;

    QTreeWidgetItem * item = insertItem(pg.base_topic, false);
    item->setData(0, Qt::UserRole, pg.base_topic);

    QMap<QString, PluginGroup::Info>::const_iterator it;
    for (it = pg.plugins.begin(); it != pg.plugins.end(); ++it) {
      const QString plugin_name = it.key();
      const PluginGroup::Info & info = it.value();
      QTreeWidgetItem * row = new QTreeWidgetItem(item);

      row->setText(0, factory->getClassName(plugin_name));
      row->setIcon(0, factory->getIcon(plugin_name));
      row->setWhatsThis(0, factory->getClassDescription(plugin_name));
      row->setData(0, Qt::UserRole, plugin_name);
      row->setData(1, Qt::UserRole, info.datatypes[0]);

      // *INDENT-OFF* - uncrustify cannot deal with commas here
      if (info.topic_suffixes.size() > 1) {
        EmbeddableComboBox * box = new EmbeddableComboBox(row, 1);
        connect(box, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
          this, SLOT(onComboBoxClicked(QTreeWidgetItem *)));
        for (int i = 0; i < info.topic_suffixes.size(); ++i) {
          box->addItem(info.topic_suffixes[i], info.datatypes[i]);
        }
        tree_->setItemWidget(row, 1, box);
        tree_->setColumnWidth(1, std::max(tree_->columnWidth(1), box->width()));
      }
      // *INDENT-ON*
    }
  }

  // Insert unvisualizable topics
  for (const std::string & unvisualizable_topic : unvisualizable) {
    insertItem(QString::fromStdString(unvisualizable_topic), true);
  }

  // Hide unvisualizable topics if necessary
  stateChanged(enable_hidden_box_->isChecked());
}

void TopicDisplayWidget::findPlugins(DisplayFactory * factory)
{
  // Build map from topic type to plugin by instantiating every plugin we have.
  QStringList lookup_names = factory->getDeclaredClassIds();

  QStringList::iterator it;
  for (it = lookup_names.begin(); it != lookup_names.end(); ++it) {
    const QString & lookup_name = *it;
    // ROS_INFO("Class: %s", lookup_name.toStdString().c_str());

    QSet<QString> topic_types = factory->getMessageTypes(lookup_name);
    Q_FOREACH (QString topic_type, topic_types) {
      // ROS_INFO("Type: %s", topic_type.toStdString().c_str());
      datatype_plugins_.insertMulti(topic_type, lookup_name);
    }
  }
}

QTreeWidgetItem * TopicDisplayWidget::insertItem(
  const QString & topic,
  bool disabled)
{
  QTreeWidgetItem * current = tree_->invisibleRootItem();
  QStringList parts = topic.split("/");

  for (int part_ind = 1; part_ind < parts.size(); ++part_ind) {
    QString part = "/" + parts[part_ind];
    // If any child matches, use that one.
    bool match = false;
    for (int c = 0; c < current->childCount(); ++c) {
      QTreeWidgetItem * child = current->child(c);
      if (child->text(0) == part && !child->data(1, Qt::UserRole).isValid()) {
        match = true;
        current = child;
        break;
      }
    }
    // If no match, create a new child.
    if (!match) {
      QTreeWidgetItem * new_child = new QTreeWidgetItem(current);
      // Only expand first few levels of the tree
      new_child->setExpanded(3 > part_ind);
      new_child->setText(0, part);
      new_child->setDisabled(disabled);
      current = new_child;
    }
  }
  return current;
}

}  // namespace rviz_common
