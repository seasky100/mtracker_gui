/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
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

/*
 * Author: Mateusz Przybyla
 */

#include "reference_generator_panel.h"

namespace mtracker_gui
{

ReferenceGeneratorPanel::ReferenceGeneratorPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("reference_generator_trigger_srv");
  params_cli_ = nh_.serviceClient<mtracker::Params>("reference_generator_params_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  stop_button_  = new QPushButton();
  pause_button_ = new QPushButton();
  play_button_  = new QPushButton();

  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setEnabled(false);
  stop_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/stop.png"));
  stop_button_->setIconSize(QSize(25, 25));

  pause_button_->setMinimumSize(50, 50);
  pause_button_->setMaximumSize(50, 50);
  pause_button_->setEnabled(false);
  pause_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/pause.png"));
  pause_button_->setIconSize(QSize(25, 25));

  play_button_->setMinimumSize(50, 50);
  play_button_->setMaximumSize(50, 50);
  play_button_->setEnabled(false);
  play_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/play.png"));
  play_button_->setIconSize(QSize(25, 25));

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addSpacerItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(pause_button_);
  buttons_layout->addWidget(play_button_);
  buttons_layout->addSpacerItem(margin);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(buttons_layout);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(pause()));
  connect(play_button_, SIGNAL(clicked()), this, SLOT(play()));
}

void ReferenceGeneratorPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      stop_button_->setEnabled(true);
      pause_button_->setEnabled(true);
      play_button_->setEnabled(true);
    }
    else {
      stop_button_->setEnabled(false);
      pause_button_->setEnabled(false);
      play_button_->setEnabled(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ReferenceGeneratorPanel::stop() {
  mtracker::Params params;
  params.request.params.resize(2);
  params.request.params[0] = 0.0; // start = false
  params.request.params[1] = 0.0; // pause = false

  if (params_cli_.call(params)) {
    stop_button_->setEnabled(false);
    pause_button_->setEnabled(true);
    play_button_->setEnabled(true);
  }
}

void ReferenceGeneratorPanel::pause() {
  mtracker::Params params;
  params.request.params.resize(2);
  params.request.params[0] = 0.0;
  params.request.params[1] = 1.0;

  if (params_cli_.call(params)) {
    stop_button_->setEnabled(true);
    pause_button_->setEnabled(false);
    play_button_->setEnabled(true);
  }
}

void ReferenceGeneratorPanel::play() {
  mtracker::Params params;
  params.request.params.resize(2);
  params.request.params[0] = 1.0;
  params.request.params[1] = 0.0;


  if (params_cli_.call(params)) {
    stop_button_->setEnabled(true);
    pause_button_->setEnabled(true);
    play_button_->setEnabled(false);
  }
}

void ReferenceGeneratorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ReferenceGeneratorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ReferenceGeneratorPanel, rviz::Panel)
