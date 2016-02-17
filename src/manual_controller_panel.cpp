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

#include "manual_controller_panel.h"

namespace mtracker_gui
{

ManualControllerPanel::ManualControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("manual_controller_trigger_srv");
  manual_gains_cli_ = nh_.serviceClient<mtracker::ManualGains>("manual_gains_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  joy_button_   = new QPushButton("Joy");
  keys_button_  = new QPushButton("Keys");
  set_button_   = new QPushButton("Set");
  left_button_  = new QPushButton("A");
  right_button_ = new QPushButton("D");
  up_button_    = new QPushButton("W");
  down_button_  = new QPushButton("S");

  joy_button_->setCheckable(true);
  joy_button_->setMinimumSize(60, 60);
  joy_button_->setMaximumSize(60, 60);
  joy_button_->setEnabled(false);

  keys_button_->setCheckable(true);
  keys_button_->setMinimumSize(60, 60);
  keys_button_->setMaximumSize(60, 60);
  keys_button_->setEnabled(false);

  set_button_->setEnabled(false);

  up_button_->setMinimumSize(50, 50);
  up_button_->setMaximumSize(50, 50);
  up_button_->setEnabled(false);

  down_button_->setMinimumSize(50, 50);
  down_button_->setMaximumSize(50, 50);
  down_button_->setEnabled(false);

  left_button_->setMinimumSize(50, 50);
  left_button_->setMaximumSize(50, 50);
  left_button_->setEnabled(false);

  right_button_->setMinimumSize(50, 50);
  right_button_->setMaximumSize(50, 50);
  right_button_->setEnabled(false);

  k_v_input_ = new QLineEdit("0.4");
  k_v_input_->setEnabled(false);

  k_w_input_ = new QLineEdit("1.5");
  k_w_input_->setEnabled(false);

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QGridLayout* buttons_layout = new QGridLayout;
  buttons_layout->addItem(margin, 0, 0, 3, 1);
  buttons_layout->addWidget(joy_button_, 0, 1, Qt::AlignLeft);
  buttons_layout->addWidget(keys_button_, 0, 3, Qt::AlignRight);
  buttons_layout->addWidget(up_button_, 1, 2, Qt::AlignCenter);
  buttons_layout->addWidget(left_button_, 2, 1, Qt::AlignRight);
  buttons_layout->addWidget(down_button_, 2, 2, Qt::AlignCenter);
  buttons_layout->addWidget(right_button_, 2, 3, Qt::AlignLeft);
  buttons_layout->addItem(margin, 0, 4, 3, 1);

  QHBoxLayout* gains_layout = new QHBoxLayout;
  gains_layout->addWidget(new QLabel("k_v:"));
  gains_layout->addWidget(k_v_input_);
  gains_layout->addWidget(new QLabel("m/s"));
  gains_layout->addItem(new QSpacerItem(10, 1));
  gains_layout->addWidget(new QLabel("k_w:"));
  gains_layout->addWidget(k_w_input_);
  gains_layout->addWidget(new QLabel("rad/s"));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(buttons_layout);
  layout->addLayout(gains_layout);
  layout->addWidget(set_button_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(updateGains()));
}

void ManualControllerPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      joy_button_->setEnabled(true);
      keys_button_->setEnabled(true);
      set_button_->setEnabled(true);
      up_button_->setEnabled(true);
      down_button_->setEnabled(true);
      left_button_->setEnabled(true);
      right_button_->setEnabled(true);
      k_v_input_->setEnabled(true);
      k_w_input_->setEnabled(true);
    }
    else {
      joy_button_->setEnabled(false);
      keys_button_->setEnabled(false);
      set_button_->setEnabled(false);
      up_button_->setEnabled(false);
      down_button_->setEnabled(false);
      left_button_->setEnabled(false);
      right_button_->setEnabled(false);
      k_v_input_->setEnabled(false);
      k_w_input_->setEnabled(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ManualControllerPanel::updateGains() {
  mtracker::ManualGains gains;

  try { gains.request.k_v = boost::lexical_cast<double>(k_v_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { k_v_input_->setText(":-("); return; }

  try { gains.request.k_w = boost::lexical_cast<double>(k_w_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { k_w_input_->setText(":-("); return; }

  manual_gains_cli_.call(gains);
}

void ManualControllerPanel::keyPressEvent(QKeyEvent * e) {
  if (e->type() == QKeyEvent::KeyPress)
  {
    if (e->key() == Qt::Key_W)
    {
      up_button_->animateClick();
    }
  }
//  else if (e->type() == QKeyEvent::KeyRelease)
//  {
//    if (e->key() == Qt::Key_W)
//    {
//      up_button_->setDown(false);
//    }
//  }
}


void ManualControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ManualControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ManualControllerPanel, rviz::Panel)
