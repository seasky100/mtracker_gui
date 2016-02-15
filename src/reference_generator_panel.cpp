#include "reference_generator_panel.h"

namespace mtracker_gui
{

ReferenceGeneratorPanel::ReferenceGeneratorPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), activate_checkbox_(false) {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("reference_generator_trigger_srv");
  play_pause_cli_ = nh_.serviceClient<mtracker::PlayPause>("reference_play_pause_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  stop_button_  = new QPushButton();
  pause_button_ = new QPushButton();
  play_button_  = new QPushButton();

  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setEnabled(false);
  stop_button_->setIcon(QIcon("/home/tysik/workspace/catkin/src/mtracker_gui/resources/stop.png"));
  stop_button_->setIconSize(QSize(25, 25));

  pause_button_->setMinimumSize(50, 50);
  pause_button_->setMaximumSize(50, 50);
  pause_button_->setEnabled(false);
  pause_button_->setIcon(QIcon("/home/tysik/workspace/catkin/src/mtracker_gui/resources/pause.png"));
  pause_button_->setIconSize(QSize(25, 25));

  play_button_->setMinimumSize(50, 50);
  play_button_->setMaximumSize(50, 50);
  play_button_->setEnabled(false);
  play_button_->setIcon(QIcon("/home/tysik/workspace/catkin/src/mtracker_gui/resources/play.png"));
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

  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(pause()));
  connect(play_button_, SIGNAL(clicked()), this, SLOT(play()));
  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(callTrigger(bool)));
}

void ReferenceGeneratorPanel::stop() {
  mtracker::PlayPause msg;
  msg.request.play = false;
  msg.request.pause = false;

  if (play_pause_cli_.call(msg)) {
    stop_button_->setEnabled(false);
    pause_button_->setEnabled(true);
    play_button_->setEnabled(true);
  }
}

void ReferenceGeneratorPanel::pause() {
  mtracker::PlayPause msg;
  msg.request.play = false;
  msg.request.pause = true;

  if (play_pause_cli_.call(msg)) {
    stop_button_->setEnabled(true);
    pause_button_->setEnabled(false);
    play_button_->setEnabled(true);
  }
}

void ReferenceGeneratorPanel::play() {
  mtracker::PlayPause msg;
  msg.request.play = true;
  msg.request.pause = false;

  if (play_pause_cli_.call(msg)) {
    stop_button_->setEnabled(true);
    pause_button_->setEnabled(true);
    play_button_->setEnabled(false);
  }
}

void ReferenceGeneratorPanel::callTrigger(bool checked) {
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

void ReferenceGeneratorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ReferenceGeneratorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ReferenceGeneratorPanel, rviz::Panel)
