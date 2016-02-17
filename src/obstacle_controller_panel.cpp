#include "obstacle_controller_panel.h"

namespace mtracker_gui
{

ObstacleControllerPanel::ObstacleControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("obstacle_controller_trigger_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(callTrigger(bool)));
}

void ObstacleControllerPanel::callTrigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      //
    }
    else {
      //
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ObstacleControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ObstacleControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ObstacleControllerPanel, rviz::Panel)
