#include "automatic_controller_panel.h"

namespace mtracker_gui
{

AutomaticControllerPanel::AutomaticControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), activate_checkbox_(false) {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("automatic_controller_trigger_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(callTrigger(bool)));
}

void AutomaticControllerPanel::callTrigger(bool checked) {
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

void AutomaticControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void AutomaticControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::AutomaticControllerPanel, rviz::Panel)
