#include "simulator_panel.h"

namespace mtracker_gui
{

SimulatorPanel::SimulatorPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), activate_checkbox_(false) {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("simulator_trigger_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(callTrigger(bool)));
}

void SimulatorPanel::callTrigger(bool checked) {
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

void SimulatorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void SimulatorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::SimulatorPanel, rviz::Panel)
