#include "controls_scaling_panel.h"

namespace mtracker_gui
{

ControlsScalingPanel::ControlsScalingPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), max_wheel_rate_(10.0) {
  max_wheel_rate_cli_ = nh_.serviceClient<mtracker::MaxWheelRate>("max_wheel_rate_srv");
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("controls_scaling_trigger_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  set_button_ = new QPushButton("Set");
  set_button_->setDisabled(true);

  max_wheel_rate_input_ = new QLineEdit("10.0");
  max_wheel_rate_input_->setAlignment(Qt::AlignRight);
  max_wheel_rate_input_->setDisabled(true);

  QHBoxLayout* h_layout = new QHBoxLayout;
  h_layout->addWidget(new QLabel("Max wheel rate:"));
  h_layout->addWidget(max_wheel_rate_input_);
  h_layout->addWidget(new QLabel("rad/s"));
  h_layout->addWidget(set_button_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(h_layout);
  setLayout(layout);

  connect(max_wheel_rate_input_, SIGNAL(editingFinished()), this, SLOT(setMaxWheelRate()));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(callMaxWheelRate()));
  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(callTrigger(bool)));
}

void ControlsScalingPanel::setMaxWheelRate() {
  try {
    max_wheel_rate_ = boost::lexical_cast<double>(max_wheel_rate_input_->text().toStdString());
  }
  catch(boost::bad_lexical_cast &) {
    max_wheel_rate_ = 0.0f;
    max_wheel_rate_input_->setText("0.0");
  }
}

void ControlsScalingPanel::callMaxWheelRate() {
  mtracker::MaxWheelRate msg;
  msg.request.max_wheel_rate = max_wheel_rate_;
  max_wheel_rate_cli_.call(msg);
}

void ControlsScalingPanel::callTrigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      set_button_->setEnabled(true);
      max_wheel_rate_input_->setEnabled(true);
    }
    else {
      set_button_->setEnabled(false);
      max_wheel_rate_input_->setEnabled(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ControlsScalingPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ControlsScalingPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ControlsScalingPanel, rviz::Panel)
