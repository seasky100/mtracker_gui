#ifndef CONTROLS_PANEL_H
#define CONTROLS_PANEL_H

#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <mtracker/MaxWheelRate.h>
#include <mtracker/Trigger.h>

#include <QCheckBox>
#include <QLineEdit>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

class QLineEdit;
class QPushButton;

namespace mtracker_gui
{

class ControlsScalingPanel : public rviz::Panel
{
Q_OBJECT
public:
  ControlsScalingPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void setMaxWheelRate();
  void callMaxWheelRate();
  void callTrigger(bool checked);

private:
  QCheckBox* activate_checkbox_;
  QPushButton* set_button_;
  QLineEdit* max_wheel_rate_input_;

  ros::NodeHandle nh_;
  ros::ServiceClient max_wheel_rate_cli_;
  ros::ServiceClient trigger_cli_;

  double max_wheel_rate_;
};

} // end namespace mtracker_gui

#endif // CONTROLS_PANEL_H
