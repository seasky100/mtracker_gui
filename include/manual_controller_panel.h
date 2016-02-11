#ifndef MANUAL_CONTROLLER_PANEL_H
#define MANUAL_CONTROLLER_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <mtracker/Trigger.h>
#include <mtracker/ManualGains.h>

#include <QCheckBox>
#include <QLineEdit>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>

#include <stdio.h>

class QLineEdit;
class QPushButton;

namespace mtracker_gui
{

class ManualControllerPanel: public rviz::Panel
{
Q_OBJECT
public:
  ManualControllerPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void setGainV();
  void setGainW();
  void callManualGains();
  void callTrigger(bool checked);
  void callService();

private:
  void keyPressEvent(QKeyEvent * e);

private:
  QCheckBox* activate_checkbox_;

  QPushButton* left_button_;
  QPushButton* right_button_;
  QPushButton* up_button_;
  QPushButton* down_button_;
  QPushButton* joy_button_;
  QPushButton* keys_button_;

  QLineEdit* v_gain_input_;
  QLineEdit* w_gain_input_;

  ros::NodeHandle nh_;
  ros::Publisher velocity_publisher_;
  ros::ServiceClient trigger_cli_;
  ros::ServiceClient manual_gains_cli_;

  double v_gain_;
  double w_gain_;

  bool keys_active_;
  bool joy_active_;
};

} // end namespace mtracker_gui

#endif
