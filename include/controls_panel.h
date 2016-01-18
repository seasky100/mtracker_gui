#ifndef CONTROLS_PANEL_H
#define CONTROLS_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <QCheckBox>
#include <QLineEdit>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

#include <stdio.h>

class QLineEdit;
class QPushButton;

namespace mtracker_gui
{

class ControlsPanel: public rviz::Panel
{
Q_OBJECT
public:
  ControlsPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

private Q_SLOTS:
  void callService();

private:
  void keyPressEvent(QKeyEvent * e);

private:
  QCheckBox* controls_checkbox_;

  QPushButton* left_button_;
  QPushButton* right_button_;
  QPushButton* up_button_;
  QPushButton* down_button_;
  QPushButton* joy_button_;
  QPushButton* keys_button_;

  QLineEdit* linear_velocity_input_;
  QLineEdit* angular_velocity_input_;

  ros::NodeHandle nh_;
  ros::ServiceClient mancontroller_trigger_cli_;
  ros::Publisher velocity_publisher_;

  float linear_velocity_;
  float angular_velocity_;
  float max_linear_velocity_;
  float max_angular_velocity_;

  bool keys_active;
  bool joy_active;
};

} // end namespace mtracker_gui

#endif // CONTROLS_PANEL_H
