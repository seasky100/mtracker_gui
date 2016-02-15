#ifndef REFERENCE_GENERATOR_PANEL_H
#define REFERENCE_GENERATOR_PANEL_H

#include <stdio.h>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <mtracker/Trigger.h>
#include <mtracker/PlayPause.h>

#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

namespace mtracker_gui
{

class ReferenceGeneratorPanel : public rviz::Panel
{
Q_OBJECT
public:
  ReferenceGeneratorPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void stop();
  void pause();
  void play();
  void callTrigger(bool checked);

private:
  QCheckBox* activate_checkbox_;

  QPushButton* stop_button_;
  QPushButton* pause_button_;
  QPushButton* play_button_;

  ros::NodeHandle nh_;
  ros::ServiceClient trigger_cli_;
  ros::ServiceClient play_pause_cli_;
};

} // end namespace mtracker_gui

#endif // REFERENCE_GENERATOR_PANEL_H
