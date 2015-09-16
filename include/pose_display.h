#ifndef POSE_DISPLAY_H
#define POSE_DISPLAY_H

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/display.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <geometry_msgs/Pose2D.h>

#include <boost/circular_buffer.hpp>
#include "pose_visual.h"

namespace mtracker_gui
{

class PoseDisplay : public rviz::Display
{
Q_OBJECT
public:
  PoseDisplay();
  virtual ~PoseDisplay();
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateTopic();
  void updateColorAndAlpha();
  void updateHistoryLength();

 private:
  void processMessage(const geometry_msgs::Pose2D::ConstPtr& msg);

  boost::circular_buffer<boost::shared_ptr <PoseVisual> > visuals_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty*   history_length_property_;
  rviz::RosTopicProperty* topic_property_;
};

} // end namespace mtracker_gui

#endif // POSE_DISPLAY_H
