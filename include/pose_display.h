#ifndef POSE_DISPLAY_H
#define POSE_DISPLAY_H

#include <boost/circular_buffer.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <rviz/message_filter_display.h>

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace mtracker_gui
{

class PoseVisual;

class PoseDisplay : public rviz::MessageFilterDisplay<geometry_msgs::PoseStamped>
{
Q_OBJECT
public:
  PoseDisplay();
  virtual ~PoseDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  void processMessage(const geometry_msgs::PoseStamped::ConstPtr& msg);
  boost::circular_buffer<boost::shared_ptr <PoseVisual> > visuals_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty*   history_length_property_;
};

} // end namespace mtracker_gui

#endif // POSE_DISPLAY_H
