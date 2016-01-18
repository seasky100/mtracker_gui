#ifndef PATH_DISPLAY_H
#define PATH_DISPLAY_H

#include <boost/circular_buffer.hpp>
#include <geometry_msgs/PointStamped.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace mtracker_gui
{

class PoseVisual;

class PathDisplay: public rviz::MessageFilterDisplay<geometry_msgs::PointStamped>
{
Q_OBJECT
public:
  PathDisplay();
  virtual ~PathDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  void processMessage( const geometry_msgs::PointStamped::ConstPtr& msg );

  boost::circular_buffer<boost::shared_ptr<PoseVisual> > visuals_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty* history_length_property_;
};

} // namespace mtracker_gui

#endif // PATH_DISPLAY_H
