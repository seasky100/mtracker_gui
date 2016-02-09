#ifndef POSE_VISUAL_H
#define POSE_VISUAL_H

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape;
}

namespace mtracker_gui
{

class PoseVisual
{
public:
  PoseVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
  virtual ~PoseVisual();

  void setMessage(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setColor(float r, float g, float b, float a);

private:
  boost::shared_ptr<rviz::Shape> dot_;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
};


} // end namespace mtracker_gui

#endif // POSE_VISUAL_H
