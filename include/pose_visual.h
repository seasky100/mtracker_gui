#ifndef POSE_VISUAL_H
#define POSE_VISUAL_H

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/shape.h>
#include <geometry_msgs/Pose2D.h>

namespace mtracker_gui
{

class PoseVisual
{
public:
  PoseVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  virtual ~PoseVisual();

  void setMessage(const geometry_msgs::Pose2D::ConstPtr& msg);
  void setColor(float r, float g, float b, float a);

private:
  boost::shared_ptr<rviz::Shape> path_dot_;
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
};

} // end namespace mtracker_gui

#endif // POSE_VISUAL_H
