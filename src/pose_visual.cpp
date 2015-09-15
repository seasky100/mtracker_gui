#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>

#include "pose_visual.h"

namespace mtracker_gui
{

PoseVisual::PoseVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  orientation_arrow_.reset(new rviz::Arrow(scene_manager_, frame_node_));
}

PoseVisual::~PoseVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void PoseVisual::setMessage(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  orientation_arrow_->setScale(Ogre::Vector3(1.0, 1.0, 1.0));
  orientation_arrow_->setPosition(Ogre::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0));
  orientation_arrow_->setDirection(Ogre::Vector3(1.0, 0.0, 0.0));
}

void PoseVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void PoseVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void PoseVisual::setColor(float r, float g, float b, float a)
{
  orientation_arrow_->setColor(r, g, b, a);
}

} // end namespace mtracker_gui
