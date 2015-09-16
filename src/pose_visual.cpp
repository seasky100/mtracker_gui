#include "pose_visual.h"

namespace mtracker_gui
{

PoseVisual::PoseVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  path_dot_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
}

PoseVisual::~PoseVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void PoseVisual::setMessage(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  path_dot_->setScale(Ogre::Vector3(1.0, 1.0, 1.0));
  path_dot_->setPosition(Ogre::Vector3(msg->x, msg->y, 0.0));
}

void PoseVisual::setColor(float r, float g, float b, float a)
{
  path_dot_->setColor(r, g, b, a);
}

} // end namespace mtracker_gui
