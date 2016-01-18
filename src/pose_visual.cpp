#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>

#include "pose_visual.h"

namespace mtracker_gui
{

PoseVisual::PoseVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  dot_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_ ));
}

PoseVisual::~PoseVisual()
{
  scene_manager_->destroySceneNode( frame_node_ );
}

void PoseVisual::setMessage( const geometry_msgs::PointStamped::ConstPtr& msg )
{
//  geometry_msgs::Vector3 asd;
//  asd.x = 0.05; asd.y = 0.05; asd.z = 0.05;
//  const geometry_msgs::Vector3& a = asd;

//  Ogre::Vector3 acc( a.x, a.y, a.z );

//  float length = acc.length();

  Ogre::Vector3 scale(0.05, 0.05, 0.05);
  dot_->setScale(scale);
}

void PoseVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void PoseVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

void PoseVisual::setColor( float r, float g, float b, float a )
{
  dot_->setColor( r, g, b, a );
}

} // end namespace mtracker_gui

