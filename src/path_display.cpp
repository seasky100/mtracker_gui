#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "pose_visual.h"
#include "path_display.h"

namespace mtracker_gui
{

PathDisplay::PathDisplay()
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 100000 );
}

void PathDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

PathDisplay::~PathDisplay()
{
}

void PathDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void PathDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setColor( color.r, color.g, color.b, alpha );
  }
}

void PathDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

void PathDisplay::processMessage( const geometry_msgs::PointStamped::ConstPtr& msg )
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  boost::shared_ptr<PoseVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new PoseVisual( context_->getSceneManager(), scene_node_ ));
  }

  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor( color.r, color.g, color.b, alpha );

  visuals_.push_back(visual);

  for (uint i = 0; i < visuals_.size(); ++i) {
    visuals_[i]->setColor( color.r, color.g, color.b, (float)i /visuals_.size() );
  }
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::PathDisplay, rviz::Display)
