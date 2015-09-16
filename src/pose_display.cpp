#include "pose_display.h"

namespace mtracker_gui
{

PoseDisplay::PoseDisplay() : Display()
{
  topic_property_ = new rviz::RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<geometry_msgs::Pose2D>() ),
                                          "geometry_msgs::Pose2D topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the pose arrows.",
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

PoseDisplay::~PoseDisplay()
{
}

void PoseDisplay::onInitialize()
{
  updateHistoryLength();
}

void PoseDisplay::reset()
{
  visuals_.clear();
}

void PoseDisplay::updateTopic()
{
  // ???
}

void PoseDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
  }
}

void PoseDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

void PoseDisplay::processMessage(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  boost::shared_ptr<PoseVisual> visual;
  if (visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new PoseVisual(context_->getSceneManager(), scene_node_));
  }

  visual->setMessage(msg);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visuals_.push_back(visual);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::PoseDisplay, rviz::Display)
