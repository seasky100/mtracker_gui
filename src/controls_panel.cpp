#include <stdio.h>

#include "geometry_msgs/Twist.h"
#include "controls_panel.h"

namespace mtracker_gui
{

ControlsPanel::ControlsPanel(QWidget* parent)
  : rviz::Panel(parent),
    linear_velocity_(0.0f),
    angular_velocity_(0.0f),
    keys_active(false),
    joy_active(true)
{
  controls_checkbox_ = new QCheckBox("Controls active");

  joy_button_   = new QPushButton("Joy");
  keys_button_  = new QPushButton("Keys");
  left_button_  = new QPushButton("A");
  right_button_ = new QPushButton("D");
  up_button_    = new QPushButton("W");
  down_button_  = new QPushButton("S");

  joy_button_->setCheckable(true);
  joy_button_->setMinimumSize(60, 60);
  joy_button_->setMaximumSize(60, 60);

  keys_button_->setCheckable(true);
  keys_button_->setMinimumSize(60, 60);
  keys_button_->setMaximumSize(60, 60);

  up_button_->setMinimumSize(50, 50);
  up_button_->setMaximumSize(50, 50);

  down_button_->setMinimumSize(50, 50);
  down_button_->setMaximumSize(50, 50);

  left_button_->setMinimumSize(50, 50);
  left_button_->setMaximumSize(50, 50);

  right_button_->setMinimumSize(50, 50);
  right_button_->setMaximumSize(50, 50);

  linear_velocity_input_ = new QLineEdit("0.0");
  angular_velocity_input_ = new QLineEdit("0.0");

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QGridLayout* buttons_layout = new QGridLayout;
  buttons_layout->addItem(margin, 0, 0, 3, 1);
  buttons_layout->addWidget(joy_button_, 0, 1, Qt::AlignLeft);
  buttons_layout->addWidget(keys_button_, 0, 3, Qt::AlignRight);
  buttons_layout->addWidget(up_button_, 1, 2, Qt::AlignCenter);
  buttons_layout->addWidget(left_button_, 2, 1, Qt::AlignRight);
  buttons_layout->addWidget(down_button_, 2, 2, Qt::AlignCenter);
  buttons_layout->addWidget(right_button_, 2, 3, Qt::AlignLeft);
  buttons_layout->addItem(margin, 0, 4, 3, 1);

  QHBoxLayout* velocities_layout = new QHBoxLayout;
  velocities_layout->addWidget(new QLabel("V:"));
  velocities_layout->addWidget(linear_velocity_input_);
  velocities_layout->addWidget(new QLabel("m/s"));
  velocities_layout->addItem(new QSpacerItem(10, 1));
  velocities_layout->addWidget(new QLabel("W:"));
  velocities_layout->addWidget(angular_velocity_input_);
  velocities_layout->addWidget(new QLabel("rad/s"));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(controls_checkbox_);
  layout->addLayout(buttons_layout);
  layout->addLayout(velocities_layout);
  setLayout(layout);

  //QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  //output_timer->start( 100 );
}

void ControlsPanel::keyPressEvent(QKeyEvent * e)
{
  if (e->type() == QKeyEvent::KeyPress)
  {
    if (e->key() == Qt::Key_W)
    {
      up_button_->animateClick();
    }
  }
//  else if (e->type() == QKeyEvent::KeyRelease)
//  {
//    if (e->key() == Qt::Key_W)
//    {
//      up_button_->setDown(false);
//    }
//  }
}


//void ControlsPanel::setVel(float lin, float ang)
//{
//  linear_velocity_ = lin;
//  angular_velocity_ = ang;
//}

//void ControlsPanel::updateTopic()
//{
//  setTopic( output_topic_editor_->text() );
//}

//void ControlsPanel::setTopic( const QString& new_topic )
//{
//  if( new_topic != output_topic_ )
//  {
//    output_topic_ = new_topic;
//    if( output_topic_ == "" )
//    {
//      velocity_publisher_.shutdown();
//    }
//    else
//    {
//      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
//    }
//    Q_EMIT configChanged();
//  }

//  drive_widget_->setEnabled( output_topic_ != "" );
//}

//void ControlsPanel::sendVel()
//{
//  if( ros::ok() && velocity_publisher_ )
//  {
//    geometry_msgs::Twist msg;
//    msg.linear.x = linear_velocity_;
//    msg.linear.y = 0;
//    msg.linear.z = 0;
//    msg.angular.x = 0;
//    msg.angular.y = 0;
//    msg.angular.z = angular_velocity_;
//    velocity_publisher_.publish( msg );
//  }
//}

void ControlsPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void ControlsPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ControlsPanel, rviz::Panel)
