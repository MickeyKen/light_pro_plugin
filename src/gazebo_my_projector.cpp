#include <algorithm>
#include <assert.h>
#include <utility>
#include <sstream>

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/RTShaderSystem.hh>
#include <gazebo_plugins/gazebo_ros_projector.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <Ogre.h>
#include <OgreMath.h>
#include <OgreSceneNode.h>
#include <OgreFrustum.h>
#include <OgreSceneQuery.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboMyProjector);

typedef std::map<std::string,Ogre::Pass*> OgrePassMap;
typedef OgrePassMap::iterator OgrePassMapIterator;


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboMyProjector::GazeboMyProjector()
{
  this->rosnode_ = NULL;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboMyProjector::~GazeboMyProjector()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboMyProjector::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->world_ = _parent->GetWorld();




  // Create a new transport node for talking to the projector
  this->node_.reset(new transport::Node());
  // Initialize the node with the world name
#if GAZEBO_MAJOR_VERSION >= 8
  this->node_->Init(this->world_->Name());
#else
  this->node_->Init(this->world_->GetName());
#endif
  // Setting projector topic
  std::string name = std::string("~/") + _parent->GetName() + "/" +
                      _sdf->Get<std::string>("projector");
  // Create a publisher on the ~/physics topic
  this->projector_pub_ = node_->Advertise<msgs::Projector>(name);



  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  this->texture_topic_name_ = "";
  if (_sdf->HasElement("textureTopicName"))
    this->texture_topic_name_ = _sdf->GetElement("textureTopicName")->Get<std::string>();

  this->projector_topic_name_ = "";
  if (_sdf->HasElement("projectorTopicName"))
    this->projector_topic_name_ = _sdf->GetElement("projectorTopicName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("projector", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }


  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);


  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int32>(
    this->projector_topic_name_,1,
    boost::bind( &GazeboRosProjector::ToggleProjector,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->projectorSubscriber_ = this->rosnode_->subscribe(so);

  ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::String>(
    this->texture_topic_name_,1,
    boost::bind( &GazeboRosProjector::LoadImage,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->imageSubscriber_ = this->rosnode_->subscribe(so2);


  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosProjector::QueueThread,this ) );

}


////////////////////////////////////////////////////////////////////////////////
// Load a texture into the projector
void GazeboMyProjector::LoadImage(const std_msgs::String::ConstPtr& imageMsg)
{
  msgs::Projector msg;
  msg.set_name("texture_projector");
  msg.set_texture(imageMsg->data);
  this->projector_pub_->Publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
// Toggle the activation of the projector
void GazeboMyProjector::ToggleProjector(const std_msgs::Int32::ConstPtr& projectorMsg)
{
  msgs::Projector msg;
  msg.set_name("texture_projector");
  msg.set_enabled(projectorMsg->data);
  this->projector_pub_->Publish(msg);
}


////////////////////////////////////////////////////////////////////////////////
// Custom callback queue thread
void GazeboMyProjector::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
