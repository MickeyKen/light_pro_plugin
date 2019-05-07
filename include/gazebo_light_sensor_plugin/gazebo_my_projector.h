#ifndef GAZEBO_MY_PROJECTOR_HH
#define GAZEBO_MY_PROJECTOR_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/RenderTypes.hh>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <OgrePrerequisites.h>
#include <OgreTexture.h>
#include <OgreFrameListener.h>

namespace Ogre
{
  class PlaneBoundedVolumeListSceneQuery;
  class Frustum;
  class Pass;
  class SceneNode;
}

namespace gazebo
{


class GazeboRosProjector : public ModelPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model
  public: GazeboMyProjector();

  /// \brief Destructor
  public: virtual ~GazeboMyProjector();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief pointer to the world
  private: physics::WorldPtr world_;

  /// \brief Callback when a texture is published
  private: void LoadImage(const std_msgs::String::ConstPtr& imageMsg);

  /// \brief Callbakc when a projector toggle is published
  private: void ToggleProjector(const std_msgs::Int32::ConstPtr& projectorMsg);

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber imageSubscriber_;
  private: ros::Subscriber projectorSubscriber_;

 /// \brief ROS texture topic name
  private: std::string texture_topic_name_;

  /// \brief ROS projector topic name
  private: std::string projector_topic_name_;

  /// \brief For setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

  private: event::ConnectionPtr add_model_event_;

  private: transport::NodePtr node_;
  private: transport::PublisherPtr projector_pub_;
};


}
#endif

