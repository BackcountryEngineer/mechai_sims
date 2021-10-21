#include <gazebo_plugin_tutorials/gazebo_ros_mechanum_control.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>


namespace gazebo_plugin_tutorials {
class GazeboRosMechanumControlPrivate {
  public:
    gazebo::physics::ModelPtr model_;
    gazebo_ros::Node::SharedPtr ros_node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    gazebo::event::ConnectionPtr update_connection_;
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
    void SetVelocity(const double &_vel);
    void SetControlPIDs();
    // void OnUpdate(const gazebo::common::UpdateInfo & _info);
};

GazeboRosMechanumControl::GazeboRosMechanumControl()
: impl_(std::make_unique<GazeboRosMechanumControlPrivate>()) {}

GazeboRosMechanumControl::~GazeboRosMechanumControl(){}

void GazeboRosMechanumControl::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  impl_->model_ = _model;

  std::cout<<"sdf is"<<std::endl;
  
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->cmd_vel_sub_ =impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)), 
    std::bind(&GazeboRosMechanumControlPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

  impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

  // Listen to the update event (broadcast every simulation iteration)
  // impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
  //   std::bind(&GazeboRosDiffDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosMechanumControl::Reset() {}

// void GazeboRosMechanumControlPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info) {

// }

void GazeboRosMechanumControlPrivate::SetControlPIDs() {
  auto pid = gazebo::common::PID(1, 0, 0);
  auto jointController = model_->GetJointController();
  for (auto joint : model_->GetJoints()) {
    jointController->SetVelocityPID(joint->GetScopedName(), pid);
  }
}

void GazeboRosMechanumControlPrivate::SetVelocity(const double &_vel) {
  auto jointController = model_->GetJointController();
  for (auto joint : model_->GetJoints()) {
    jointController->SetVelocityTarget(joint->GetScopedName(), _vel);
  }
}

void GazeboRosMechanumControlPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg) {
  SetVelocity(_msg->linear.x);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMechanumControl)
}