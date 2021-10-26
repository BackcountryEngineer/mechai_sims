#include <mechai_sims/gazebo_ros_mechanum_control.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
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


namespace mechai_sims {
class GazeboRosMechanumControlPrivate {
  public:
    gazebo::physics::ModelPtr model_;
    gazebo_ros::Node::SharedPtr ros_node_;
    gazebo::common::Time last_update_time_;
    nav_msgs::msg::Odometry odom_;
    double update_period_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    gazebo::event::ConnectionPtr update_connection_;
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
    void UpdateOdometryWorld(const gazebo::common::Time & _current_time);
    void PublishOdometryTf(const gazebo::common::Time & _current_time);
    void SetVelocity(const double &_vel);
    void SetControlPIDs();
    void OnUpdate(const gazebo::common::UpdateInfo & _info);
};

GazeboRosMechanumControl::GazeboRosMechanumControl()
: impl_(std::make_unique<GazeboRosMechanumControlPrivate>()) {
  impl_->update_period_ = 0.025;
}

GazeboRosMechanumControl::~GazeboRosMechanumControl(){}

void GazeboRosMechanumControl::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  impl_->model_ = _model;
  
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->cmd_vel_sub_ =impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)), 
    std::bind(&GazeboRosMechanumControlPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

  impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosMechanumControlPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosMechanumControl::Reset() {}

void GazeboRosMechanumControlPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info) {
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  if (seconds_since_last_update < update_period_) {
    return;
  }
  UpdateOdometryWorld(_info.simTime);
  PublishOdometryTf(_info.simTime);
  last_update_time_ = _info.simTime;
}

void GazeboRosMechanumControlPrivate::UpdateOdometryWorld(const gazebo::common::Time & _current_time) {
  auto pose = model_->WorldPose();

  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  odom_.header.frame_id = "world";
  odom_.child_frame_id = "mechanum_chasis";
  
  odometry_pub_->publish(odom_);
}

void GazeboRosMechanumControlPrivate::PublishOdometryTf(const gazebo::common::Time & _current_time) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = "world";
  msg.child_frame_id = "mechanum_chasis";
  msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

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