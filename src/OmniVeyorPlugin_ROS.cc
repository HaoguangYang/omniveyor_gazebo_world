/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/transport/transport.hh"
#include "OmniVeyorPlugin_ROS.hh"

using namespace gazebo;

/// \brief Private data class
class gazebo::OmniVeyorPluginPrivate
{
  /// \brief Pointer to model containing plugin.
  public: physics::ModelPtr model;

  /// \brief SDF for this plugin;
  public: sdf::ElementPtr sdf;

  /// \brief Pointer to a node with robot prefix.
  public: transport::NodePtr robotNode;

  /// \brief Velocity command subscriber.
  public: transport::SubscriberPtr velocityPoseSub;

  /// \brief Velocity command subscriber.
  public: transport::SubscriberPtr velocityTwistSub;

  /// \brief Publisher of the joint velocities.
  public: transport::PublisherPtr jointVelocityPub;

  /// \brief Max axle velocity in rad/s.
  public: double maxLinearSpeed = 1.0;

  /// \brief Max steering speed in rad/s.
  public: double maxAngularSpeed = 1.0;
  
  /// \brief Max axle velocity in rad/s.
  public: double maxAxleSpeed = 100.0;

  /// \brief Max steering speed in rad/s.
  public: double maxSteerSpeed = 100.0;

  /// \brief Namespace used as a prefix for gazebo topic names.
  public: std::string robotNamespace;
};

OmniVeyorPlugin::OmniVeyorPlugin()
  : dataPtr(new OmniVeyorPluginPrivate) { }

OmniVeyorPlugin::~OmniVeyorPlugin() = default;

void OmniVeyorPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "OmniVeyorPlugin _model pointer is NULL");
  this->dataPtr->model = _model;

  GZ_ASSERT(_sdf, "OmniVeyorPlugin _sdf pointer is NULL");
  this->dataPtr->sdf = _sdf;

  // Load parameters from SDF plugin contents.
  this->LoadParam(_sdf, "robot_namespace", this->dataPtr->robotNamespace,
                  _model->GetName());
  
  this->LoadParam(_sdf, "max_linear_speed",
                  this->dataPtr->maxLinearSpeed, 1.);
  this->LoadParam(_sdf, "max_angular_speed",
                  this->dataPtr->maxAngularSpeed, 1.);

  if (this->dataPtr->maxLinearSpeed <= 0.)
    throw std::runtime_error("Maximum linear speed must be positive");
  if (this->dataPtr->maxAngularSpeed < 0.)
    throw std::runtime_error("Maximum angular speed must be non-negative");

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
        "/" + _model->GetName() + "/cmd_vel_twist",
        1,
        boost::bind(&OmniVeyorPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread =
    std::thread(std::bind(&OmniVeyorPlugin::QueueThread, this));
}

void OmniVeyorPlugin::Init()
{
  // Initialize transport nodes.

  // Prepend world name to robot namespace if it isn't absolute.
  auto robotNamespace = this->GetRobotNamespace();
  if (!robotNamespace.empty() && robotNamespace.at(0) != '/')
  {
    robotNamespace = this->dataPtr->model->GetWorld()->Name() +
      "/" + robotNamespace;
  }
  this->dataPtr->robotNode = transport::NodePtr(new transport::Node());
  this->dataPtr->robotNode->Init(robotNamespace);

  this->dataPtr->velocityTwistSub =
      this->dataPtr->robotNode->Subscribe<msgs::Twist, OmniVeyorPlugin>(
          "~/cmd_vel_twist", &OmniVeyorPlugin::OnVelMsg, this);
}

void OmniVeyorPlugin::Reset()
{
  this->SetMotorVelocity(0., 0., 0., 0., 0., 0., 0., 0.);

  ModelPlugin::Reset();
}

void OmniVeyorPlugin::SetMotorVelocity(double _strFL, double _rolFL,
                                       double _strFR, double _rolFR,
                                       double _strRL, double _rolRL,
                                       double _strRR, double _rolRR)
{
  // Apply the max track velocity limit.
  const double strFL = ignition::math::clamp(_strFL,
                                          -this->dataPtr->maxSteerSpeed,
                                          this->dataPtr->maxSteerSpeed);
  const double rolFL = ignition::math::clamp(_rolFL,
                                           -this->dataPtr->maxAxleSpeed,
                                           this->dataPtr->maxAxleSpeed);

  const double strFR = ignition::math::clamp(_strFR,
                                          -this->dataPtr->maxSteerSpeed,
                                          this->dataPtr->maxSteerSpeed);
  const double rolFR = ignition::math::clamp(_rolFR,
                                           -this->dataPtr->maxAxleSpeed,
                                           this->dataPtr->maxAxleSpeed);
                                           
  const double strRL = ignition::math::clamp(_strRL,
                                          -this->dataPtr->maxSteerSpeed,
                                          this->dataPtr->maxSteerSpeed);
  const double rolRL = ignition::math::clamp(_rolRL,
                                           -this->dataPtr->maxAxleSpeed,
                                           this->dataPtr->maxAxleSpeed);

  const double strRR = ignition::math::clamp(_strRR,
                                          -this->dataPtr->maxSteerSpeed,
                                          this->dataPtr->maxSteerSpeed);
  const double rolRR = ignition::math::clamp(_rolRR,
                                           -this->dataPtr->maxAxleSpeed,
                                           this->dataPtr->maxAxleSpeed);

  // Set motor velocities.
  this->dataPtr->model->GetJoint("FL_steer")->SetParam("vel", 0, strFL);
  this->dataPtr->model->GetJoint("FL_wheel_axle")->SetParam("vel", 0, rolFL);
  this->dataPtr->model->GetJoint("FR_steer")->SetParam("vel", 0, strFR);
  this->dataPtr->model->GetJoint("FR_wheel_axle")->SetParam("vel", 0, rolFR);
  this->dataPtr->model->GetJoint("RL_steer")->SetParam("vel", 0, strRL);
  this->dataPtr->model->GetJoint("RL_wheel_axle")->SetParam("vel", 0, rolRL);
  this->dataPtr->model->GetJoint("RR_steer")->SetParam("vel", 0, strRR);
  this->dataPtr->model->GetJoint("RR_wheel_axle")->SetParam("vel", 0, rolRR);
}

void OmniVeyorPlugin::SetBodyVelocity(
    const double _linearX, const double _linearY, const double _angular)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Compute effective linear and angular speed.
  const double linearX = ignition::math::clamp(
    _linearX,
    -this->dataPtr->maxLinearSpeed,
    this->dataPtr->maxLinearSpeed);

  const double linearY = ignition::math::clamp(
    _linearY,
    -this->dataPtr->maxLinearSpeed,
    this->dataPtr->maxLinearSpeed);

  const double angular = ignition::math::clamp(
    _angular,
    -this->dataPtr->maxAngularSpeed,
    this->dataPtr->maxAngularSpeed);
  
  // TODO: apply OmniVeyor kinematics and OS controller.

  // TODO: Call the track velocity handler (which does the actual vehicle control).
  // this->SetMotorVelocity(...);
}

void OmniVeyorPlugin::OnVelMsg(ConstTwistPtr &_msg)
{
  this->SetBodyVelocity(_msg->linear().x(), _msg->linear().y(), _msg->angular().z());
}

std::string OmniVeyorPlugin::GetRobotNamespace()
{
  return this->dataPtr->robotNamespace;
}
