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
#include "omniveyor_gazebo_world/OmniVeyorPlugin_ROS.hh"

#if BOOST_VERSION < 107400
namespace std {
template<class T>
class hash<boost::shared_ptr<T>> {
  public: size_t operator()(const boost::shared_ptr<T>& key) const {
    return (size_t)key.get();
  }
};
}
#endif

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(OmniVeyorPlugin)

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
  public: double maxAxleSpeed = 31.4;

  /// \brief Max steering speed in rad/s.
  public: double maxSteerSpeed = 31.4;

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

  this->modelName = this->dataPtr->robotNamespace;
  this->worldName = _model->GetWorld()->Name();

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
        "cmd_vel",
        1,
        boost::bind(&OmniVeyorPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->cmdVelSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread =
    std::thread(std::bind(&OmniVeyorPlugin::QueueThread, this));
}

void OmniVeyorPlugin::Init()
{
  // Initialize transport nodes.

  // Prepend world name to robot namespace if it isn't absolute.
  auto robotNamespace = this->GetRobotNamespace();

  this->dataPtr->robotNode = transport::NodePtr(new transport::Node());
  if (!robotNamespace.empty() && robotNamespace.at(0) != '/')
  {
    robotNamespace = this->worldName + "/" + robotNamespace;
  }
  this->dataPtr->robotNode->Init(robotNamespace);

  this->dataPtr->velocityTwistSub =
      this->dataPtr->robotNode->Subscribe<msgs::Twist, OmniVeyorPlugin>(
          "~/cmd_vel_twist", &OmniVeyorPlugin::OnVelMsg, this);
  
  // Joint velocity using joint motors
  this->dataPtr->model->GetJoint("FL_steer")->SetParam("fmax", 0, 100.0);
  this->dataPtr->model->GetJoint("FL_wheel_axle")->SetParam("fmax", 0, 500.0);
  this->dataPtr->model->GetJoint("FR_steer")->SetParam("fmax", 0, 100.0);
  this->dataPtr->model->GetJoint("FR_wheel_axle")->SetParam("fmax", 0, 500.0);
  this->dataPtr->model->GetJoint("RL_steer")->SetParam("fmax", 0, 100.0);
  this->dataPtr->model->GetJoint("RL_wheel_axle")->SetParam("fmax", 0, 500.0);
  this->dataPtr->model->GetJoint("RR_steer")->SetParam("fmax", 0, 100.0);
  this->dataPtr->model->GetJoint("RR_wheel_axle")->SetParam("fmax", 0, 500.0);

  Reset();
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

  Eigen::Vector3d xd_com_in;
  xd_com_in << _linearX, _linearY, _angular;
  _gxd_com = clampVelocity(xd_com_in);
}

void OmniVeyorPlugin::OnVelMsg(ConstTwistPtr &_msg)
{
  this->SetBodyVelocity(_msg->linear().x(), _msg->linear().y(), _msg->angular().z());
}

std::string OmniVeyorPlugin::GetRobotNamespace()
{
  return this->dataPtr->robotNamespace;
}

/*---------- Below functions adapted from OmniVeyor control source code ----------*/

void OmniVeyorPlugin::controlUpdate(){
  // TODO: apply OmniVeyor kinematics and OS controller.
  // This function needs to be run every 0.01s (CONTROL_PERIOD_s)
  // Retrieve latest joint data
	updateJointData();

  // odometry
  // Calculate C_p^+
	updateConstraintPinvMatrix();

	// Find velocity and position of the base in the frame of the base with respect to the world origin
	// _xd_local = _C_pinv * deltaQ / CONTROL_PERIOD_s;
	_xd_local = _C_pinv * _qd;
	_x_local += _xd_local * CONTROL_PERIOD_s;

	_heading = _gx(2)+ 0.5*_xd_local(2)*CONTROL_PERIOD_s;

	// Rotation matrix to convert vectors from local frame to global frame 
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	R(0, 0) =  cos(_heading);
  R(0, 1) = -sin(_heading);
  R(1, 0) =  sin(_heading);
  R(1, 1) =  cos(_heading);
  R(2, 2) =  1.0;
  
  _gxd = R * _xd_local;
  _gx += _gxd * CONTROL_PERIOD_s;

  // publishes odometry
  // ROS Code Goes Here.
  current_time = ros::Time::now();
  if ((current_time - odom.header.stamp).toSec()>=0.02){
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_gx(2));
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = _gx(0);//*cos_PI_4 + gx(1)*sin_PI_4;
    odom.pose.pose.position.y = _gx(1);//*sin_PI_4 + gx(1)*cos_PI_4;
    odom.pose.pose.orientation = odom_quat;
    
    odom.twist.twist.linear.x = _gxd(0);
    odom.twist.twist.linear.y = _gxd(1);
    odom.twist.twist.angular.z = _gxd(2);
    
    odomPub.publish(odom);
  }
  // this->rosPub.publish();
  
  // control
	// Calculate C
	updateConstraintMatrix();

  // Store velocity command and determine current xd_des (limit acceleration)
	_gxd_des = _gxd_des + clampAcceleration(_gxd_com - _gxd_des);

  // Calculate joint velocities to command
  _qd_des = _C * _gxd_des;
  
  // TODO: Call the track velocity handler (which does the actual vehicle control).
  this->SetMotorVelocity(_qd_des(2), _qd_des(3),
                         _qd_des(0), _qd_des(1),
                         _qd_des(4), _qd_des(5),
                         _qd_des(6), _qd_des(7));
}

/* Update joint data */
void OmniVeyorPlugin::updateJointData() 
{
  _q_steer(0) = this->dataPtr->model->GetJoint("FR_steer")->Position(0);
  _q_steer(1) = this->dataPtr->model->GetJoint("FL_steer")->Position(0);
  _q_steer(2) = this->dataPtr->model->GetJoint("RL_steer")->Position(0);
  _q_steer(3) = this->dataPtr->model->GetJoint("RR_steer")->Position(0);
  _qd(0) = this->dataPtr->model->GetJoint("FR_steer")->GetVelocity(0);
  _qd(1) = this->dataPtr->model->GetJoint("FR_wheel_axle")->GetVelocity(0);
  _qd(2) = this->dataPtr->model->GetJoint("FL_steer")->GetVelocity(0);
  _qd(3) = this->dataPtr->model->GetJoint("FL_wheel_axle")->GetVelocity(0);
  _qd(4) = this->dataPtr->model->GetJoint("RL_steer")->GetVelocity(0);
  _qd(5) = this->dataPtr->model->GetJoint("RL_wheel_axle")->GetVelocity(0);
  _qd(6) = this->dataPtr->model->GetJoint("RR_steer")->GetVelocity(0);
  _qd(7) = this->dataPtr->model->GetJoint("RR_wheel_axle")->GetVelocity(0);
}

/* Update Moore-Penrose pseudoinverse of the constraint matrix C_p. Assumes constraint matrix, C#, is up to date */
void OmniVeyorPlugin::updateConstraintPinvMatrix() 
{
	// Update Cp matrix with current data
	updateCpMatrix(); 

	Eigen::MatrixXd CptCli = Eigen::MatrixXd::Zero(3, NUM_MOTORS);

	for(int i = 0; i < NUM_CASTERS; i++)
	{
		// Cli IS 2x2 BLOCK DIAGONAL, SO MULTIPLY
	    // IN PIECES TO AVOID SLOW NxN MATRIX OPERATIONS

		// Take sin/cos of steering angle
		double steerSin = sin(_q_steer(i));
		double steerCos = cos(_q_steer(i));

		// Caster position values
		double hx = casterPosXSign(i+1) * DIST_TO_CASTER_X;
		double hy = casterPosYSign(i+1) * DIST_TO_CASTER_Y;

	    int j = 2*i;

	    // Note: sign of p-dot is: wheel as viewed from ground
	    CptCli(0, j ) =  PC_b * steerSin;
	    CptCli(0,j+1) =  PC_r * steerCos;
	    CptCli(1, j ) = -PC_b * steerCos;
	    CptCli(1,j+1) =  PC_r * steerSin;
	    CptCli(2, j ) = -PC_b * ((hx * steerCos + hy * steerSin) + PC_b);
	    CptCli(2,j+1) =  PC_r * (hx * steerSin - hy * steerCos);
	}

	_C_pinv = (_Cp.transpose()*_Cp).llt().solve(CptCli);
}

/* Update Cp matrix. Transforms x_d's into p_d's (p_d = C_p*x_d) */
void OmniVeyorPlugin::updateCpMatrix() 
{
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		// Which rows to calculate
		int steerRow = 2*i;
		int rollRow = 2*i + 1;

		// Take sin/cos of steering angle
		double steerSin = sin(_q_steer(i));
		double steerCos = cos(_q_steer(i));

		// Caster position values
		double hx = casterPosXSign(i+1) * DIST_TO_CASTER_X;
		double hy = casterPosYSign(i+1) * DIST_TO_CASTER_Y;

		// Cp = Jq*C, could calculate it this way instead
		// Compute steer row
		_Cp(steerRow,0) = 1.0;
		_Cp(steerRow,1) = 0.0;
		_Cp(steerRow,2) = -PC_b * steerSin - hy;

		// Compute roll row
		_Cp(rollRow,0) = 0.0;
		_Cp(rollRow,1) = 1.0;
		_Cp(rollRow,2) = PC_b * steerCos + hx;
	}
}

/* Update kinematic constraint matrix, C#. Assumes joint data is up to date.
   Converts from operational space (xdot) to joint space (qdot). See pg 27 PCV_Dyanmics.pdf*/
void OmniVeyorPlugin::updateConstraintMatrix() 
{
	for(int i = 0; i < NUM_CASTERS; i++)
	{
		int steerRow = 2*i;
		int rollRow = 2*i + 1;

		// Caster position values
		double hx = casterPosXSign(i+1) * DIST_TO_CASTER_X;
		double hy = casterPosYSign(i+1) * DIST_TO_CASTER_Y;

		// Take sin/cos of steeering angle
		double steerSin = sin(_q_steer(i));
		double steerCos = cos(_q_steer(i));

		// Compute steer row
		/**************** Alternative sign convention *******************/
		// C(steerRow,0) = -steerSin / PC_b;
		// C(steerRow,1) = steerCos / PC_b;
		// C(steerRow,2) = (hx * steerCos + hy * steerSin) / PC_b - 1.0;
		/****************************************************************/
		_C(steerRow,0) = steerSin / PC_b;
		_C(steerRow,1) = -steerCos / PC_b;
		_C(steerRow,2) = -(hx * steerCos + hy * steerSin) / PC_b - 1.0;

		// Compute roll row
		_C(rollRow,0) = steerCos / PC_r;
		_C(rollRow,1) = steerSin / PC_r;
		_C(rollRow,2) = (hx * steerSin - hy * steerCos) / PC_r;
	}
}

/* ------------ Static functions-------------- */
// Determine sign of x position of caster based on number
inline double OmniVeyorPlugin::casterPosXSign(int casterNum)
{
	switch(casterNum)
	{
		case 1:
		case 2:
			return 1.0;
		case 3:
		case 4:
			return -1.0;
		default:
			return 0;
	}
}


// Determine sign of y position of caster based on number
inline double OmniVeyorPlugin::casterPosYSign(int casterNum)
{
	switch(casterNum)
	{
		case 2:
		case 3:
			return 1.0;
		case 1:		
		case 4:
			return -1.0;
		default:
			return 0;
	}
}

// Function to clamp velocity command to max values
Eigen::Vector3d OmniVeyorPlugin::clampVelocity(Eigen::Vector3d xd_com_in){
	// Clamp velocity values to maximum allowed
	xd_com_in(0) = abs(xd_com_in(0)) < MAX_VEL_TRANS ? xd_com_in(0) : copysign(MAX_VEL_TRANS, xd_com_in(0));
	xd_com_in(1) = abs(xd_com_in(1)) < MAX_VEL_TRANS ? xd_com_in(1) : copysign(MAX_VEL_TRANS, xd_com_in(1));
	xd_com_in(2) = abs(xd_com_in(2)) < MAX_VEL_ROT ? xd_com_in(2) : copysign(MAX_VEL_ROT, xd_com_in(2));

	return xd_com_in;
}


// Function to clamp acceleration command to max values (accel is change in xd_des for one control loop here)
Eigen::Vector3d OmniVeyorPlugin::clampAcceleration(Eigen::Vector3d xdd_com_in){

	// Clamp acceleration values to maximum allowed

	// Check accel/decel for x
	if(xdd_com_in(0) >= 0){
		// Clamp trans accel
		xdd_com_in(0) = xdd_com_in(0) < MAX_VEL_TRANS_INC ? xdd_com_in(0) : MAX_VEL_TRANS_INC;
	}
	else{
		// Clamp trans decel
		xdd_com_in(0) = xdd_com_in(0) > -MAX_VEL_TRANS_DEC ? xdd_com_in(0) : -MAX_VEL_TRANS_DEC;
	}

	// Check accel/decel for y
	if(xdd_com_in(1) >= 0){
		// Clamp trans accel
		xdd_com_in(1) = xdd_com_in(1) < MAX_VEL_TRANS_INC ? xdd_com_in(1) : MAX_VEL_TRANS_INC;
	}
	else{
		// Clamp trans decel
		xdd_com_in(1) = xdd_com_in(1) > -MAX_VEL_TRANS_DEC ? xdd_com_in(1) : -MAX_VEL_TRANS_DEC;
	}
	
	// Check accel/decel for theta
	if(xdd_com_in(2) >= 0){
		// Clamp trans accel
		xdd_com_in(2) = xdd_com_in(2) < MAX_VEL_ROT_INC ? xdd_com_in(2) : MAX_VEL_ROT_INC;
	}
	else{
		// Clamp trans decel
		xdd_com_in(2) = xdd_com_in(2) > -MAX_VEL_ROT_DEC ? xdd_com_in(2) : -MAX_VEL_ROT_DEC;
	}

	return xdd_com_in;
}
