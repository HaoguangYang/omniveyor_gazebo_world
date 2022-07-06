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
#ifndef GAZEBO_PLUGINS_OMNIVEYORPLUGIN_ROS_HH_
#define GAZEBO_PLUGINS_OMNIVEYORPLUGIN_ROS_HH_

#include <string>
#include <unordered_map>

#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>

#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/pose.pb.h"
#include "gazebo/msgs/twist.pb.h"

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Byte.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <omniveyor_common/definitions.h>
#include <omniveyor_common/electricalStatus.h>

namespace gazebo
{
  class OmniVeyorPluginPrivate;

  /// \class OmniVeyorPlugin OmniVeyorPlugin.hh
  /// \brief An abstract gazebo model plugin for tracked vehicles.
  /// \since 8.1
  ///
  /// The plugin processes the following parameters (all have defaults):
  /// <max_linear_speed>  Max linear velocity in m/s. Also max track velocity.
  ///                     Default is 1.0.
  /// <max_angular_speed>  Max angular speed in rad/s. Default is 1.0.
  /// <robot_namespace>  Namespace used as a prefix for gazebo topic names.
  ///                    Default is the name of the model.
  class GZ_PLUGIN_VISIBLE OmniVeyorPlugin : public ModelPlugin
  {
    /// \brief Default Contstuctor
    public: OmniVeyorPlugin();

    /// \brief Destructor
    public: virtual ~OmniVeyorPlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] _model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    /// \brief Initialize the plugin.
    public: void Init() override;

    /// \brief Reset the plugin.
    public: void Reset() override;

    /// \brief Namespace used as a prefix for gazebo topic names.
    protected: virtual std::string GetRobotNamespace();

    /// \brief Set new target velocity for the tracks.
    ///
    /// Descendant classes need to implement this function.
    ///
    /// \param[in] _left Velocity of left track.
    /// \param[in] _right Velocity of right track.
    protected: virtual void SetMotorVelocity(double _strFL, double _rolFL,
                                            double _strFR, double _rolFR,
                                            double _strRL, double _rolRL,
                                            double _strRR, double _rolRR);

    /// \brief Set new target velocity for the tracks based on the desired
    // body motion.
    ///
    /// \param[in] _linear Desired linear velocity of the vehicle.
    /// \param[in] _angular Desired angular velocity of the vehicle.
    protected: void SetBodyVelocity(
        double _linearX, double _linearY, double _angular);  // TODO peci1: make virtual

    /// \brief Callback for setting desired body velocity.
    ///
    /// Normally, this callback converts the x/yaw message to track velocities
    /// and calls SetTrackVelocity().
    ///
    /// \param[in] _msg Twist message from external publisher
    protected: void OnVelMsg(ConstTwistPtr &_msg);  // TODO peci1: make virtual

    /// \brief Mutex to protect updates
    protected: std::mutex mutex;

    /// \brief Private data pointer
    private: std::unique_ptr<OmniVeyorPluginPrivate> dataPtr;

    private: std::string modelName, worldName;
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber cmdVelSub;
    private: ros::Subscriber ctrlModeSub;
    private: ros::Publisher odomPub, eStatusPub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->SetBodyVelocity(_msg->linear.x, _msg->linear.y, _msg->angular.z);
    }
    public: void OnModeMsg(const std_msgs::ByteConstPtr &_msg)
    {
      // TODO: Control Mode is not behaving the right behavior.
      if (_msg->data==0)
        this->SetBodyVelocity(0., 0., 0.);
    }

    private: nav_msgs::Odometry odom;
    /// \brief ROS helper function that processes messages
    private: void QueueThread();

    private:
      void controlUpdate();
      void updateJointData();
      void updateConstraintPinvMatrix();
      void updateCpMatrix();
      void updateConstraintMatrix();
      inline double casterPosXSign(int casterNum);
      inline double casterPosYSign(int casterNum);
      Eigen::Vector3d clampVelocity(Eigen::Vector3d xd_com_in);
      Eigen::Vector3d clampAcceleration(Eigen::Vector3d xdd_com_in);
      
      Eigen::Vector3d _x_local;                             	// local position
      Eigen::Vector3d _gx;                             		// global position
      Eigen::Vector3d _gx_des;  								// last global operational space position command sent 
      Eigen::Vector3d _xd_local; 								// local operational space velocity 
      Eigen::Vector3d _gxd; 								 	// global operational space velocity 
      Eigen::Vector3d _gxd_com;								// command global operational space velocity (sent from outside)
      Eigen::Vector3d _gxd_des;								// last global operational space velocity command sent
      Eigen::Vector3d _gxdd;									// global operational space acceleration 
      Eigen::Vector3d _gxdd_des; 								// last global operational space acceleration command sent 
      
      Eigen::Matrix<double, NUM_CASTERS, 1> _q_steer; 	    // joint steering angles
      Eigen::Matrix<double, NUM_MOTORS,  1> _qd; 		 		// joint velocities
      Eigen::Matrix<double, NUM_MOTORS,  1> _qd_des; 	 	    // commanded joint velocities
      Eigen::Matrix<double, NUM_MOTORS,  1> _tq; 		 		// joint torques
      Eigen::Matrix<double, NUM_MOTORS,  1> _tq_des; 	 		// commanded joint torques
      
      Eigen::Vector3d _g_cf;    	 							// global control force
      Eigen::Vector3d _cf_des_local; 							// local commanded control force
      Eigen::Matrix3d _lambda; 								// vehicle mass matrix
      Eigen::Vector3d _mu;     					 			// velocity coupling vector (centripetal,coriolis)
      
      void setZeros(){
          // initialize variables
          this->_x_local << 0., 0., 0.;
          this->_gx << 0., 0., 0.;
          this->_gx_des << 0., 0., 0.;
          this->_xd_local << 0., 0., 0.;
          this->_gxd << 0., 0., 0.;
          this->_gxd_com << 0., 0., 0.;
          this->_gxd_des << 0., 0., 0.;
          this->_gxdd << 0., 0., 0.;
          this->_gxdd_des << 0., 0., 0.;
          this->_g_cf << 0., 0., 0.;
          this->_cf_des_local << 0., 0., 0.;
          this->_heading = 0.;
      }
      
      Eigen::Matrix<double, NUM_MOTORS, 3> _C; 		 		// constraint matrix
      Eigen::Matrix<double, NUM_MOTORS, 3> _Cp; 
      Eigen::Matrix<double, NUM_MOTORS, NUM_MOTORS> _Jq; 		 					
      Eigen::MatrixXd _C_pinv; 	 							// constraint matrix pseudoinverse (calculated -> dynamic size)		 	  		
      
      double _heading; 										// Vehicle heading (theta)
      ros::Time current_time;
  };
}

#endif 
