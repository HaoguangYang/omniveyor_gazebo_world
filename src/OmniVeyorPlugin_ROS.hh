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
#include "geometry_msgs/Twist.h"

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

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

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

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };
}
#endif 
