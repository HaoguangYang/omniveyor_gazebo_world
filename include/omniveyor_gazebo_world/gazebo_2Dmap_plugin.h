/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GAZEBO_2DMAP_PLUGIN_H
#define GAZEBO_2DMAP_PLUGIN_H

#include <iostream>
#include <math.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <std_srvs/Empty.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_srvs/Empty.h>

#include <Eigen/Dense>

namespace gazebo {

#if GAZEBO_MAJOR_VERSION >= 9
  typedef ignition::math::Vector3d vector3d;
  auto GetPhysicsPtr = std::mem_fn(&gazebo::physics::World::Physics);
#else
  typedef math::Vector3 vector3d;
  auto GetPhysicsPtr = std::mem_fn(&gazebo::physics::World::GetPhysicsEngine);
#endif

//===============================================================================================//
//========================================= DEBUGGING ===========================================//
//===============================================================================================//

/// \addtogroup Debug Print Switches
/// @{

// The following boolean constants enable/disable debug printing when certain plugin methods are called.
// Suitable for debugging purposes. Left on permanently can swamp std::out and can crash Gazebo.

static const bool kPrintOnPluginLoad    = false;
static const bool kPrintOnUpdates       = false;
static const bool kPrintOnMsgCallback   = false;

/// @}

// Default values
static const std::string kDefaultNamespace = "";
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

//===============================================================================================//
//================================== TOPICS FOR ROS INTERFACE ===================================//
//===============================================================================================//

// These should perhaps be defined in an .sdf/.xacro file instead?
static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

/// \brief    Special-case topic for ROS interface plugin to listen to (if present)
///           and broadcast transforms to the ROS system.
static const std::string kBroadcastTransformSubtopic = "broadcast_transform";


/// \brief      Obtains a parameter from sdf.
/// \param[in]  sdf           Pointer to the sdf object.
/// \param[in]  name          Name of the parameter.
/// \param[out] param         Param Variable to write the parameter to.
/// \param[in]  default_value Default value, if the parameter not available.
/// \param[in]  verbose       If true, gzerror if the parameter is not available.
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}


//===============================================================================================//
//==================================       PLUGIN CLASS       ===================================//
//===============================================================================================//
/// \brief    Octomap plugin for Gazebo.
/// \details  This plugin is dependent on ROS, and is not built if NO_ROS=TRUE is provided to
///           CMakeLists.txt. The PX4/Firmware build does not build this file.
class OccupancyMapFromWorld : public WorldPlugin {
 public:
  OccupancyMapFromWorld()
      : WorldPlugin(), name_("gazebo_2Dmap_plugin")
  {
    ROS_INFO_NAMED(name_, "occupancy map plugin started");
  }
  virtual ~OccupancyMapFromWorld();



 protected:

  /// \brief Load the plugin.
  /// \param[in] _parent Pointer to the world that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

  bool worldCellIntersection(const vector3d& cell_center, const double cell_length,
                              gazebo::physics::RayShapePtr ray);
  

//  void FloodFill(const math::Vector3& seed_point,
//                 const math::Vector3& bounding_box_origin,
//                 const math::Vector3& bounding_box_lengths,
//                 const double leaf_size);
  
  /*! \brief
  */
  void CreateOccupancyMap();

  static void cell2world(unsigned int cell_x, unsigned int cell_y,
                         double map_size_x, double map_size_y, double map_resolution,
                         double& world_x, double &world_y);

  static void world2cell(double world_x, double world_y,
                         double map_size_x, double map_size_y, double map_resolution,
                         unsigned int& cell_x, unsigned int& cell_y);

  static bool cell2index(int cell_x, int cell_y,
                         unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& map_index);

  static bool index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& cell_x, unsigned int& cell_y);

 private:
  bool ServiceCallback(std_srvs::Empty::Request& req,
                       std_srvs::Empty::Response& res);

  physics::WorldPtr world_;
  ros::NodeHandle nh_;
  ros::ServiceServer map_service_;
  ros::Publisher map_pub_;
  nav_msgs::OccupancyGrid* occupancy_map_;
  std::string name_;
  double map_resolution_;
  double map_height_;
  double map_size_x_;
  double map_size_y_;
  double init_robot_x_;
  double init_robot_y_;
};

} // namespace gazebo

/// \brief    This class can be used to apply a first order filter on a signal.
///           It allows different acceleration and deceleration time constants.
/// \details
///           Short reveiw of discrete time implementation of first order system:
///           Laplace:
///             X(s)/U(s) = 1/(tau*s + 1)
///           continous time system:
///             dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
///           discretized system (ZoH):
///             x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
template <typename T>
class FirstOrderFilter {

 public:
  FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

  /// \brief    This method will apply a first order filter on the inputState.
  T updateFilter(T inputState, double samplingTime) {

    T outputState;
    if (inputState > previousState_) {
      // Calcuate the outputState if accelerating.
      double alphaUp = exp(-samplingTime / timeConstantUp_);
      // x(k+1) = Ad*x(k) + Bd*u(k)
      outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

    }
    else {
      // Calculate the outputState if decelerating.
      double alphaDown = exp(-samplingTime / timeConstantDown_);
      outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
    }
    previousState_ = outputState;
    return outputState;

  }

  ~FirstOrderFilter() {}

 protected:
  double timeConstantUp_;
  double timeConstantDown_;
  T previousState_;
};

/// \brief    Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

#endif  // GAZEBO_2DMAP_PLUGIN_H
