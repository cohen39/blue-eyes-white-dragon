/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <functional>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "RocketPlugin.hh"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RocketPlugin)

////////////////////////////////////////////////////////////////////////////////
RocketPlugin::RocketPlugin()
  :
	// _double_this(double_this_functions()),
  _rocket_aero_forces(rocket_aero_forces_functions()),
  _rocket_aero_moments(rocket_aero_moments_functions()),
  _rocket_prop_forces(rocket_prop_forces_functions()),
  _rocket_prop_moments(rocket_prop_moments_functions()),
  _quat2mrp(quat2mrp_functions())
{
  _rocket_aero_forces.arg (2,p); //Set rocket constant parameters for each eom
  _rocket_aero_moments.arg(2,p); //Set rocket constant parameters for each eom
  _rocket_prop_forces.arg (2,p); //Set rocket constant parameters for each eom
  _rocket_prop_moments.arg(2,p); //Set rocket constant parameters for each eom

  _rocket_aero_forces.res(0,FA_b); //Set rocket aero forces
  _rocket_aero_moments.res(0,MA_b); //Set rocket aero moments
  _rocket_prop_forces.res(0,FP_b); //Set rocket propulsion forces
  _rocket_prop_moments.res(0,FP_b); //Set rocket propulsion moments
}

/////////////////////////////////////////////////
RocketPlugin::~RocketPlugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool RocketPlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::JointPtr &_joint)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string jointName = _sdf->Get<std::string>(_sdfParam);
  _joint = this->model->GetJoint(jointName);
  if (!_joint)
  {
    gzerr << "Failed to find joint [" << jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool RocketPlugin::FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::LinkPtr &_link)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string linkName = _sdf->Get<std::string>(_sdfParam);
  _link = this->model->GetLink(linkName);
  if (!_link)
  {
    gzerr << "Failed to find link [" << linkName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void RocketPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "RocketPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "RocketPlugin _sdf pointer is NULL");
  this->model = _model;

  // Find motor link to apply propulsion forces
  if (!this->FindLink("motor", _sdf, this->motor)) {
    GZ_ASSERT(false, "RocketPlugin failed to find motor");
  }
  
  // Find body link to apply aero forces
  if (!this->FindLink("body", _sdf, this->body)) {
    GZ_ASSERT(false, "RocketPlugin failed to find body");
  }

  // Update time.
  this->lastUpdateTime = this->model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&RocketPlugin::Update, this, std::placeholders::_1));

  gzlog << "Rocket ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void RocketPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastUpdateTime)
  {
    double dt = (curTime - this->lastUpdateTime).Double();

    // ref to definition of Model.hh to get parameters
    auto motor_inertial = this->motor->GetInertial();
    float m_fuel = motor_inertial->Mass();
    float m_dot = 0.1;
    float m_empty = 0.0;
    m_fuel = m_fuel - m_dot*dt; // x[13]
  
    float ve = this->p[6]; // exit velocity, m/s, guess
    
    // auto pose_world_Cog =  this->body->WorldCoGPose(); // pose in world link frame, maybe
    // auto pos_cog = pose_world_Cog.Pos(); // x,y,z
    // auto rot_cog = pose_world_Cog.Rot(); // w x y z, quaternions.
    // Get current states from gazebo
    auto pose_world_inertia = this->body->WorldInertialPose(); //pose in world inertia frame ????
    auto pos_ine = pose_world_inertia.Pos(); // x,y,z
    auto rot_ine = pose_world_inertia.Rot(); // w x y z, quaternions.

    // std::cout<<"\nPos comparison"<<std::endl;
    // std::cout<<pos_rel<<"\n"<<pos_ine<<std::endl;
    
    auto omega_ine = this->body->WorldAngularVel(); // inertial angular velocity
    auto vel_ine = this->body->WorldLinearVel(); // inertial linaer velocity
    
    // TODO: Convert inertial frame values to body frame.

    // TODO: Set states x and inputs u
    
    FP_b[1] = 1;
    
    this->motor->AddRelativeForce(ignition::math::Vector3d(FP_b[0], FP_b[1],FP_b[2]));
    motor_inertial->SetMass(m_fuel);
    motor_inertial->SetInertiaMatrix(0, 0, 0, 0, 0, 0); // treat as point mass
    this->lastUpdateTime = curTime;
  }
}
