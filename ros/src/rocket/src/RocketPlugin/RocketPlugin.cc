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
  _quat2mrp(quat2mrp_functions()),
  _tf_linvel(tf_linvel_functions()),
  _tf_angvel(tf_angvel_functions())
{
  _rocket_aero_forces.arg (2,p); //Set rocket constant parameters for each eom
  _rocket_aero_moments.arg(2,p); //Set rocket constant parameters for each eom
  _rocket_prop_forces.arg (2,p); //Set rocket constant parameters for each eom
  _rocket_prop_moments.arg(2,p); //Set rocket constant parameters for each eom

  _rocket_aero_forces.res(0,FA_b); //Set rocket aero forces
  _rocket_aero_moments.res(0,MA_b); //Set rocket aero moments
  _rocket_prop_forces.res(0,FP_b); //Set rocket propulsion forces
  _rocket_prop_moments.res(0,FP_b); //Set rocket propulsion moments

  _quat2mrp.res(0,r_nb);  //mrp
  _tf_linvel.res(0,v_b);  //transfer frame
  _tf_angvel.res(0,omg_b);//transfer frame

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
    
    // Get current states from gazebo
    auto pose_world_inertia = this->body->WorldInertialPose(); //pose in world inertia frame ????
    auto pos_ine = pose_world_inertia.Pos(); // [x,y,z], use [] to access elements
    auto rot_ine = pose_world_inertia.Rot(); // [w x y z], quaternions. use .qw .qx .qy .qz to access.

    auto omega_ine = this->body->WorldAngularVel(); // inertial angular velocity
    auto vel_ine = this->body->WorldLinearVel(); // inertial linaer velocity
    
    // std::cout<<"\n Inertial Velocity"<<std::endl;
    // std::cout<<vel_ine<<std::endl;

    // TODO: Convert inertial frame velocities to body frame.
    double pos_n[3] = {pos_ine.X(),pos_ine.Y(),pos_ine.Z()};
    double q[4] = {rot_ine.W(), rot_ine.X(), rot_ine.Y(), rot_ine.Z()};
    double v_ine[3] = {vel_ine.X(),vel_ine.Y(),vel_ine.Z()};
    double omg_ine[3] = {omega_ine.X(), omega_ine.Y(), omega_ine.Z()};

    _quat2mrp.arg(0,q);
    _tf_linvel.arg(0,v_ine);
    _tf_linvel.arg(1,q);
    _tf_angvel.arg(0,omg_ine);
    _tf_angvel.arg(1,q);

    _tf_linvel.eval(); //v_b
    _tf_angvel.eval(); //omega_b
    _quat2mrp.eval(); //r_nb
    
  
    double *x = new double[13];
    std::copy(omg_b,omg_b+3,x);
    std::copy(r_nb,r_nb+4,x+3);
    std::copy(v_b,v_b+3,x+7);
    std::copy(pos_n,pos_n+3,x+10);
    x[13] = m_fuel;
    //{omg_b, r_nb, v_b, pos_n, m_fuel}


    // TODO: Set states x and inputs u
    double u[3] = {0};

    
    FP_b[2] = 10;
    
    // TODO: Add forces and moments to relative parts
    this->motor->AddRelativeForce(ignition::math::Vector3d(FP_b[0], FP_b[1],FP_b[2]));
    this->body->AddRelativeForce(ignition::math::Vector3d(FA_b[0], FA_b[1],FA_b[2]));
    
    motor_inertial->SetMass(m_fuel);
    motor_inertial->SetInertiaMatrix(0, 0, 0, 0, 0, 0); // treat as point mass
    this->lastUpdateTime = curTime;
  }
}
