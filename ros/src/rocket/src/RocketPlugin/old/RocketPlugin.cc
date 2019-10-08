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
  _rocket_force_moment(rocket_force_moment_functions())
{
  _rocket_force_moment.arg (0,x_gzb); //Set address of rocket state for each eom
  _rocket_force_moment.arg (1,u); 
  _rocket_force_moment.arg (2,p); //Set rocket constant parameters for each eom

  _rocket_force_moment.res (0,F_b); //Set rocket forces
  _rocket_force_moment.res (1,M_b); //Set rocket moments
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

    // Find body link to apply forces and moments
  if (!this->FindLink("body", _sdf, this->body)) {
    GZ_ASSERT(false, "RocketPlugin failed to find body");
  }

  // Disable gravity, we will handle this in plugin
  this->body->SetGravityMode(false);
  this->body->GetInertial()->SetMass(0.4); //  set your total mass with fuel herethis->body->SetGravityMode(false);
  
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
    double dt           = (curTime - this->lastUpdateTime).Double();

    // ref to definition of Model.hh to get parameters
    auto  rocket_inertial= this->body->GetInertial();
    float m_empty = p[13];
    float m_total = rocket_inertial->Mass();
    float m_fuel  = m_total - m_empty;
    float m_dot   = u[0];
    gzdbg << "\nm_empty = " << m_empty <<std::endl << "m_total = " << m_total <<std::endl;
    m_fuel = m_fuel - m_dot*dt; // x[13]
    if (m_fuel <= 0)
    {
      m_fuel = 0;
      u[0] = 0;
    }
    float Jx = p[1];
    float Jy = p[2];
    float Jz = p[3];
    float Jxz = p[4];
    float l_motor = p[15];
    rocket_inertial->SetMass(m_fuel+m_empty);
    rocket_inertial->SetInertiaMatrix(Jx + m_fuel*l_motor*l_motor, Jy+m_fuel*l_motor*l_motor, Jz, 0, 0, 0); // treat as point mass
    this->lastUpdateTime = curTime;
        
    // Get current states from gazebo
    auto pose_world_inertia = this->body->WorldPose();  //pose in world inertia frame ????
    auto pos_ENU            = pose_world_inertia.Pos();         // [x,y,z], use [] to access elements
    auto rot_ENU            = pose_world_inertia.Rot();         // [w x y z], quaternions. use .qw .qx .qy .qz to access.

    auto omega_ENU          = this->body->RelativeAngularVel();    // inertial angular velocity
    auto vel_ENU            = this->body->RelativeLinearVel();     // inertial linaer velocity

    double pos_n[3]   = {pos_ENU.X()  ,pos_ENU.Y()  ,pos_ENU.Z()  };
    double q[4]       = {rot_ENU.W()  ,rot_ENU.X()  ,rot_ENU.Y()  ,rot_ENU.Z()};
    double v_ENU[3]   = {vel_ENU.X()  ,vel_ENU.Y()  ,vel_ENU.Z()  };
    double omg_ENU[3] = {omega_ENU.X(),omega_ENU.Y(),omega_ENU.Z()};

    //Convert velocities wrt inertial frame to velocities wrt body frame
    //(It is also necessary to convert orientation from world to local csys using modified rodrigues parameters)
    
    //Fill x array with states (ENU frame)
    //{omg_b, r_nb, v_b, pos_n, m_fuel}
    std::copy(omg_ENU, omg_ENU+3, x_gzb   );
    std::copy(q , q+4, x_gzb+3 );
    std::copy(v_ENU  , v_ENU  +3, x_gzb+7 );
    std::copy(pos_ENU, pos_ENU+3, x_gzb+10);
    x_gzb[13] = m_fuel;

    _rocket_force_moment.eval (); //Calculate new value of F_b and M_b

    //TODO: Make sure local csys convention in Gazebo is consistent with what we used for formulation of casadi functions
    //Apply updated forces and moments to rocket body and motor in gazebo
    
    gzdbg << "M_fuel = " << m_fuel << std::endl;
    gzdbg << "F_F=" << F_b[0] << " F_L=" << F_b[1] << " F_B=" << F_b[2] << std::endl;
    gzdbg << "M_F=" << M_b[0] << " M_L=" << M_b[1] << " M_B=" << M_b[2] << std::endl;

    for (int i=0; i<3; i++) {
      GZ_ASSERT(isfinite(F_b[i]), "non finite force");
      GZ_ASSERT(isfinite(M_b[i]), "non finite moment");
    }

    this->body ->AddRelativeForce (ignition::math::Vector3d(F_b[0], F_b[1], F_b[2]));
    this->body ->AddRelativeTorque(ignition::math::Vector3d(M_b[0], M_b[1], M_b[2]));
  }
}
