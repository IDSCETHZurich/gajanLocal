// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2006 Ruben Smits <ruben.smits@mech.kuleuven.ac.be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "CartesianGeneratorPos.hpp"
#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT( trajectory_generators::CartesianGeneratorPos );

namespace trajectory_generators
{

    using namespace RTT;
    using namespace KDL;
    using namespace std;
    using namespace brics_actuator;

    CartesianGeneratorPos::CartesianGeneratorPos(string name)
        : TaskContext(name,PreOperational),
          m_motion_profile(6,VelocityProfile_Trap(0,0)),
          m_is_moving(false)
    {
        //Creating TaskContext

        //Adding Ports
        this->addPort("CartesianPoseMsr",m_position_meas_port);
        this->addPort("CartesianPoseDes",m_position_desi_port);
        this->addPort("CartesianTwistDes",m_velocity_desi_port);
        this->addPort("moveFinished",m_move_finished_port);

        //Adding Properties
		this->addProperty("max_vel",m_maximum_velocity).doc("Maximum Velocity in Trajectory");
		this->addProperty("max_acc",m_maximum_acceleration).doc("Maximum Acceleration in Trajectory");

        //Adding Commands
        this->addOperation("moveTo",&CartesianGeneratorPos::moveTo,this,OwnThread)
	  .doc("Set the position setpoint")
	  .arg("setpoint", "position setpoint for end effector")
	  .arg("time", "minimum time to execute trajectory" );

        //Adding Methods
        this->addOperation( "resetPosition",&CartesianGeneratorPos::resetPosition,this,OwnThread).doc("Reset generator's position" );

    }

    CartesianGeneratorPos::~CartesianGeneratorPos()
    {
    }

    bool CartesianGeneratorPos::configureHook()
    {
      for(unsigned int i=0;i<3;i++){
	m_motion_profile[i].SetMax(m_maximum_velocity.vel[i],m_maximum_acceleration.vel[i]);
	m_motion_profile[i+3].SetMax(m_maximum_velocity.rot[i],m_maximum_acceleration.rot[i]);
      }      
      return true;

    }

    bool CartesianGeneratorPos::startHook()
    {
        m_is_moving = false;
	//initialize
	CartesianPose pose;
	m_position_meas_port.read(pose);
	m_position_desi_port.write(pose);
	CartesianTwist twist;
	SetToZero(m_velocity_desi_local);
	twist.translation.x=m_velocity_desi_local.vel.x();
	twist.translation.y=m_velocity_desi_local.vel.y();
	twist.translation.z=m_velocity_desi_local.vel.z();
	twist.rotation.x=m_velocity_desi_local.rot.x();
	twist.rotation.y=m_velocity_desi_local.rot.y();
	twist.rotation.z=m_velocity_desi_local.rot.z();
	m_velocity_desi_port.write(twist);
        return true;
    }

    void CartesianGeneratorPos::updateHook()
    {
        if (m_is_moving){
        	m_time_passed = os::TimeService::Instance()->secondsSince(m_time_begin);
            if ( m_time_passed > m_max_duration ){
                // set end position
                m_position_desi_local = m_traject_end;
                SetToZero(m_velocity_desi_local);
                m_move_finished_port.write(true);
                m_is_moving = false;
            }else{
                // position
                m_velocity_delta = Twist(Vector(m_motion_profile[0].Pos(m_time_passed),
						m_motion_profile[1].Pos(m_time_passed),
						m_motion_profile[2].Pos(m_time_passed)),
					 Vector(m_motion_profile[3].Pos(m_time_passed),
						m_motion_profile[4].Pos(m_time_passed),
						m_motion_profile[5].Pos(m_time_passed)) );
                m_position_desi_local = Frame( m_traject_begin.M * Rot( m_traject_begin.M.Inverse( m_velocity_delta.rot ) ),
					       m_traject_begin.p + m_velocity_delta.vel);

                // velocity
                for(unsigned int i=0; i<6; i++)
                	m_velocity_desi_local(i) = m_motion_profile[i].Vel( m_time_passed );
            }


	    //TODO:convert frame into CartesianPose
	    CartesianPose pose;
	    pose.position.x=m_position_desi_local.p.x();
	    pose.position.y=m_position_desi_local.p.y();
	    pose.position.z=m_position_desi_local.p.z();
	    m_position_desi_local.M.GetQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	    m_position_desi_port.write(pose);


	    CartesianTwist twist;
	    twist.translation.x=m_velocity_desi_local.vel.x();
	    twist.translation.y=m_velocity_desi_local.vel.y();
	    twist.translation.z=m_velocity_desi_local.vel.z();
	    twist.rotation.x=m_velocity_desi_local.rot.x();
	    twist.rotation.y=m_velocity_desi_local.rot.y();
	    twist.rotation.z=m_velocity_desi_local.rot.z();
	    m_velocity_desi_port.write(twist);
        }
    }

    void CartesianGeneratorPos::stopHook()
    {
    }

    void CartesianGeneratorPos::cleanupHook()
    {
    }

  bool CartesianGeneratorPos::moveTo(CartesianPose pose, double time)
    {
	  m_max_duration = 0;
	  
	  m_traject_end.p.x(pose.position.x);
	  m_traject_end.p.y(pose.position.y);
	  m_traject_end.p.z(pose.position.z);
	  m_traject_end.M=Rotation::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);

	  // get current position
	  CartesianPose pose_meas;
	  m_position_meas_port.read(pose_meas);
	  m_traject_begin.p.x(pose_meas.position.x);
	  m_traject_begin.p.y(pose_meas.position.y);
	  m_traject_begin.p.z(pose_meas.position.z);
	  m_traject_begin.M=Rotation::Quaternion(pose_meas.orientation.x,pose_meas.orientation.y,pose_meas.orientation.z,pose_meas.orientation.w);

	  m_velocity_begin_end = diff(m_traject_begin, m_traject_end);

	  // Set motion profiles
	  for (unsigned int i=0; i<6; i++){
	    m_motion_profile[i].SetProfileDuration( 0, m_velocity_begin_end(i), time );
	    m_max_duration = max( m_max_duration, m_motion_profile[i].Duration() );
	  }
	  
      // Rescale trajectories to maximal duration
	  for (unsigned int i=0; i<6; i++)
	    m_motion_profile[i].SetProfileDuration( 0, m_velocity_begin_end(i), m_max_duration );
	  
	  m_time_begin = os::TimeService::Instance()->getTicks();
	  m_time_passed = 0;

	  m_is_moving = true;
	  return true;
    }

    void CartesianGeneratorPos::resetPosition()
    {
      CartesianPose pose;
      m_position_meas_port.read(pose);
      SetToZero(m_velocity_desi_local);
      CartesianTwist twist;
      twist.translation.x=m_velocity_desi_local.vel.x();
      twist.translation.y=m_velocity_desi_local.vel.y();
      twist.translation.z=m_velocity_desi_local.vel.z();
      twist.rotation.x=m_velocity_desi_local.rot.x();
      twist.rotation.y=m_velocity_desi_local.rot.y();
      twist.rotation.z=m_velocity_desi_local.rot.z();
      m_position_desi_port.write(pose);
      m_velocity_desi_port.write(twist);
      m_is_moving = false;
    }
}//namespace
