// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist, Leon Yang
// =============================================================================
//
//gcVehicle defines a vehicle based on chrono_vehicle and gazebo relying on the
//dynamics and definitions provided by chrono and the world geometry, sensors,
//and visuals within gazebo. This class enables connection to gazebo sensors
//and chrono vehicle
//
// =============================================================================

#include "GcNetworkVehicle.hh"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <cassert>
#include <cstdbool>
#include <iostream>
#include <string>

#include "GcNetworkUtil.hh"
#include "GcUtil.hh"

using namespace gazebo;

//constructor
GcNetworkVehicle::GcNetworkVehicle(int id,
		int sockfd,
		const physics::WorldPtr world) :
		m_id(id), m_sockfd(sockfd), m_world(world) {
}

/*
 * Initialize the vehicle models. Sensor is not initialized because network
 * vehicles do not have that.
 */
bool GcNetworkVehicle::Init() {
	if (m_initialized)
		return true;
	const std::string vehicleName = "vehicle_" + std::to_string(m_id);

	if ((m_gazeboVehicle = m_world->GetModel(vehicleName)) == NULL) {
		std::cerr << "COULD NOT FIND GAZEBO MODEL: " + vehicleName + '\n';
		return false;
	}

	m_initialized = true;
	m_gazeboWheels.push_back(m_gazeboVehicle->GetLink("wheel_0"));
	m_gazeboWheels.push_back(m_gazeboVehicle->GetLink("wheel_1"));
	m_gazeboWheels.push_back(m_gazeboVehicle->GetLink("wheel_2"));
	m_gazeboWheels.push_back(m_gazeboVehicle->GetLink("wheel_3"));
	assert(m_gazeboWheels[0] != NULL);
	assert(m_gazeboWheels[1] != NULL);
	assert(m_gazeboWheels[2] != NULL);
	assert(m_gazeboWheels[3] != NULL);

	return true;
}

/*
 * Get the next gcPacket from the server and update the model position with
 * that. Since the server sends out packets in order of network id, this function
 * must be called in order of the indices of the vehicles.
 */
void GcNetworkVehicle::Advance(double step) {
	struct gcPacket packet;
	gc::Recv(m_sockfd, (char*) &packet, sizeof(packet));
	m_gazeboVehicle->SetWorldPose(gc::GetGcPose(packet.poses[4]));
	for (int i = 0; i < 4; i++) {
		m_gazeboWheels[i]->SetWorldPose(gc::GetGcPose(packet.poses[i]));
	}
}

