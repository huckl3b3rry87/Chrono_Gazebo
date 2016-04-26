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
//and visuals within gazebo
//
// =============================================================================

#include "GcVehicleBuilder.hh"

#include <chrono_vehicle/driver/ChPathFollowerACCDriver.h>
#include <chrono_vehicle/powertrain/SimplePowertrain.h>
#include <chrono_vehicle/wheeled_vehicle/tire/RigidTire.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <core/ChCoordsys.h>
#include <core/ChMathematics.h>
#include <core/ChQuaternion.h>
#include <core/ChVector.h>
#include <physics/ChBody.h>
#include <physics/ChSystem.h>
#include <cmath>
#include <iostream>
#include <vector>

#include "GcLocalVehicle.hh"
#include "GcNetworkVehicle.hh"

using namespace chrono;
using namespace gazebo;

GcVehicleBuilder::GcVehicleBuilder(physics::WorldPtr world,
		std::shared_ptr<chrono::ChSystem> chsys,
		gc::ChTerrainPtr terrain,
		double pathRadius,
		double maxSpeed,
		double followingTime,
		double minDist,
		double vehicleGap) :
		m_world(world), m_chsys(chsys), m_terrain(terrain),
				m_pathRadius(pathRadius), m_maxSpeed(maxSpeed),
				m_followingTime(followingTime), m_minDist(minDist),
				m_vehicleGap(vehicleGap) {
	m_networkVehicle = false;
	m_vehId = 0;
}

std::shared_ptr<GcVehicle> GcVehicleBuilder::BuildLocalGcVehicle() {

	const std::string id = std::to_string(m_vehId);

#ifdef DEBUG
	std::cout << "[GcVehicle] Start building vehicle " << id << "." << std::endl;
#endif /* debug information */

	// -- Chrono part --

	// create and initialize a Chrono vehicle model
	auto veh = std::make_shared<vehicle::WheeledVehicle>(m_chsys.get(),
			m_vehicleFile);

	const double ang = m_vehId * m_vehicleGap;
//	auto pos = ChVector<>(m_pathRadius * std::cos(ang),
//			m_pathRadius * std::sin(ang), 1);
//	auto rot = Q_from_AngZ(ang - (CH_C_PI / 2.0));
	auto pos = ChVector<>(-180-6 * m_vehId, 206, 0.5);
	auto rot = Q_from_AngZ(0);

#ifdef DEBUG
	std::cout << "[GcVehicle] Rotation Quaternion: " << rot.e0 << ", " << rot.e1
			<< ", " << rot.e2 << ", " << rot.e3 << std::endl;
#endif /* debug information */

	veh->Initialize(ChCoordsys<>(pos, rot));

#ifdef DEBUG
	std::cout << "[GcVehicle] Vehicle initialzied." << std::endl;
#endif /* debug information */

	// create and initialize a powertrain
	auto powertrain = std::make_shared<vehicle::SimplePowertrain>(
			m_powertrainFile);
	powertrain->Initialize();

#ifdef DEBUG
	std::cout << "[GcVehicle] Powertrain initialized." << std::endl;
#endif /* debug information */

	// create and initialize the tires
	const int numWheels = 2 * veh->GetNumberAxles();
	auto tires = std::vector<gc::ChRigidTirePtr>(numWheels);
	for (int i = 0; i < numWheels; i++) {
		//create the tires from the tire file
		tires[i] = std::make_shared<vehicle::RigidTire>(m_tireFile);
		tires[i]->Initialize(veh->GetWheelBody(i));

#ifdef DEBUG
		std::cout << "[GcVehicle] Tire " << i << " initialized." << std::endl;
#endif /* debug information */

	}

// create path follower
	auto driver = std::make_shared<vehicle::ChPathFollowerACCDriver>(*veh,
			m_steerFile, m_speedFile, m_path, std::string("my_path"), m_maxSpeed,
			m_followingTime, m_minDist, 100000.0,
			false);
	driver->GetSteeringController().CalcTargetLocation();

#ifdef DEBUG
	std::cout << "[GcVehicle] Path follower driver loading completes."
			<< std::endl;
#endif /* debug information */

// -- Gazebo part --

// create a GcVehicle
	auto gcVeh = std::make_shared<GcLocalVehicle>(m_vehId, m_terrain, veh,
			powertrain, tires, driver, m_maxSpeed, m_world);

// subscribe the GcVehicle to ros
//	auto opt = ros::SubscribeOptions::create<std_msgs::Float64>(
//			"/track_point" + id, 1,
//			boost::bind(&GcLocalVehicle::UpdateDriver, gcVeh, _1), ros::VoidPtr(),
//			m_queue);
//	m_lastSub = m_handle->subscribe(opt);

#ifdef DEBUG
	std::cout << "[GcVehicle] Vehicle subscribed to ROS." << std::endl;
#endif /* debug information */

// increase the vehicle id
	m_vehId++;

#ifdef DEBUG
	std::cout << "[GcVehicle] Vehicle " << id << " building complete."
			<< std::endl;
#endif /* debug information */

	return gcVeh;
}

std::shared_ptr<GcVehicle> GcVehicleBuilder::BuildNetworkGcVehicle() {
	return std::make_shared<GcNetworkVehicle>(m_vehId++, m_sockfd, m_world);
}

std::shared_ptr<GcVehicle> GcVehicleBuilder::BuildGcVehicle() {
	if (m_networkVehicle) {
		return BuildNetworkGcVehicle();
	}
	return BuildLocalGcVehicle();
}
