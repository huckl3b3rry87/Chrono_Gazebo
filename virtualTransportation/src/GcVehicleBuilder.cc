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

#include <boost/bind/bind.hpp>
#include <chrono/core/ChCoordsys.h>
#include <chrono_vehicle/driver/ChPathFollowerACCDriver.h>
#include <chrono_vehicle/powertrain/SimplePowertrain.h>
#include <chrono_vehicle/wheeled_vehicle/tire/RigidTire.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <gazebo/common/SingletonT.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/physics/World.hh>
#include <GcVehicleBuilder.hh>
#include <ros/callback_queue.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <vector>

using namespace chrono;
using namespace gazebo;

GcVehicleBuilder::GcVehicleBuilder(physics::WorldPtr world,
		std::shared_ptr<chrono::ChSystem> chsys, GcVehicle::ChTerrainPtr terrain,
		const double pathRadius, const double vehicleGap, const double maxSpeed) :
		m_world(world), m_chsys(chsys), m_terrain(terrain), m_pathRadius(
				pathRadius), m_vehicleGap(vehicleGap), m_maxSpeed(maxSpeed) {
	m_vehicleDist = pathRadius * vehicleGap;
	m_followingTime = 2.0;
}

std::shared_ptr<GcVehicle> GcVehicleBuilder::BuildGcVehicle() {

	const std::string id = std::to_string(m_vehId);

#ifdef DEBUG
	std::cout << "[GcVehicle] Start building vehicle " << id << "." << std::endl;
#endif /* debug information */

	// -- Chrono part --

	// create and initialize a Chrono vehicle model
	auto veh = std::make_shared<vehicle::WheeledVehicle>(m_chsys.get(),
			m_vehicleFile);

	const double ang = m_vehId * m_vehicleGap;
	auto pos = ChVector<>(m_pathRadius * std::cos(ang),
			m_pathRadius * std::sin(ang), 1);
	auto rot = Q_from_AngZ(ang - (CH_C_PI / 2.0));

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
	auto tires = std::vector<GcVehicle::ChRigidTirePtr>(numWheels);
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
			m_followingTime, 5.0, m_vehicleDist, true);

#ifdef DEBUG
	std::cout << "[GcVehicle] Path follower driver loading completes."
	<< std::endl;
#endif /* debug information */

// -- Gazebo part --

	physics::ModelPtr gazeboVehicle;
	std::vector<physics::ModelPtr> gazeboWheels(numWheels);
	sensors::RaySensorPtr raySensor;

// retrieve vehicle model from Gazebo
	const std::string vehicleName = "vehicle" + id;
	if ((gazeboVehicle = m_world->GetModel(vehicleName)) == NULL) {
		std::cerr << "COULD NOT FIND GAZEBO MODEL: " + vehicleName + '\n';
		return NULL;
	}

#ifdef DEBUG
	std::cout << "[GcVehicle] Vehicle model loading complete." << std::endl;
#endif /* debug information */

// retrieve wheel models from Gazebo
	for (int i = 0; i < numWheels; i++) {
		physics::ModelPtr wheelPtr;
		const std::string wheelName = vehicleName + "::wheel" + std::to_string(i);
		if ((wheelPtr = m_world->GetModel(wheelName)) != NULL) {
			gazeboWheels[i] = wheelPtr;
		} else {
			std::cerr << "COULD NOT FIND GAZEBO MODEL: " + wheelName + '\n';
			return NULL;
		}

#ifdef DEBUG
		std::cout << "[GcVehicle] Wheel model " << id << " loading complete"
		<< std::endl;
#endif /* debug information */

	}

// retrieve sensor model from Gazebo
	const std::string sensorName = m_world->GetName() + "::" + vehicleName
			+ "::chassis::laser";
	if ((raySensor = boost::dynamic_pointer_cast<sensors::RaySensor>(
			sensors::SensorManager::Instance()->GetSensor(sensorName))) == NULL) {
		std::cerr << "COULD NOT FIND LASER SENSOR: " + sensorName + '\n';
		return NULL;
	}

#ifdef DEBUG
	std::cout << "[GcVehicle] Ray sensor loading complete." << std::endl;
#endif /* debug information */

// create a GcVehicle
	auto gcVeh = std::make_shared<GcVehicle>(m_vehId, m_terrain, veh, powertrain,
			tires, driver, m_maxSpeed, raySensor, gazeboVehicle, gazeboWheels);

// subscribe the GcVehicle to ros
	auto opt = ros::SubscribeOptions::create<std_msgs::Float64>(
			"/track_point" + id, 1, boost::bind(&GcVehicle::UpdateDriver, gcVeh, _1),
			ros::VoidPtr(), m_queue);
	m_lastSub = m_handle->subscribe(opt);

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
