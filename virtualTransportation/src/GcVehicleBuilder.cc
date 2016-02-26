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
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/driver/ChPathFollowerDriver.h>
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

GcVehicleBuilder::GcVehicleBuilder(physics::WorldPtr world, ChSystem *chsys,
		ChSharedPtr<vehicle::RigidTerrain> terrain, const double stepSize) :
		world(world), chsys(chsys), terrain(terrain), stepSize(stepSize) {
}

boost::shared_ptr<GcVehicle> GcVehicleBuilder::buildGcVehicle() {

	const std::string id = std::to_string(vehId);
#ifdef DEBUG
	std::cout << "[GcVehicle] Start building vehicle " << id << "." << std::endl;
#endif /* debug information */

	// -- Chrono part --

	// create and initialize a Chrono vehicle model
	auto veh = ChSharedPtr<vehicle::WheeledVehicle>(
			new vehicle::WheeledVehicle(chsys, vehicleFile));
	veh->Initialize(coordsys);
#ifdef DEBUG
	std::cout << "[GcVehicle] Vehicle initialzied." << std::endl;
#endif /* debug information */

	// create and initialize a powertrain
	auto powertrain = ChSharedPtr<vehicle::SimplePowertrain>(
			new vehicle::SimplePowertrain(powertrainFile));
	powertrain->Initialize();
#ifdef DEBUG
	std::cout << "[GcVehicle] Powertrain initialized." << std::endl;
#endif /* debug information */

	// create and initialize the tires
	const int numWheels = 2 * veh->GetNumberAxles();
	auto tires = std::vector<ChSharedPtr<vehicle::RigidTire> >(numWheels);
	for (int i = 0; i < numWheels; i++) {
		//create the tires from the tire file
		tires[i] = ChSharedPtr<vehicle::RigidTire>(
				new vehicle::RigidTire(tireFile));
		tires[i]->Initialize(veh->GetWheelBody(i));
#ifdef DEBUG
		std::cout << "[GcVehicle] Tire " << i << " initialized." << std::endl;
#endif /* debug information */
	}

// create path follower
	auto driver = ChSharedPtr<vehicle::ChPathFollowerDriver>(
			new vehicle::ChPathFollowerDriver(*veh, steerFile, speedFile, path,
					std::string("my_path"), 0.0));
#ifdef DEBUG
	std::cout << "[GcVehicle] Path follower driver loading completes." << std::endl;
#endif /* debug information */

// -- Gazebo part --

	physics::ModelPtr gazeboVehicle;
	std::vector<physics::ModelPtr> gazeboWheels(numWheels);
	sensors::RaySensorPtr raySensor;

// retrieve vehicle model from Gazebo
	const std::string vehicleName = "vehicle" + id;
	if ((gazeboVehicle = world->GetModel(vehicleName)) == NULL) {
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
		if ((wheelPtr = world->GetModel(wheelName)) != NULL) {
			gazeboWheels[i] = wheelPtr;
		} else {
			std::cerr << "COULD NOT FIND GAZEBO MODEL: " + wheelName + '\n';
			return NULL;
		}
#ifdef DEBUG
		std::cout << "[GcVehicle] Wheel model " << id << " loading complete" << std::endl;
#endif /* debug information */
	}

// retrieve sensor model from Gazebo
	const std::string sensorName = world->GetName() + "::" + vehicleName
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
	auto gcVeh = boost::shared_ptr<GcVehicle>(
			new GcVehicle(vehId, terrain, veh, powertrain, tires, driver, maxSpeed,
					raySensor, gazeboVehicle, gazeboWheels, stepSize));

// subscribe the GcVehicle to ros
	auto opt = ros::SubscribeOptions::create<std_msgs::Float64>(
			"/track_point" + std::to_string(vehId), 1,
			boost::bind(&GcVehicle::updateDriver, gcVeh, _1), ros::VoidPtr(), queue);
	lastSub = handle->subscribe(opt);
#ifdef DEBUG
	std::cout << "[GcVehicle] Vehicle subscribed to ROS." << std::endl;
#endif /* debug information */

// increase the vehicle id
	vehId++;

#ifdef DEBUG
	std::cout << "[GcVehicle] Vehicle " << id << " building complete." << std::endl;
#endif /* debug information */
	return gcVeh;
}

ros::Subscriber &GcVehicleBuilder::getLastRosSubscriber() {
	return lastSub;
}

void GcVehicleBuilder::setVehicleFile(const std::string &vehicleFile) {
	this->vehicleFile = vehicle::GetDataFile(vehicleFile);
}

void GcVehicleBuilder::setPowertrainFile(const std::string &powertrainFile) {
	this->powertrainFile = vehicle::GetDataFile(powertrainFile);
}

void GcVehicleBuilder::setTireFile(const std::string &tireFile) {
	this->tireFile = vehicle::GetDataFile(tireFile);
}

void GcVehicleBuilder::setSteerCtrlFile(const std::string &steerFile) {
	this->steerFile = vehicle::GetDataFile(steerFile);
}

void GcVehicleBuilder::setSpeedCtrlFile(const std::string &speedFile) {
	this->speedFile = vehicle::GetDataFile(speedFile);
}

void GcVehicleBuilder::setInitCoordsys(const ChCoordsys<> &coordsys) {
	this->coordsys = coordsys;
}

void GcVehicleBuilder::setMaxSpeed(const double maxSpeed) {
	this->maxSpeed = maxSpeed;
}

void GcVehicleBuilder::setPath(ChBezierCurve * const path) {
	this->path = path;
}

void GcVehicleBuilder::setNodeHandler(ros::NodeHandle * const handle) {
	this->handle = handle;
}

void GcVehicleBuilder::setCallbackQueue(ros::CallbackQueue * const queue) {
	this->queue = queue;
}
