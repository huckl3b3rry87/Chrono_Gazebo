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
// Authors: Asher Elmquist
// =============================================================================
//
//gcVehicle defines a vehicle based on chrono_vehicle and gazebo relying on the
//dynamics and definitions provided by chrono and the world geometry, sensors,
//and visuals within gazebo. This class enables connection to gazebo sensors
//and chrono vehicle
//
// =============================================================================

#include <boost/bind/arg.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/bind_mf_cc.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono_vehicle/ChDriver.h>
#include <chrono_vehicle/ChSubsysDefs.h>
#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/powertrain/ChSimplePowertrain.h>
#include <chrono_vehicle/wheeled_vehicle/ChTire.h>
#include <gazebo/common/SingletonT.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gcVehicle.hh>
#include <physics/ChObject.h>
#include <physics/ChSystem.h>
#include <ros/callback_queue.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/subscribe_options.h>
#include <cmath>
#include <cstdlib>
#include <iostream>

//includes

//namespace(s)

//constructor
GcVehicle::GcVehicle(const int id,
		const ChSharedPtr<vehicle::RigidTerrain> terrain,
		const ChSharedPtr<vehicle::WheeledVehicle> vehicle,
		const ChSharedPtr<vehicle::SimplePowertrain> powertrain,
		const std::vector<ChSharedPtr<vehicle::RigidTire> > &tires,
		ChSharedPtr<vehicle::ChPathFollowerDriver> driver,
		const double maxSpeed, const sensors::RaySensor &raySensor,
		const physics::ModelPtr gazeboVehicle,
		const std::vector<physics::ModelPtr> &gazeboWheels,
		const double stepSize) :
		id(id), terrain(terrain), vehicle(vehicle), powertrain(powertrain), tires(
				tires), driver(driver), maxSpeed(maxSpeed), raySensor(
				raySensor), gazeboVehicle(gazeboVehicle), gazeboWheels(
				gazeboWheels), steeringInput(0), stepSize(stepSize) {
	numWheels = vehicle->GetNumberAxles() * 2;
}

ChSharedPtr<vehicle::WheeledVehicle> GcVehicle::getVehicle() {
	return vehicle;
}

void GcVehicle::updateDriver(const std_msgs::Float64::ConstPtr& _msg) {
	steeringInput = _msg->data;
}

//private listener functions
math::Pose &GcVehicle::getPose(const ChVector<> vec,
		const ChQuaternion<> quat) {
	return math::Pose(math::Vector3(vec.x, vec.y, vec.z),
			math::Quaternion(quat.e0, quat.e1, quat.e2, quat.e3));
}

//advance
void GcVehicle::advance() {
	std::vector<double> ranges;
	raySensor.GetRanges(ranges);
	double center = steeringInput * 50 + 50;
	double minRange = 100000.0;
	for (int i = 0; i < ranges.size(); i++) {
		if (ranges[i] * sin(abs(i - center) * 3.14159 / 180.0) <= 2.5) {
			if (ranges[i] < minRange)
				minRange = ranges[i];
		}
	}
	double targetSpeed = maxSpeed;
	if (minRange < 10000.0) {
		//linear so 6m/s at 20m at 0m/s at 2.5m
		targetSpeed = .343 * minRange - .857;
	}

	driver->SetDesiredSpeed(targetSpeed);
	double brakingInput = driver->GetBraking();
	steeringInput = driver->GetSteering();
	double throttleInput = driver->GetThrottle();
	double powertrainTorque = powertrain->GetOutputTorque();
	double driveshaftSpeed = vehicle->GetDriveshaftSpeed();
	vehicle::TireForces tireForces;
	vehicle::WheelStates wheelStates;
	for (int i = 0; i < numWheels; i++) {
		tireForces[i] = tires[1]->GetTireForce();
		wheelStates[i] = vehicle->GetWheelState(i);
	}

	double time = vehicle->GetSystem()->GetChTime();
	driver->Update(time);
	powertrain->Update(time, throttleInput, driveshaftSpeed);
	vehicle->Update(time, steeringInput, brakingInput, powertrainTorque,
			tireForces);
	for (int i = 0; i < numWheels; i++)
		tires[i]->Update(time, wheelStates[i], *terrain);

	// Advance simulation for one timestep for all modules
	driver->Advance(stepSize);
	powertrain->Advance(stepSize);
	vehicle->Advance(stepSize);
	for (int i = 0; i < numWheels; i++) {
		tires[i]->Advance(stepSize);
	}

	//Communication and updates between Chrono and Gazebo
	gazeboVehicle->SetWorldPose(
			getPose(vehicle->GetChassisPos(), vehicle->GetChassisRot()),
			"link");
	for (int i = 0; i < numWheels; i++) {
		gazeboWheels[i]->SetWorldPose(
				getPose(vehicle->GetWheelPos(i), vehicle->GetWheelRot(i)),
				"link");
	}
}

GcVehicleBuilder::GcVehicleBuilder(physics::WorldPtr world, ChSystem *chsys,
		ChSharedPtr<vehicle::RigidTerrain> terrain, const double stepSize) :
		world(world), chsys(chsys), terrain(terrain), stepSize(stepSize) {
}

GcVehicle &GcVehicleBuilder::buildGcVehicle() {

	const std::string id = std::to_string(vehId);

	auto veh = ChSharedPtr<vehicle::WheeledVehicle>(
			new vehicle::WheeledVehicle(chsys, vehicleFile));
	veh->Initialize(coordsys);
	const int numWheels = 2 * veh->GetNumberAxles();

	auto driver = ChSharedPtr<vehicle::ChPathFollowerDriver>(
			new vehicle::ChPathFollowerDriver(*veh, steerFile, speedFile, path,
					std::string("my_path"), 0.0));

	auto powertrain = ChSharedPtr<vehicle::SimplePowertrain>(
			new vehicle::SimplePowertrain(powertrainFile));
	powertrain->Initialize();

	auto tires = std::vector<ChSharedPtr<vehicle::RigidTire>>(numWheels);

	for (int i = 0; i < numWheels; i++) {
		//create the tires from the tire file
		tires[i] = ChSharedPtr<vehicle::RigidTire>(
				new vehicle::RigidTire(tireFile));
		tires[i]->Initialize(veh->GetWheelBody(i));
	}

	physics::ModelPtr gazeboVehicle;
	std::vector<physics::ModelPtr> gazeboWheels(numWheels);
	sensors::SensorPtr raySensor;

	if ((gazeboVehicle = world->GetModel("vehicle" + id)) == NULL) {
		std::cerr << "COULD NOT FIND GAZEBO MODEL: vehicle" + id + '\n';
	}
	for (int i = 1; i < numWheels; i++) {
		physics::ModelPtr wheelPtr;
		const std::string wheelName = "wheel" + id + "_" + std::to_string(i);
		if ((wheelPtr = world->GetModel(wheelName)) != NULL) {
			gazeboWheels.push_back(wheelPtr);
		} else {
			std::cerr << "COULD NOT FIND GAZEBO MODEL: " + wheelName + '\n';
		}
	}
	const std::string sensorName = world->GetName() + "::vehicle" + id
			+ "::chassis0::laser";
	if ((raySensor = sensors::SensorManager::Instance()->GetSensor(sensorName))
			== NULL) {
		std::cerr << "COULD NOT FIND LASER SENSOR " + id + '\n';
	}

	GcVehicle gcVeh(vehId, terrain, veh, powertrain, tires, driver, maxSpeed,
			raySensor, gazeboVehicle, gazeboWheels, stepSize);

	auto opt = ros::SubscribeOptions::create<std_msgs::Float64>(
			"/track_point" + std::to_string(vehId), 1,
			boost::bind(&GcVehicle::updateDriver, gcVeh, _1), ros::VoidPtr(),
			queue);
	lastSub = handle->subscribe(opt);

	vehId++;

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

void GcVehicleBuilder::setPath(const ChBezierCurve *path) {
	this->path = path;
}

void GcVehicleBuilder::setNodeHandler(const ros::NodeHandle *handle) {
	this->handle = handle;
}

void GcVehicleBuilder::setCallbackQueue(const ros::CallbackQueue *queue) {
	this->queue = queue;
}
