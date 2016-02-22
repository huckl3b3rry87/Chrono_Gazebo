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

#include "GcVehicle.hh"

#include <gazebo/math/Vector3.hh>
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
		const double maxSpeed, const sensors::RaySensorPtr raySensor,
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
math::Pose GcVehicle::getPose(const ChVector<> vec,
		const ChQuaternion<> quat) {
	return math::Pose(math::Vector3(vec.x, vec.y, vec.z),
			math::Quaternion(quat.e0, quat.e1, quat.e2, quat.e3));
}

//advance
void GcVehicle::advance() {
	std::cout << "advancing vehicle " << id << std::endl;
	std::vector<double> ranges;
	raySensor->GetRanges(ranges);
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

	std::cout << 1 << std::endl;

	driver->SetDesiredSpeed(targetSpeed);

	std::cout << 1.1 << std::endl;

	double brakingInput = driver->GetBraking();
	steeringInput = driver->GetSteering();
	double throttleInput = driver->GetThrottle();
	double powertrainTorque = powertrain->GetOutputTorque();
	double driveshaftSpeed = vehicle->GetDriveshaftSpeed();
	vehicle::TireForces tireForces;
	vehicle::WheelStates wheelStates;
	for (int i = 0; i < numWheels; i++) {

		std::cout << 1.2 << std::endl;

		tireForces[i] = tires[i]->GetTireForce();

		std::cout << 1.21 << std::endl;

		wheelStates[i] = vehicle->GetWheelState(i);

		std::cout << 1.22 << std::endl;
	}

	std::cout << 2 << std::endl;

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

	std::cout << 3 << std::endl;

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
