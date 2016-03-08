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

#include "GcVehicle.hh"

#include <cmath>
#include <cstdlib>

using namespace chrono;
using namespace gazebo;

//constructor
GcVehicle::GcVehicle(const int id, const GcVehicle::ChTerrainPtr terrain,
		const GcVehicle::ChWheeledVehiclePtr vehicle,
		const GcVehicle::ChPowertrainPtr powertrain,
		const std::vector<GcVehicle::ChRigidTirePtr> &tires,
		const GcVehicle::ChDriverPtr driver, const double maxSpeed,
		const sensors::RaySensorPtr raySensor,
		const physics::ModelPtr gazeboVehicle,
		const std::vector<physics::ModelPtr> &gazeboWheels) :
		m_id(id), m_terrain(terrain), m_vehicle(vehicle), m_powertrain(powertrain), m_tires(
				tires), m_driver(driver), m_maxSpeed(maxSpeed), m_raySensor(raySensor), m_gazeboVehicle(
				gazeboVehicle), m_gazeboWheels(gazeboWheels), m_steeringInput(0) {
	m_numWheels = vehicle->GetNumberAxles() * 2;
}

void GcVehicle::UpdateDriver(const std_msgs::Float64::ConstPtr& _msg) {
	m_steeringInput = _msg->data;
}

//private functions
math::Pose GcVehicle::GetPose(const ChVector<> vec, const ChQuaternion<> quat) {
	return math::Pose(math::Vector3(vec.x, vec.y, vec.z),
			math::Quaternion(quat.e0, quat.e1, quat.e2, quat.e3));
}

//advance

void GcVehicle::Synchronize(const double time) {
	std::vector<double> ranges;
	m_raySensor->SetActive(false);
	m_raySensor->GetRanges(ranges);
	m_raySensor->SetActive(true);
	//double center = steeringInput * 50 + 50;
	double minRange = 100000.0;
	for (int i = 0; i < ranges.size(); i++) {
		//if (ranges[i] * sin(abs(i - center) * 3.14159 / 180.0) <= 2.5) {
		if (ranges[i] < minRange)
			minRange = ranges[i];
		//}
	}

	m_driver->SetCurrentDistance(minRange);

	double brakingInput = m_driver->GetBraking();
	m_steeringInput = m_driver->GetSteering();
	double throttleInput = m_driver->GetThrottle();
	double powertrainTorque = m_powertrain->GetOutputTorque();
	double driveshaftSpeed = m_vehicle->GetDriveshaftSpeed();
	vehicle::TireForces tireForces(m_numWheels);
	vehicle::WheelStates wheelStates(m_numWheels);
	for (int i = 0; i < m_numWheels; i++) {
		tireForces[i] = m_tires[i]->GetTireForce();
		wheelStates[i] = m_vehicle->GetWheelState(i);
	}

	m_driver->Synchronize(time);
	for (int i = 0; i < m_numWheels; i++)
		m_tires[i]->Synchronize(time, wheelStates[i], *m_terrain);
	m_powertrain->Synchronize(time, throttleInput, driveshaftSpeed);
	m_vehicle->Synchronize(time, m_steeringInput, brakingInput, powertrainTorque,
			tireForces);
}

void GcVehicle::Advance(const double step) {
	// Advance simulation for one timestep for all modules
	m_driver->Advance(step);
	for (int i = 0; i < m_numWheels; i++) {
		m_tires[i]->Advance(step);
	}
	m_powertrain->Advance(step);
	// Since advance a vehicle will advance the whole system, it is advanced in the top level outside the loop.

	//Communication and updates between Chrono and Gazebo
	m_gazeboVehicle->SetWorldPose(
			GetPose(m_vehicle->GetChassisPos(), m_vehicle->GetChassisRot()), "link");
	auto rot = m_vehicle->GetChassisRot();
	for (int i = 0; i < m_numWheels; i++) {
		m_gazeboWheels[i]->SetWorldPose(
				GetPose(m_vehicle->GetWheelPos(i), m_vehicle->GetWheelRot(i)), "link");
	}
	////////////////DEBUG LINE////////////
//	std::cout<<"Vehicle "<<this->id<<": \t"<<"range: "<<minRange<<"\tVelocity: "<<vehicle->GetVehicleSpeedCOM()<<"\tbrakeInput: "
//				<<brakingInput<<"\twheelSpeed: "<<vehicle->GetWheelOmega(0)<<"\tbrakeTorque: "<<vehicle->GetBrake(0)->GetBrakeTorque()<<"\t";
}

