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

#include "GcLocalVehicle.hh"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono_vehicle/ChDriver.h>
#include <chrono_vehicle/ChPowertrain.h>
#include <chrono_vehicle/ChSubsysDefs.h>
#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/driver/ChPathFollowerACCDriver.h>
#include <chrono_vehicle/wheeled_vehicle/ChTire.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h>
#include <gazebo/common/SingletonT.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <algorithm>
#include <cassert>
#include <cstdbool>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "GcUtil.hh"

using namespace chrono;
using namespace gazebo;

//constructor
GcLocalVehicle::GcLocalVehicle(int id,
		const gc::ChTerrainPtr terrain,
		const gc::ChWheeledVehiclePtr vehicle,
		const gc::ChPowertrainPtr powertrain,
		const std::vector<gc::ChRigidTirePtr> &tires,
		const gc::ChDriverPtr driver,
		double maxSpeed,
		const physics::WorldPtr world) :
		m_id(id), m_terrain(terrain), m_vehicle(vehicle), m_powertrain(powertrain),
				m_tires(tires), m_driver(driver), m_maxSpeed(maxSpeed),
				m_steeringInput(0), m_world(world) {
	m_currDist = 100000.0;
	m_numWheels = vehicle->GetNumberAxles() * 2;
}

bool GcLocalVehicle::InitSensor() {
	std::vector<double> ranges;
	m_raySensor->Update(false);
	m_raySensor->SetActive(false);
	m_raySensor->Ranges(ranges);
	m_raySensor->SetActive(true);

	for (double range : ranges) {
		if (range < 100000) {
			return true;
		}
	}
	return false;
}

bool GcLocalVehicle::Init() {
	if (m_initialized)
		return true;
	const std::string vehicleName = "vehicle_" + std::to_string(m_id);
	std::string sensorName = m_world->GetName() + "::" + vehicleName
			+ "::chassis::";

	//sensorName = sensorName + "laser";
	std::cout<<"SensorName: "<<sensorName<<std::endl;
	if ((m_raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(
			sensors::SensorManager::Instance()->GetSensor(sensorName + "laser")))
			== NULL) {
		std::cerr << "COULD NOT FIND LASER SENSOR: " + sensorName + '\n';
		return false;
	}

	if ((m_gazeboVehicle = m_world->GetModel(vehicleName)) == NULL) {
		std::cerr << "COULD NOT FIND GAZEBO MODEL: " + vehicleName + '\n';
		return false;
	}

	for (int i = 0; i < m_numWheels; i++) {
		m_gazeboWheels.push_back(
				m_gazeboVehicle->GetLink("wheel_" + std::to_string(i)));
		assert(m_gazeboWheels[i] != NULL);
	}

	m_gazeboVehicle->SetWorldPose(
			gc::GetGcPose(m_vehicle->GetChassisPos(), m_vehicle->GetChassisRot()));
	auto rot = m_vehicle->GetChassisRot();
	for (int i = 0; i < m_numWheels; i++) {
		m_gazeboWheels[i]->SetWorldPose(
				gc::GetGcPose(m_vehicle->GetWheelPos(i), m_vehicle->GetWheelRot(i)));
	}

	m_initialized = true;
	return true;
}

//void GcLocalVehicle::UpdateDriver(const std_msgs::Float64::ConstPtr& _msg) {
//	m_steeringInput = _msg->data;
//}

//advance

void GcLocalVehicle::Synchronize(double time) {
	std::vector<double> ranges;
	m_raySensor->SetActive(false);
	m_raySensor->Ranges(ranges);
	m_raySensor->SetActive(true);
	//double center = steeringInput * 50 + 50;
	m_currDist = 100000.0;
	for (int i = 0; i < ranges.size(); i++) {
		//if (ranges[i] * sin(abs(i - center) * 3.14159 / 180.0) <= 2.5) {
		if (ranges[i] < m_currDist)
			m_currDist = ranges[i];
		//}
	}
	std::cout<<"vehicle"<<m_id<<" range: "<<m_currDist<<std::endl;
	m_driver->SetCurrentDistance(m_currDist);

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

void GcLocalVehicle::Advance(double step) {
	// Advance simulation for one timestep for all modules
	m_driver->Advance(step);
	for (int i = 0; i < m_numWheels; i++) {
		m_tires[i]->Advance(step);
	}
	m_powertrain->Advance(step);
	// Since advance a vehicle will advance the whole system, it is advanced in the top level outside the loop.
	//Communication and updates between Chrono and Gazebo

	m_gazeboVehicle->SetWorldPose(
			gc::GetGcPose(m_vehicle->GetChassisPos(), m_vehicle->GetChassisRot()));
	auto rot = m_vehicle->GetChassisRot();
	for (int i = 0; i < m_numWheels; i++) {
		m_gazeboWheels[i]->SetWorldPose(
				gc::GetGcPose(m_vehicle->GetWheelPos(i), m_vehicle->GetWheelRot(i)));
	}
	////////////////DEBUG LINE////////////
//	std::cout<<"Vehicle "<<this->id<<": \t"<<"range: "<<minRange<<"\tVelocity: "<<vehicle->GetVehicleSpeedCOM()<<"\tbrakeInput: "
//				<<brakingInput<<"\twheelSpeed: "<<vehicle->GetWheelOmega(0)<<"\tbrakeTorque: "<<vehicle->GetBrake(0)->GetBrakeTorque()<<"\t";
}

