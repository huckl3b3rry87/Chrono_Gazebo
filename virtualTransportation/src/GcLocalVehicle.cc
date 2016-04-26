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

bool GcLocalVehicle::Init() {
	if (m_initialized)
		return true;
	const std::string vehicleName = "vehicle_" + std::to_string(m_id);
	const std::string sensorName = m_world->GetName() + "::" + vehicleName
			+ "::chassis::";

	if ((m_raySensor = boost::dynamic_pointer_cast<sensors::RaySensor>(
			sensors::SensorManager::Instance()->GetSensor(sensorName + "laser")))
			== NULL) {
		std::cerr << "COULD NOT FIND LASER SENSOR: " + sensorName + '\n';
		return false;
	}

	if ((m_gazeboVehicle = m_world->GetModel(vehicleName)) == NULL) {
		std::cerr << "COULD NOT FIND GAZEBO MODEL: " + vehicleName + '\n';
		return false;
	}

	m_initialized = true;
	for (int i = 0; i < m_numWheels; i++) {
		m_gazeboWheels.push_back(
				m_gazeboVehicle->GetLink("wheel_" + std::to_string(i)));
		assert(m_gazeboWheels[i] != NULL);
	}

	return true;
}

//void GcLocalVehicle::UpdateDriver(const std_msgs::Float64::ConstPtr& _msg) {
//	m_steeringInput = _msg->data;
//}

//advance

void GcLocalVehicle::Synchronize(double time) {
	std::vector<double> ranges;
	m_raySensor->SetActive(false);
	m_raySensor->GetRanges(ranges);
	m_raySensor->SetActive(true);
	//double center = steeringInput * 50 + 50;
	m_currDist = 100000.0;
	for (int i = 0; i < ranges.size(); i++) {
		//if (ranges[i] * sin(abs(i - center) * 3.14159 / 180.0) <= 2.5) {
		if (ranges[i] < m_currDist)
			m_currDist = ranges[i];
		//}
	}
	m_driver->SetCurrentDistance(m_currDist);

	auto sent = m_driver->GetSteeringController().GetSentinelLocation();
	sent.z = 0;
	auto target = m_driver->GetSteeringController().GetTargetLocation();
	target.z = 0;
	auto err = target - sent;
	auto sent_vec = (sent - m_vehicle->GetChassisPos());
	sent_vec.z = 0;
	sent_vec.Normalize();
	auto target_vec = (target - m_vehicle->GetChassisPos());
	target_vec.z = 0;
	target_vec.Normalize();
	double target_ang = Vdot(Vcross(sent_vec, target_vec), ChVector<>(0, 0, 1));
	auto tang = m_driver->GetSteeringController().GetTargetDirection();
	tang.z = 0;
	tang.Normalize();
	double tang_ang = Vdot(Vcross(sent_vec, tang), ChVector<>(0, 0, 1));

	double brakingInput = m_driver->GetBraking();
	m_steeringInput = m_driver->GetSteering() * 0.6;
	double throttleInput = m_driver->GetThrottle();
	double powertrainTorque = m_powertrain->GetOutputTorque();
	double driveshaftSpeed = m_vehicle->GetDriveshaftSpeed();
	vehicle::TireForces tireForces(m_numWheels);
	vehicle::WheelStates wheelStates(m_numWheels);
	for (int i = 0; i < m_numWheels; i++) {
		tireForces[i] = m_tires[i]->GetTireForce();
		wheelStates[i] = m_vehicle->GetWheelState(i);
	}

	double factor = m_vehicle->GetVehicleSpeed() / m_maxSpeed;
	throttleInput += (rand() / (double) RAND_MAX - 0.5) * 0.2;
	throttleInput = std::max(0.0,
			std::min(throttleInput, 0.6 - fabs(m_steeringInput) * factor));
	brakingInput += std::max(0.0, fabs(m_steeringInput) * factor);
	brakingInput = std::min(1.0, brakingInput);

	m_driver->Synchronize(time);
	for (int i = 0; i < m_numWheels; i++)
		m_tires[i]->Synchronize(time, wheelStates[i], *m_terrain);
	m_powertrain->Synchronize(time, throttleInput, driveshaftSpeed);
	m_vehicle->Synchronize(time, m_steeringInput, brakingInput, powertrainTorque,
			tireForces);

//	err.z = 0;
//	printf(
//			"sent: %.2f, %.2f    target: %.2f, %.2f    tang: %.2f, %.2f    len: %.2f    target_ang: %.2f    tang_ang: %.2f    steering: %.2f    brake:%.2f    throttle:%.2f\n",
//			sent.x, sent.y, target.x, target.y, tang.x, tang.y, err.Length(),
//			target_ang, tang_ang, m_steeringInput, brakingInput, throttleInput);
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
	if (m_id == 0) {
		auto camLink = m_gazeboVehicle->GetLink("camRef");
		auto pose = camLink->GetRelativePose();
		pose.pos += gazebo::math::Vector3(1.0, -0.5, 1.0) * 0.006
				/ std::cbrt(m_vehicle->GetSystem()->GetChTime() + 1.0);
		camLink->SetRelativePose(pose);
	}
	////////////////DEBUG LINE////////////
//	std::cout<<"Vehicle "<<this->id<<": \t"<<"range: "<<minRange<<"\tVelocity: "<<vehicle->GetVehicleSpeedCOM()<<"\tbrakeInput: "
//				<<brakingInput<<"\twheelSpeed: "<<vehicle->GetWheelOmega(0)<<"\tbrakeTorque: "<<vehicle->GetBrake(0)->GetBrakeTorque()<<"\t";
}

