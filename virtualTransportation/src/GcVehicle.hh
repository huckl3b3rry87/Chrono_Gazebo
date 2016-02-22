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
//and visuals within gazebo
//
// =============================================================================

#ifndef SRC_GCVEHICLE_HH_
#define SRC_GCVEHICLE_HH_

//includes

#include <chrono_vehicle/driver/ChPathFollowerDriver.h>
#include <chrono_vehicle/powertrain/SimplePowertrain.h>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/tire/RigidTire.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <core/ChQuaternion.h>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>

//namespace(s)
using namespace chrono;
using namespace gazebo;

class GcVehicle {
public:
	//constructor
	GcVehicle(const int id, const ChSharedPtr<vehicle::RigidTerrain> terrain,
			const ChSharedPtr<vehicle::WheeledVehicle> vehicle,
			const ChSharedPtr<vehicle::SimplePowertrain> powertrain,
			const std::vector<ChSharedPtr<vehicle::RigidTire> > &tires,
			ChSharedPtr<vehicle::ChPathFollowerDriver> driver,
			const double maxSpeed, const sensors::RaySensorPtr raySensor,
			const physics::ModelPtr gazeboVehicle,
			const std::vector<physics::ModelPtr> &gazeboWheels,
			const double stepSize);

	ChSharedPtr<vehicle::WheeledVehicle> getVehicle();

	//advance
	void updateDriver(const std_msgs::Float64::ConstPtr& _msg);

	void advance();

	//other functions
	std::string &getName();

	int getId();
private:
	math::Pose getPose(const ChVector<> vec, const ChQuaternion<> quat);

private:
	ChSharedPtr<vehicle::RigidTerrain> terrain;
	ChSharedPtr<vehicle::WheeledVehicle> vehicle;
	ChSharedPtr<vehicle::SimplePowertrain> powertrain;
	std::vector<ChSharedPtr<vehicle::RigidTire> > tires;
	ChSharedPtr<vehicle::ChPathFollowerDriver> driver;

	int numWheels;

	sensors::RaySensorPtr raySensor;
	physics::ModelPtr gazeboVehicle;
	std::vector<physics::ModelPtr> gazeboWheels;

	double steeringInput;

	double maxSpeed;

	int id;

	double stepSize;
};

#endif /* SRC_GCVEHICLE_HH_ */
