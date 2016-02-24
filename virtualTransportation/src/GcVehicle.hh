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

class GcVehicle {

	typedef chrono::ChSharedPtr<chrono::vehicle::RigidTerrain> ChTerrainPtr;
	typedef chrono::ChSharedPtr<chrono::vehicle::WheeledVehicle> ChWheeledVehiclePtr;
	typedef chrono::ChSharedPtr<chrono::vehicle::SimplePowertrain> ChSimplePowertrainPtr;
	typedef chrono::ChSharedPtr<chrono::vehicle::RigidTire> ChRigidTirePtr;
	typedef chrono::ChSharedPtr<chrono::vehicle::ChPathFollowerDriver> ChPathFollowerDriverPtr;

public:
	// constructor
	GcVehicle(const int id, const ChTerrainPtr terrain,
			const ChWheeledVehiclePtr vehicle, const ChSimplePowertrainPtr powertrain,
			const std::vector<ChRigidTirePtr> &tires,
			const ChPathFollowerDriverPtr driver, const double maxSpeed,
			const gazebo::sensors::RaySensorPtr raySensor,
			const gazebo::physics::ModelPtr gazeboVehicle,
			const std::vector<gazebo::physics::ModelPtr> &gazeboWheels,
			const double stepSize);

	// advance
	void updateDriver(const std_msgs::Float64::ConstPtr& _msg);
	void advance();

	// other functions
	ChWheeledVehiclePtr getVehicle();
	int getId();

private:
	// helper functions
	gazebo::math::Pose getPose(const chrono::ChVector<> vec,
			const chrono::ChQuaternion<> quat);

private:
	int id;
	int numWheels;

	// chrono components
	ChTerrainPtr terrain;
	ChWheeledVehiclePtr vehicle;
	ChSimplePowertrainPtr powertrain;
	std::vector<ChRigidTirePtr> tires;
	ChPathFollowerDriverPtr driver;

	// gazebo components
	gazebo::sensors::RaySensorPtr raySensor;
	gazebo::physics::ModelPtr gazeboVehicle;
	std::vector<gazebo::physics::ModelPtr> gazeboWheels;

	// other members
	double steeringInput;
	double maxSpeed;
	double stepSize;
};

#endif /* SRC_GCVEHICLE_HH_ */
