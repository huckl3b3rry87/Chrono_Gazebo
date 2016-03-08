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

#include <chrono_vehicle/driver/ChPathFollowerACCDriver.h>
#include <chrono_vehicle/powertrain/ChSimplePowertrain.h>
#include <chrono_vehicle/ChTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h>
#include <core/ChQuaternion.h>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>

class GcVehicle {

public:

	typedef std::shared_ptr<chrono::vehicle::ChTerrain> ChTerrainPtr;
	typedef std::shared_ptr<chrono::vehicle::ChWheeledVehicle> ChWheeledVehiclePtr;
	typedef std::shared_ptr<chrono::vehicle::ChPowertrain> ChPowertrainPtr;
	typedef std::shared_ptr<chrono::vehicle::ChRigidTire> ChRigidTirePtr;
	typedef std::shared_ptr<chrono::vehicle::ChPathFollowerACCDriver> ChDriverPtr;

	// constructor
	GcVehicle(const int id, const ChTerrainPtr terrain,
			const ChWheeledVehiclePtr vehicle, const ChPowertrainPtr powertrain,
			const std::vector<ChRigidTirePtr> &tires, const ChDriverPtr driver,
			const double maxSpeed, const gazebo::sensors::RaySensorPtr raySensor,
			const gazebo::physics::ModelPtr gazeboVehicle,
			const std::vector<gazebo::physics::ModelPtr> &gazeboWheels);

	// advance
	void UpdateDriver(const std_msgs::Float64::ConstPtr& _msg);
	void Synchronize(const double time);
	void Advance(const double step);

	// other functions
	ChWheeledVehiclePtr GetVehicle() {
		return m_vehicle;
	}
	int GetId() {
		return m_id;
	}

private:
	// helper functions
	gazebo::math::Pose GetPose(const chrono::ChVector<> vec,
			const chrono::ChQuaternion<> quat);

private:
	int m_id;
	int m_numWheels;

	// chrono components
	ChTerrainPtr m_terrain;
	ChWheeledVehiclePtr m_vehicle;
	ChPowertrainPtr m_powertrain;
	std::vector<ChRigidTirePtr> m_tires;
	ChDriverPtr m_driver;

	// gazebo components
	gazebo::sensors::RaySensorPtr m_raySensor;
	gazebo::physics::ModelPtr m_gazeboVehicle;
	std::vector<gazebo::physics::ModelPtr> m_gazeboWheels;

	// other members
	double m_steeringInput;
	double m_maxSpeed;
};

#endif /* SRC_GCVEHICLE_HH_ */
