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

#ifndef SRC_GCLOCALVEHICLE_HH_
#define SRC_GCLOCALVEHICLE_HH_

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <vector>

#include "GcTypes.hh"
#include "GcVehicle.hh"

class GcLocalVehicle : public GcVehicle {

public:

	// constructor
	GcLocalVehicle(int id,
			const gc::ChTerrainPtr terrain,
			const gc::ChWheeledVehiclePtr vehicle,
			const gc::ChPowertrainPtr powertrain,
			const std::vector<gc::ChRigidTirePtr> &tires,
			const gc::ChDriverPtr driver,
			double maxSpeed,
			const gazebo::physics::WorldPtr world);

	// advance
//	void UpdateDriver(const std_msgs::Float64::ConstPtr& _msg);

	/* Synchronize the driver and other Chrono components */
	void Synchronize(double time) override;

	/*
	 * Advance Chrono components except for the vehicle itself. It will be advanced
	 * with the ChSystem outside the update loop for all vehicles.
	 */
	void Advance(double step) override;

	// other functions
	gc::ChWheeledVehiclePtr GetVehicle() {
		return m_vehicle;
	}

	/*
	 * Initialized the vehicle models (because they are added after the world plugin
	 * has been loaded). This functional could be called multiple times.
	 */
	bool Init() override;


	/*
	 * Try to get sensor ranges. Return true iff some range value is not infinite.
	 * This might not work if a vehicle has no other vehicle in front of it. Need
	 * more consideration on this function.
	 */
	bool InitSensor() override;

	double GetCurrDist() {
		return m_currDist;
	}

	int GetId() const override{
		return m_id;
	}

private:
	int m_id;
	int m_numWheels;
	bool m_initialized = false;

	// chrono components
	gc::ChTerrainPtr m_terrain;
	gc::ChWheeledVehiclePtr m_vehicle;
	gc::ChPowertrainPtr m_powertrain;
	std::vector<gc::ChRigidTirePtr> m_tires;
	gc::ChDriverPtr m_driver;

	// gazebo components
	gazebo::physics::WorldPtr m_world;
	gazebo::sensors::RaySensorPtr m_raySensor;
	gazebo::physics::ModelPtr m_gazeboVehicle;
	std::vector<gazebo::physics::LinkPtr> m_gazeboWheels;

	// other members
	double m_steeringInput;
	double m_maxSpeed;
	double m_currDist;
};

#endif /* SRC_GCLOCALVEHICLE_HH_ */
