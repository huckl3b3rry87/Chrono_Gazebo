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

#ifndef SRC_GCVEHICLEBUILDER_HH_
#define SRC_GCVEHICLEBUILDER_HH_

#include <core/ChCoordsys.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <GcVehicle.hh>
#include <ros/subscriber.h>
#include <string>

namespace chrono {
class ChBezierCurve;
} /* namespace chrono */

class GcVehicleBuilder {

public:
	// constructor
	GcVehicleBuilder(gazebo::physics::WorldPtr world,
			std::shared_ptr<chrono::ChSystem> chsys, GcVehicle::ChTerrainPtr terrain,
			const double pathRadius, const double vehicleGap, const double maxSpeed);

	// build a vehicle with all the components
	std::shared_ptr<GcVehicle> BuildGcVehicle();

	ros::Subscriber &GetLastRosSubscriber() {
		return m_lastSub;
	}

	// parameter setters

	void SetVehicleFile(const std::string &vehicleFile) {
		m_vehicleFile = vehicleFile;
	}

	void SetPowertrainFile(const std::string &powertrainFile) {
		m_powertrainFile = powertrainFile;
	}

	void SetTireFile(const std::string &tireFile) {
		m_tireFile = tireFile;
	}

	void SetSteerCtrlFile(const std::string &steerFile) {
		m_steerFile = steerFile;
	}

	void SetSpeedCtrlFile(const std::string &speedFile) {
		m_speedFile = speedFile;
	}

	void SetPath(chrono::ChBezierCurve *path) {
		m_path = path;
	}

	void SetNodeHandler(ros::NodeHandle *handle) {
		m_handle = handle;
	}

	void SetCallbackQueue(ros::CallbackQueue *queue) {
		m_queue = queue;
	}

private:
	gazebo::physics::WorldPtr m_world;

	std::shared_ptr<chrono::ChSystem> m_chsys;
	GcVehicle::ChTerrainPtr m_terrain;

	std::string m_vehicleFile;
	std::string m_powertrainFile;
	std::string m_tireFile;
	std::string m_steerFile;
	std::string m_speedFile;

	ros::NodeHandle *m_handle = NULL;
	ros::CallbackQueue *m_queue = NULL;

	// vehicle specific parameters
	int m_vehId = 0;
	double m_pathRadius = 0;
	double m_vehicleGap = 0;
	double m_vehicleDist = 0;
	double m_followingTime = 0;
	double m_maxSpeed = 0;
	chrono::ChBezierCurve *m_path = NULL;
	ros::Subscriber m_lastSub;
};

#endif /* SRC_GCVEHICLEBUILDER_HH_ */
