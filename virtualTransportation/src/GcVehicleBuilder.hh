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
	GcVehicleBuilder(gazebo::physics::WorldPtr world, chrono::ChSystem *chsys,
			GcVehicle::ChTerrainPtr terrain,
			const double stepSize);

	// build a vehicle with all the components
	std::shared_ptr<GcVehicle> buildGcVehicle();

	ros::Subscriber &getLastRosSubscriber();

	// parameter setters

	void setVehicleFile(const std::string &vehicleFile);

	void setPowertrainFile(const std::string &powertrainFile);

	void setTireFile(const std::string &tireFile);

	void setSteerCtrlFile(const std::string &steerFile);

	void setSpeedCtrlFile(const std::string &speedFile);

	void setInitCoordsys(const chrono::ChCoordsys<> &coordsys);

	void setMaxSpeed(const double maxSpeed);

	void setPath(chrono::ChBezierCurve *path);

	void setNodeHandler(ros::NodeHandle *handle);

	void setCallbackQueue(ros::CallbackQueue *queue);

private:
	gazebo::physics::WorldPtr world;

	chrono::ChSystem *chsys;
	GcVehicle::ChTerrainPtr terrain;
	double stepSize = 0.01;

	std::string vehicleFile;
	std::string powertrainFile;
	std::string tireFile;
	std::string steerFile;
	std::string speedFile;

	ros::NodeHandle *handle = NULL;
	ros::CallbackQueue *queue = NULL;

	// vehicle specific parameters
	int vehId = 0;
	chrono::ChCoordsys<> coordsys;
	double maxSpeed = 0;
	chrono::ChBezierCurve *path = NULL;
	ros::Subscriber lastSub;
};

#endif /* SRC_GCVEHICLEBUILDER_HH_ */
