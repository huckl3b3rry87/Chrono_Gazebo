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


#ifndef SRC_GCVEHICLEBUILDER_HH_
#define SRC_GCVEHICLEBUILDER_HH_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <core/ChCoordsys.h>
#include <core/ChSmartpointers.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ros/subscriber.h>
#include <string>

#include "GcVehicle.hh"

namespace chrono {
class ChBezierCurve;
} /* namespace chrono */

class GcVehicle;

using namespace chrono;
using namespace gazebo;

class GcVehicleBuilder {

public:
	GcVehicleBuilder(physics::WorldPtr world, ChSystem *chsys,
			ChSharedPtr<vehicle::RigidTerrain> terrain, const double stepSize);

	boost::shared_ptr<GcVehicle> buildGcVehicle();

	ros::Subscriber &getLastRosSubscriber();

	void setVehicleFile(const std::string &vehicleFile);

	void setPowertrainFile(const std::string &powertrainFile);

	void setTireFile(const std::string &tireFile);

	void setSteerCtrlFile(const std::string &steerFile);

	void setSpeedCtrlFile(const std::string &speedFile);

	void setInitCoordsys(const ChCoordsys<> &coordsys);

	void setMaxSpeed(const double maxSpeed);

	void setPath(ChBezierCurve *path);

	void setNodeHandler(ros::NodeHandle *handle);

	void setCallbackQueue(ros::CallbackQueue *queue);

private:
	physics::WorldPtr world;
	ChSystem *chsys;
	ChSharedPtr<vehicle::RigidTerrain> terrain;
	std::string vehicleFile;
	std::string powertrainFile;
	std::string tireFile;
	std::string steerFile;
	std::string speedFile;
	ros::NodeHandle *handle = NULL;
	ros::CallbackQueue *queue = NULL;
	int vehId = 0;
	ChCoordsys<> coordsys;
	double maxSpeed = 0;
	ChBezierCurve *path = NULL;
	double stepSize = 0.01;
	ros::Subscriber lastSub;
};

#endif /* SRC_GCVEHICLEBUILDER_HH_ */
