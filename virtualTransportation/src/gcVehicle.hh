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

//includes

#include <chrono_vehicle/driver/ChPathFollowerDriver.h>
#include <chrono_vehicle/powertrain/SimplePowertrain.h>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/tire/RigidTire.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <core/ChCoordsys.h>
#include <core/ChQuaternion.h>
#include <core/ChSmartpointers.h>
#include <core/ChVector.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ros/subscriber.h>
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
			const double maxSpeed, const sensors::RaySensor &raySensor,
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
	math::Pose &getPose(const ChVector<> vec, const ChQuaternion<> quat);

private:
	ChSharedPtr<vehicle::RigidTerrain> terrain;
	ChSharedPtr<vehicle::WheeledVehicle> vehicle;
	ChSharedPtr<vehicle::SimplePowertrain> powertrain;
	std::vector<ChSharedPtr<vehicle::RigidTire> > tires;
	ChSharedPtr<vehicle::ChPathFollowerDriver> driver;

	int numWheels;

	sensors::RaySensor raySensor;
	physics::ModelPtr gazeboVehicle;
	std::vector<physics::ModelPtr> gazeboWheels;

	double steeringInput;

	double maxSpeed;

	int id;

	double stepSize;
};

class GcVehicleBuilder {

public:
	GcVehicleBuilder(physics::WorldPtr world, ChSystem *chsys,
			ChSharedPtr<vehicle::RigidTerrain> terrain, const double stepSize);

	GcVehicle &buildGcVehicle();

	ros::Subscriber &getLastRosSubscriber();

	void setVehicleFile(const std::string &vehicleFile);

	void setPowertrainFile(const std::string &powertrainFile);

	void setTireFile(const std::string &tireFile);

	void setSteerCtrlFile(const std::string &steerFile);

	void setSpeedCtrlFile(const std::string &speedFile);

	void setInitCoordsys(const ChCoordsys<> &coordsys);

	void setMaxSpeed(const double maxSpeed);

	void setPath(const ChBezierCurve *path);

	void setNodeHandler(const ros::NodeHandle *handle);

	void setCallbackQueue(const ros::CallbackQueue *queue);

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
