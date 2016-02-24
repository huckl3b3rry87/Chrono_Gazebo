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
//This sets up the simulation by creating gcVehicles and loading a world for
//these vehicles to reference
//
// =============================================================================

// local includes
#include <GcVehicle.hh>
#include <GcVehicleBuilder.hh>

// standard library
#include <vector>
#include <cmath>

//chrono vehicle includes
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

///the chrono-engine includes
#include "physics/ChSystem.h"

///the gazebo includes
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

//gazebo_ros includes
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "boost/thread/mutex.hpp"

using namespace chrono;
using namespace gazebo;

class chrono_gazebo: public WorldPlugin {
public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
		this->_world = _parent;
		//disable the physics engine
		_world->EnablePhysicsEngine(false);
		_world->GetPhysicsEngine()->SetRealTimeUpdateRate(1000);

		//BEGIN LINE FOLLOW
		if (!ros::isInitialized()) {
			std::string msg =
					"A ROS node for Gazebo has not been initialized, unable to load plugin. "
							"Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package";
			ROS_FATAL_STREAM(msg);
			return;
		}
		this->rosnode_ = new ros::NodeHandle("chrono_gazebo");

		// initialize Chrono system
		chsys = new ChSystem();
		chsys->Set_G_acc(ChVector<>(0, 0, -9.81));
		chsys->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
		chsys->SetIterLCPmaxItersSpeed(iters);
		chsys->SetIterLCPmaxItersStab(150);
		chsys->SetMaxPenetrationRecoverySpeed(10.0);

		// load and initialize terrain
		terrain = ChSharedPtr<vehicle::RigidTerrain>(
				new vehicle::RigidTerrain(chsys,
						vehicle::GetDataFile(rigidterrain_file)));

		// create and initialize GcVehicleBuilder
		auto builder = GcVehicleBuilder(_world, chsys, terrain, step_size);
		builder.setVehicleFile(vehicle_file);
		builder.setPowertrainFile(simplepowertrain_file);
		builder.setTireFile(rigidtire_file);
		builder.setSpeedCtrlFile(speed_controller_file);
		builder.setSteerCtrlFile(steering_controller_file);
		builder.setNodeHandler(rosnode_);
		builder.setCallbackQueue(&queue_);
		//have chrono drive the vehicle around a circle
		ChBezierCurve* path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
		builder.setPath(path);
		builder.setMaxSpeed(maxSpeedInput);

		// Custom Callback Queue
		this->callback_queue_thread_ = boost::thread(
				boost::bind(&chrono_gazebo::QueueThread, this));

		// store the created vehicles
		gcVehicles = std::vector<boost::shared_ptr<GcVehicle> >(num_vehicles);
		const double pi = 3.1415926535;
		const double step = pi * 2 / 21;
		const double radius = 50;
		for (int i = 0; i < num_vehicles; i++) {
			const double ang = (i + 1) * step;
			auto pos = ChVector<>(radius * std::cos(ang), radius * std::sin(ang), 1);
			auto rot = ChQuaternion<>(std::cos((ang + pi / 2) / 2), 0, 0,
					std::sin((ang + pi / 2) / 2));
			builder.setInitCoordsys(ChCoordsys<>(pos, rot));
			gcVehicles[i] = builder.buildGcVehicle();
		}

		SetChronoDataPath("../data/");

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&chrono_gazebo::OnUpdate, this));

	}
public:
	void OnUpdate() {
		// update terrain
		terrain->Update(chsys->GetChTime());
		// advance each vehicle
		for (int i = 0; i < num_vehicles; i++) {
			gcVehicles[i]->advance();
		}
		terrain->Advance(step_size);
	}

	// custom callback queue thread
	void QueueThread() {
		static const double timeout = 0.01;

		while (this->rosnode_->ok()) {
			this->queue_.callAvailable(ros::WallDuration(timeout));
		}
	}

	~chrono_gazebo() {
		delete this->chsys;
		// Custom Callback Queue
		this->queue_.clear();
		this->queue_.disable();
		this->rosnode_->shutdown();
		this->callback_queue_thread_.join();
		delete this->rosnode_;
	}

private:
	int num_vehicles = 2;
	// list of vehicles
	std::vector<boost::shared_ptr<GcVehicle> > gcVehicles;

	// Chrono system
	ChSystem *chsys;
	// terrain object
	ChSharedPtr<vehicle::RigidTerrain> terrain;

	//chrono vehicle parameters, values should not matter unless no driver
	double maxSpeedInput = 8.333;

	//define the step size
	double step_size = 0.001;
	double iters = 10;

	ros::NodeHandle* rosnode_;
	ros::CallbackQueue queue_;
	boost::thread callback_queue_thread_;

	physics::WorldPtr _world;

	event::ConnectionPtr updateConnection;

	//Chrono vehicle setups
	////******THESE ARE BEING PULLED FROM THE DATA DIRECTORY AT
	////****** Chrono_Gazebo/data
	// JSON file for vehicle model
	std::string vehicle_file = "hmmwv/vehicle/HMMWV_Vehicle_4WD.json";

	// JSON files for tire models (rigid) and powertrain (simple)
	std::string rigidtire_file = "generic/tire/RigidTire.json";

	//JSON file for terrain models
	//std::string rigidterrain_file = "terrain/RigidMesh.json";
	std::string rigidterrain_file = "terrain/RigidPlane.json";

	std::string simplepowertrain_file = "generic/powertrain/SimplePowertrain.json";

	// Driver input file (if not using Irrlicht)
	std::string driver_file = "generic/driver/Sample_Maneuver.txt";

	//driver files
	// std::string driver_file = "generic/driver/Sample_Maneuver.txt";
	std::string steering_controller_file =
			"generic/driver/SteeringController.json";
	std::string speed_controller_file = "generic/driver/SpeedController.json";
	std::string path_file = "paths/testpath.txt";
	// std::string path_file = "paths/ISO_double_lane_change.txt";

	//	std::vector<ChSharedPtr<ChBody>> chCinderBlocks;
	//	std::vector<physics::ModelPtr> gzCinderBlocks;
	//	std::vector<ChSharedPtr<ChBody>> chLogs;
	//	std::vector<physics::ModelPtr> gzLogs;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(chrono_gazebo)
