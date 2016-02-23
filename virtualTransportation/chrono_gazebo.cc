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
//This sets up the simulation by creating gcVehicles and loading a world for
//these vehicles to reference
//
// =============================================================================
#include <GcVehicle.hh>
#include <GcVehicleBuilder.hh>
#include <vector>

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
		//create a world pointer
		this->_world = _parent;
		//disable the physics engine
		_world->EnablePhysicsEngine(false);
		_world->GetPhysicsEngine()->SetRealTimeUpdateRate(1000);

		//BEGIN LINE FOLLOW
		if (!ros::isInitialized()) {
			ROS_FATAL_STREAM(
					"A ROS node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		this->rosnode_ = new ros::NodeHandle("chrono_gazebo");

		auto chsys = new ChSystem();
		chsys->Set_G_acc(ChVector<>(0, 0, -9.81));
		chsys->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
		chsys->SetIterLCPmaxItersSpeed(iters);
		chsys->SetIterLCPmaxItersStab(150);
		chsys->SetMaxPenetrationRecoverySpeed(10.0);

		terrain = ChSharedPtr<vehicle::RigidTerrain>(
				new vehicle::RigidTerrain(chsys,
						vehicle::GetDataFile(rigidterrain_file)));

		auto builder = GcVehicleBuilder(_world, chsys, terrain, step_size);
		builder.setVehicleFile(vehicle_file);
		builder.setPowertrainFile(simplepowertrain_file);
		builder.setTireFile(rigidtire_file);
		builder.setSpeedCtrlFile(speed_controller_file);
		builder.setSteerCtrlFile(steering_controller_file);
		builder.setNodeHandler(rosnode_);
		builder.setCallbackQueue(&queue_);
		//have chrono drive the vehicle around a circle
		ChBezierCurve* path = ChBezierCurve::read(
				vehicle::GetDataFile(path_file));
		builder.setPath(path);
		builder.setMaxSpeed(maxSpeedInput);

// Custom Callback Queue
		this->callback_queue_thread_ = boost::thread(
				boost::bind(&chrono_gazebo::QueueThread, this));
		//END LINE FOLLOW LOAD
		//CONNECT TO LASER

//END CONNECT TO LASER
//create model pointers
		for (int i = 0; i < num_vehicles; i++) {
			builder.setInitCoordsys(
					ChCoordsys<>(ChVector<>(0, 5 * i, 1.0),
							ChQuaternion<>(1, 0, 0, 0)));
			gcVehicles.push_back(builder.buildGcVehicle());
		}

		//setup chrono vehicle
		//set initial conditions
// std::cout<<"Reached Line 120\n";

		// ChVector<> initLoc(0, 0, 1.0);
		// ChQuaternion<> initRot(1, 0, 0, 0);
// std::cout<<"Reached Line 128\n";
		//create the chrono vehicle


		//Add the terrain mesh to Chrono
		SetChronoDataPath("../data/");

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&chrono_gazebo::OnUpdate, this));

	}
public:
	void OnUpdate() {

		//terrain->Update(chsys->GetChTime());

		for (int i = 0; i < num_vehicles; i++) {
			gcVehicles[i]->advance_();
		}

		//terrain->Advance(step_size);
		//std::cout<<"updating the cinderblocks\n";
	}

	// custom callback queue thread
	void QueueThread() {
		static const double timeout = 0.01;

		while (this->rosnode_->ok()) {
			this->queue_.callAvailable(ros::WallDuration(timeout));
		}
	}
	~chrono_gazebo() {
		// Custom Callback Queue
		this->queue_.clear();
		this->queue_.disable();
		this->rosnode_->shutdown();
		this->callback_queue_thread_.join();
		delete this->rosnode_;
	}

private:

	//vector of vehicle and associated models
	int num_vehicles = 2;

	std::vector<boost::shared_ptr<GcVehicle> > gcVehicles;

	ros::NodeHandle* rosnode_;
	ros::CallbackQueue queue_;
	boost::thread callback_queue_thread_;

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

	std::string simplepowertrain_file =
			"generic/powertrain/SimplePowertrain.json";

	// Driver input file (if not using Irrlicht)
	std::string driver_file = "generic/driver/Sample_Maneuver.txt";

	//driver files
	// std::string driver_file = "generic/driver/Sample_Maneuver.txt";
	std::string steering_controller_file =
			"generic/driver/SteeringController.json";
	std::string speed_controller_file = "generic/driver/SpeedController.json";
	std::string path_file = "paths/testpath.txt";
	// std::string path_file = "paths/ISO_double_lane_change.txt";

	//chrono vehicle components
	ChSharedPtr<vehicle::RigidTerrain> terrain;

	//chrono vehicle parameters, values should not matter unless no driver
	double maxSpeedInput = 8.333;

	//define the step size
	double step_size = 0.001;
	double iters = 10;

	//pointers to reference the models shared by chrono and gazebo
//	std::vector<ChSharedPtr<ChBody>> chCinderBlocks;
//	std::vector<physics::ModelPtr> gzCinderBlocks;
//	std::vector<ChSharedPtr<ChBody>> chLogs;
//	std::vector<physics::ModelPtr> gzLogs;

	//vector for gazebo's vehicles

	physics::WorldPtr _world;

	event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(chrono_gazebo)
