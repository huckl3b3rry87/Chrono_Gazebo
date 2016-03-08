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
// Authors: Asher Elmquist, Leon Yang
//
//This sets up the simulation by creating gcVehicles and loading a world for
//these vehicles to reference
//

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
#include "gazebo/sensors/SensorManager.hh"

//gazebo_ros includes
#include <ros/callback_queue.h>
#include <ros/ros.h>

// others
#include <tinyxml.h>

using namespace chrono;
using namespace gazebo;

#define PATH_RADIUS 36.605637

class chrono_gazebo: public WorldPlugin {
public:
	void LoadMetadata() {
		TiXmlDocument meta("../data/metadata.xml");
		if (meta.LoadFile()) {
			TiXmlHandle root = TiXmlHandle(&meta).FirstChild("metadata");
			TiXmlElement *element;

			// program parameters
			element = root.FirstChild("numVehicles").Element();
			numVehicles = std::max(1, std::stoi(element->GetText()));
			element = root.FirstChild("vehicleGap").Element();
			vehicleGap = std::stof(element->GetText());
			element = root.FirstChild("iterations").Element();
			iters = std::max(1, std::stoi(element->GetText()));
			element = root.FirstChild("stepSize").Element();
			stepSize = std::stod(element->GetText());
			element = root.FirstChild("maxSpeed").Element();
			maxSpeed = std::stod(element->GetText());

			// get file locations
			TiXmlHandle files = root.FirstChild("files");
			rigidTerrainFile = vehicle::GetDataFile(
					files.FirstChild("terrain").Element()->GetText());
			vehicleFile = vehicle::GetDataFile(
					files.FirstChild("vehicle").Element()->GetText());
			rigidTireFile = vehicle::GetDataFile(
					files.FirstChild("tire").Element()->GetText());
			simplePowertrainFile = vehicle::GetDataFile(
					files.FirstChild("powertrain").Element()->GetText());
			driverFile = vehicle::GetDataFile(
					files.FirstChild("driver").Element()->GetText());
			steeringControllerFile = vehicle::GetDataFile(
					files.FirstChild("steeringController").Element()->GetText());
			speedControllerFile = vehicle::GetDataFile(
					files.FirstChild("speedController").Element()->GetText());
			pathFile = vehicle::GetDataFile(
					files.FirstChild("path").Element()->GetText());
		} else {
			std::cerr << "Error: metadata file does not exist." << std::endl;
			return;
		}

#ifdef DEBUG
		std::cout << "[GcVehicle] numVehicles: " << numVehicles << std::endl;
		std::cout << "[GcVehicle] vehicleGap: " << vehicleGap << std::endl;
		std::cout << "[GcVehicle] iters: " << iters << std::endl;
		std::cout << "[GcVehicle] step_size: " << stepSize << std::endl;
		std::cout << "[GcVehicle] maxSpeed: " << maxSpeed << std::endl;
		std::cout << "[GcVehicle] rigidTerrainFile: " << rigidTerrainFile
		<< std::endl;
		std::cout << "[GcVehicle] vehicleFile: " << vehicleFile << std::endl;
		std::cout << "[GcVehicle] rigidTireFile: " << rigidTireFile << std::endl;
		std::cout << "[GcVehicle] simplepowertrainFile: " << simplePowertrainFile
		<< std::endl;
		std::cout << "[GcVehicle] driverFile: " << driverFile << std::endl;
		std::cout << "[GcVehicle] steeringControllerFile: "
		<< steeringControllerFile << std::endl;
		std::cout << "[GcVehicle] speedControllerFile: " << speedControllerFile
		<< std::endl;
		std::cout << "[GcVehicle] pathFile: " << pathFile << std::endl;
#endif /* debug information */

	}

	void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
		LoadMetadata();

		this->_world = _parent;
		// disable the physics engine
		_world->EnablePhysicsEngine(false);
		_world->GetPhysicsEngine()->SetRealTimeUpdateRate(100);
		_world->GetPhysicsEngine()->SetMaxStepSize(stepSize);

		// BEGIN LINE FOLLOW
		if (!ros::isInitialized()) {
			std::string msg =
					"A ROS node for Gazebo has not been initialized, unable to load plugin. "
							"Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package";
			ROS_FATAL_STREAM(msg);
			return;
		}
		this->rosnode_ = new ros::NodeHandle("chrono_gazebo");

#ifdef DEBUG
		std::cout << "[GcVehicle] ROS node loaded." << std::endl;
#endif /* debug information */

		// initialize Chrono system
		chsys = std::make_shared<ChSystem>();
		chsys->Set_G_acc(ChVector<>(0, 0, -9.81));
		chsys->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
		chsys->SetIterLCPmaxItersSpeed(iters);
		chsys->SetIterLCPmaxItersStab(150);
		chsys->SetMaxPenetrationRecoverySpeed(10.0);

		// load and initialize terrain
		terrain = std::make_shared<vehicle::RigidTerrain>(chsys.get(),
				rigidTerrainFile);

#ifdef DEBUG
		std::cout << "[GcVehicle] Terrain file loaded." << std::endl;
#endif /* debug information */

		// create and initialize GcVehicleBuilder
		auto builder = GcVehicleBuilder(_world, chsys, terrain, PATH_RADIUS,
				vehicleGap, maxSpeed);
		builder.SetVehicleFile(vehicleFile);
		builder.SetPowertrainFile(simplePowertrainFile);
		builder.SetTireFile(rigidTireFile);
		builder.SetSpeedCtrlFile(speedControllerFile);
		builder.SetSteerCtrlFile(steeringControllerFile);
		builder.SetNodeHandler(rosnode_);
		builder.SetCallbackQueue(&queue_);
		// have chrono drive the vehicle around a circle
		ChBezierCurve* path = ChBezierCurve::read(pathFile);

#ifdef DEBUG
		std::cout << "[GcVehicle] Path file loaded." << std::endl;
#endif /* debug information */

		builder.SetPath(path);

		// Custom Callback Queue
		this->callback_queue_thread_ = boost::thread(
				boost::bind(&chrono_gazebo::QueueThread, this));

		// store the created vehicles
		gcVehicles = std::vector<std::shared_ptr<GcVehicle> >(numVehicles);
		for (int i = 0; i < numVehicles; i++) {
			gcVehicles[i] = builder.BuildGcVehicle();
		}

		SetChronoDataPath("../data/");

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&chrono_gazebo::OnUpdate, this));

#ifdef DEBUG
		std::cout << "[GcVehicle] chrono_gazebo loading complete." << std::endl;
#endif /* debug information */

	}

public:
	void OnUpdate() {
		// update terrain
		const double time = chsys->GetChTime();
		terrain->Synchronize(time);
		// advance each vehicle
		for (int i = 0; i < numVehicles; i++) {
			gcVehicles[i]->Synchronize(time);
		}
		terrain->Advance(stepSize);
		for (int i = 0; i < numVehicles; i++) {
			gcVehicles[i]->Advance(stepSize);
		}
		gcVehicles[0]->GetVehicle()->Advance(stepSize);
		//std::cout<<std::endl;
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
	int numVehicles;
	double vehicleGap;
	std::vector<std::shared_ptr<GcVehicle>> gcVehicles;

	std::shared_ptr<ChSystem> chsys;
	GcVehicle::ChTerrainPtr terrain;

	double maxSpeed;
	double stepSize;
	double iters;

	ros::NodeHandle* rosnode_;
	ros::CallbackQueue queue_;
	boost::thread callback_queue_thread_;

	physics::WorldPtr _world;

	event::ConnectionPtr updateConnection;

	// THESE ARE BEING PULLED FROM THE DATA DIRECTORY AT
	// Chrono_Gazebo/data

	// JSON file for vehicle model
	std::string vehicleFile;

	// JSON files for tire models (rigid) and powertrain (simple)
	std::string rigidTireFile;

	// JSON file for terrain models
	std::string rigidTerrainFile;

	std::string simplePowertrainFile;

	// Driver input file (if not using Irrlicht)
	std::string driverFile;

	// driver files
	std::string steeringControllerFile;
	std::string speedControllerFile;
	std::string pathFile;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(chrono_gazebo)
