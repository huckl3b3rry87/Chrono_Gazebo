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
#include <GcLocalVehicle.hh>
#include <GcNetworkUtil.hh>
#include <GcTypes.hh>
#include <GcUtil.hh>
#include <GcVehicleBuilder.hh>
#include <OSM2Gc.hh>

// standard library
#include <vector>
#include <cmath>
#include <stdio.h>

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
//#include <ros/callback_queue.h>
//#include <ros/ros.h>

// others
#include <tinyxml.h>

using namespace chrono;
using namespace gazebo;

#define PATH_RADIUS 36.605637
#ifndef DEBUG
#define DEBUG
class chrono_gazebo: public WorldPlugin {

private:
	void LoadMetadata() {
		TiXmlDocument meta("../data/metadata.xml");
		if (meta.LoadFile()) {
			TiXmlHandle root = TiXmlHandle(&meta).FirstChild("metadata");
			TiXmlElement *element;
			// program parameters
			fprintf(stderr, "loading metadata\n");
			if (root.FirstChild("network").ToNode()) {
				networkMode = true;
				serveraddr =
						root.FirstChild("network").FirstChild("serveraddr").Element()->GetText();
			}
			element = root.FirstChild("numVehicles").Element();
			numVehicles = std::max(1, std::stoi(element->GetText()));
			element = root.FirstChild("maxSpeed").Element();
			maxSpeed = std::stod(element->GetText());
			element = root.FirstChild("followingTime").Element();
			followingTime = std::stof(element->GetText());
			element = root.FirstChild("minDist").Element();
			minDist = std::stof(element->GetText());
			element = root.FirstChild("vehicleGap").Element();
			vehicleGap = std::stof(element->GetText());
			element = root.FirstChild("iterations").Element();
			iters = std::max(1, std::stoi(element->GetText()));
			element = root.FirstChild("stepSize").Element();
			stepSize = std::stod(element->GetText());

			fprintf(stderr, "first part loaded\n");

			// get file locations
			TiXmlHandle files = root.FirstChild("files");
			if (files.FirstChild("citymap").ToNode()) {
				citymapFile = vehicle::GetDataFile(
						files.FirstChild("citymap").Element()->GetText());
			}
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
		fprintf(stderr, "metadata loaded\n");

#ifdef DEBUG
		std::cout << "[GcLocalVehicle] numVehicles: " << numVehicles << std::endl;
		std::cout << "[GcLocalVehicle] maxSpeed: " << maxSpeed << std::endl;
		std::cout << "[GcLocalVehicle] followingTime: " << followingTime
				<< std::endl;
		std::cout << "[GcLocalVehicle] minDist: " << minDist << std::endl;
		std::cout << "[GcLocalVehicle] vehicleGap: " << vehicleGap << std::endl;
		std::cout << "[GcLocalVehicle] iters: " << iters << std::endl;
		std::cout << "[GcLocalVehicle] step_size: " << stepSize << std::endl;
		std::cout << "[GcLocalVehicle] rigidTerrainFile: " << rigidTerrainFile
				<< std::endl;
		std::cout << "[GcLocalVehicle] vehicleFile: " << vehicleFile << std::endl;
		std::cout << "[GcLocalVehicle] rigidTireFile: " << rigidTireFile
				<< std::endl;
		std::cout << "[GcLocalVehicle] simplepowertrainFile: "
				<< simplePowertrainFile << std::endl;
		std::cout << "[GcLocalVehicle] driverFile: " << driverFile << std::endl;
		std::cout << "[GcVehicle] steeringControllerFile: "
				<< steeringControllerFile << std::endl;
		std::cout << "[GcLocalVehicle] speedControllerFile: " << speedControllerFile
				<< std::endl;
		std::cout << "[GcLocalVehicle] pathFile: " << pathFile << std::endl;
#endif /* debug information */

	}

public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
		LoadMetadata();
		SetChronoDataPath("../data/");
		this->_world = _parent;

		if (networkMode) {
			fprintf(stderr, "connecting to server\n");
			sockfd = gc::ConnectGcServer(serveraddr, &networkId, &numVehicles);
			printf("networkId: %d, numVehicles: %d\n", networkId, numVehicles);
		}

		// add camera to the first vehicle
//		TiXmlDocument doc("models/gcVehicle/model.sdf");
//		assert(doc.LoadFile());
//		sdf::ElementPtr vehiclePtr(new sdf::Element);
//		sdf::initFile("model.sdf", vehiclePtr);
//		sdf::readDoc(&doc, vehiclePtr, "models/gcVehicle/model.sdf");
//		for (int i = numVehicles - 1; i >= 0; i--) {
//			vehiclePtr->GetAttribute("name")->SetFromString(
//					"vehicle_" + std::to_string(i));
//			if (i == 0) {
//				sdf::ElementPtr ptr(new sdf::Element);
//				sdf::initFile("link.sdf", ptr);
//				TiXmlDocument doc1("models/camera.sdf");
//				doc1.LoadFile();
//				sdf::readDoc(&doc1, ptr, "models/camera.sdf");
//				vehiclePtr->InsertElement(ptr);
//			}
//			sdf::SDFPtr newSDF(new sdf::SDF);
//			newSDF->Root(vehiclePtr->Clone());
//			_world->InsertModelSDF(*newSDF);
//		}

		// load city file
		if (citymapFile != "") {
			OSM2Gc importer(citymapFile);
			_world->InsertModelSDF(*importer.GetRoadModel());
			auto buildingModels = importer.GetBuildingModels();
			for (auto model : buildingModels)
				_world->InsertModelSDF(*model);
		}

		// disable the physics engine
		_world->EnablePhysicsEngine(false);
		_world->GetPhysicsEngine()->SetRealTimeUpdateRate(300);
		_world->GetPhysicsEngine()->SetMaxStepSize(stepSize);

//		// BEGIN LINE FOLLOW
//		if (!ros::isInitialized()) {
//			std::string msg =
//					"A ROS node for Gazebo has not been initialized, unable to load plugin. "
//							"Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package";
//			ROS_FATAL_STREAM(msg);
//			return;
//		}
//		this->rosnode_ = new ros::NodeHandle("chrono_gazebo");

//#ifdef DEBUG
//		std::cout << "[GcLocalVehicle] ROS node loaded." << std::endl;
//#endif /* debug information */

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
		auto builder = GcVehicleBuilder(_world, chsys, terrain,
		PATH_RADIUS, maxSpeed, followingTime, minDist, vehicleGap);
		builder.SetSockfd(sockfd);
		builder.SetVehicleFile(vehicleFile);
		builder.SetPowertrainFile(simplePowertrainFile);
		builder.SetTireFile(rigidTireFile);
		builder.SetSpeedCtrlFile(speedControllerFile);
		builder.SetSteerCtrlFile(steeringControllerFile);
//		builder.SetNodeHandler(rosnode_);
//		builder.SetCallbackQueue(&queue_);
		// have chrono drive the vehicle around a circle
		ChBezierCurve* path = ChBezierCurve::read(pathFile);
//		for (int i = 0; i < path->getNumPoints() - 1; i++) {
//			for (double j = 0; j < 1; j += 0.05) {
//				auto pt = path->eval(i, j);
//				printf("%.3f %.3f\n", pt.x, pt.y);
//			}
//		}

#ifdef DEBUG
		std::cout << "[GcLocalVehicle] Path file loaded." << std::endl;
#endif /* debug information */

		builder.SetPath(path);

		// Custom Callback Queue
//		this->callback_queue_thread_ = boost::thread(
//				boost::bind(&chrono_gazebo::QueueThread, this));

		// store the created vehicles
		gcVehicles = std::vector<std::shared_ptr<GcVehicle>>(numVehicles);
		for (int i = 0; i < numVehicles; i++) {
			builder.SetNetworkVehicle(networkMode && i != networkId);
			gcVehicles[i] = builder.BuildGcVehicle();
		}

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&chrono_gazebo::OnUpdate, this));

#ifdef DEBUG
		std::cout << "[GcVehicle] chrono_gazebo loading complete." << std::endl;
#endif /* debug information */

	}

public:
	void OnUpdate() {
		// make sure initialized
		if (!initialized) {
			initialized = true;
			for (auto &veh : gcVehicles) {
				initialized &= veh->Init();
			}
			if (!initialized)
				return;
		}

		// update terrain
		const double time = chsys->GetChTime();
		terrain->Synchronize(time);
		// advance each vehicle
		for (int i = 0; i < numVehicles; i++) {
			gcVehicles[i]->Synchronize(time);
		}
		terrain->Advance(stepSize);

		if (networkMode) {
			gcVehicles[networkId]->Advance(stepSize);
			auto veh = std::dynamic_pointer_cast<GcLocalVehicle>(
					gcVehicles[networkId])->GetVehicle();
			struct gcPacket packet;
			std::vector<math::Pose> poses(5);
			for (int i = 0; i < 4; i++) {
				poses[i] = gc::GetGcPose(veh->GetWheelPos(i), veh->GetWheelRot(i));
			}
			poses[4] = gc::GetGcPose(veh->GetChassisPos(), veh->GetChassisRot());
			packet = gc::GetPacket(poses);
			gc::Send(sockfd, (char*) &packet, sizeof(packet));
		}

		for (int i = 0; i < numVehicles; i++) {
			if (i == networkId)
				continue;
			gcVehicles[i]->Advance(stepSize);
		}
//		gcVehicles[0]->GetVehicle()->Advance(stepSize);
		double t = 0;
		while (t < stepSize) {
			double h = std::min<>(1e-3, stepSize - t);
			chsys->DoStepDynamics(h);
			t += h;
		}
		//std::cout<<std::endl;
	}

// custom callback queue thread
//	void QueueThread() {
//		static const double timeout = 0.01;
//
//		while (this->rosnode_->ok()) {
//			this->queue_.callAvailable(ros::WallDuration(timeout));
//		}
//	}

	~chrono_gazebo() {
		if (sockfd != -1) {
			close(sockfd);
		}
		// Custom Callback Queue
//		this->queue_.clear();
//		this->queue_.disable();
//		this->rosnode_->shutdown();
//		this->callback_queue_thread_.join();
//		delete this->rosnode_;
	}

private:
	bool networkMode = false;
	std::string serveraddr;
	int networkId = -1;
	int sockfd = -1;
	int numVehicles;
	std::vector<std::shared_ptr<GcVehicle>> gcVehicles;bool initialized = false;

	std::shared_ptr<ChSystem> chsys;
	gc::ChTerrainPtr terrain;

	double maxSpeed;
	double followingTime;
	double minDist;
	double vehicleGap;

	double stepSize;
	double iters;

//	ros::NodeHandle* rosnode_;
//	ros::CallbackQueue queue_;
//	boost::thread callback_queue_thread_;

	physics::WorldPtr _world;

	event::ConnectionPtr updateConnection;

// THESE ARE BEING PULLED FROM THE DATA DIRECTORY AT
// Chrono_Gazebo/data

	std::string citymapFile = "";

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
}
;
#endif
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(chrono_gazebo)
