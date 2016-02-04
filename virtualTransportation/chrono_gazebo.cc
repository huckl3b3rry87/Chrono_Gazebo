/*
 * This is a library (world plugin) for gazebo. It will add a vehicle in
 *chrono and use the OnUpdate() loop to run a step(s) of the chrono simulation
 *loop. It will then update the vehicel model pose in gazebo. The goal of this
 *is to have chrono simulate the vehicle and have gazebo output the results as
 *well as simulate sensors and provide feedback in the form of driving
 *parameters into chrono.
 *
 * Created by Asher Elmquist (UW SBEL and OSRF)
*/

#include <vector>
#include <cstdio>
#include <stdio.h>

#include <algorithm>
#include <assert.h>

//chrono vehicle includes
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

///the chrono-engine includes
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "geometry/ChCTriangleMesh.h"
#include "geometry/ChCTriangleMeshConnected.h"
#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "physics/ChBody.h"
#include "physics/ChLinkDistance.h"
#include "physics/ChLinkMate.h"

///the gazebo includes
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/sensors/sensors.hh>

//gazebo_ros includes
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <std_msgs/Float64.h>


using namespace chrono;
using namespace gazebo;

class chrono_gazebo : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    //create a world pointer
    this->_world = _parent;
    //disable the physics engine
    _parent->EnablePhysicsEngine(false);
    _world->GetPhysicsEngine()->SetRealTimeUpdateRate(1000);

    //BEGIN LINE FOLLOW
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosnode_ = new ros::NodeHandle("chrono_gazebo");
    // Custom Callback Queue
    ros::SubscribeOptions so0 = ros::SubscribeOptions::create<std_msgs::Float64>(
      "/track_point0",1,
      boost::bind( &chrono_gazebo::UpdateDriverInput0,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub0_ = this->rosnode_->subscribe(so0);

    ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Float64>(
      "/track_point1",1,
      boost::bind( &chrono_gazebo::UpdateDriverInput1,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub1_ = this->rosnode_->subscribe(so1);




    // Custom Callback Queue
    this->callback_queue_thread_ = boost::thread( boost::bind( &chrono_gazebo::QueueThread,this));
    //END LINE FOLLOW LOAD
    //CONNECT TO LASER
    this->raySensor0 =
    boost::dynamic_pointer_cast<sensors::RaySensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->_world->GetName() + "::vehicle0::chassis0::laser"));
        // this->world->GetName() + "::" + this->model->GetScopedName()
        // + "::pelvis::"S
        // "imu_sensor"));
    if (!this->raySensor0)
    std::cout << "laser 0 not found!\n\n";
    this->raySensor1 =
    boost::dynamic_pointer_cast<sensors::RaySensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->_world->GetName() + "::vehicle1::chassis1::laser"));
        // this->world->GetName() + "::" + this->model->GetScopedName()
        // + "::pelvis::"S
        // "imu_sensor"));
    if (!this->raySensor1)
    std::cout << "laser 1 not found!\n\n";
    if(raySensor0 == NULL || raySensor1 == NULL){
      std::cout<<"\n\n\nA LASER WAS NOT FOUND!!!\n\n\n";
    }
    else{
      raySensors.push_back(raySensor0);
      raySensors.push_back(raySensor1);
    }


    //END CONNECT TO LASER
    //create model pointers
    for(int i = 0; i<num_vehicles; i++){
      std::vector<physics::ModelPtr> wheels;
      physics::ModelPtr _model1;
      if(_world->GetModel("vehicle" + std::to_string(i)) != NULL){
          _model1 = _world->GetModel("vehicle" + std::to_string(i));
          gazeboVehicles.push_back(_model1);
      }
      else{std::cout<<"COULD NOT FIND GAZEBO MODEL: vehicle"<<i<<std::endl;}
      //create vector of wheels for easy reference
      wheels.push_back(_world->GetModel("wheel"+ std::to_string(i)+ "_1"));
      wheels.push_back(_world->GetModel("wheel"+ std::to_string(i)+ "_2"));
      wheels.push_back(_world->GetModel("wheel"+ std::to_string(i)+ "_3"));
      wheels.push_back(_world->GetModel("wheel"+ std::to_string(i)+ "_4"));
      gazeboWheels.push_back(wheels);
    }



    // for(int i=0;i<num_vehicles; i++){
    //   if(_world->GetModel("vehicle" + std::to_string (i)) != NULL){
    //     std::cout<<"Adding vehicles\n";
    //     gazeboVehicles.push_back(_world->GetModel("vehicle" + std::to_string (i)));
    //     gazeboWheels[i].push_back(_world->GetModel("wheel"+std::to_string(i) + "_1"));
    //     gazeboWheels[i].push_back(_world->GetModel("wheel"+std::to_string(i) + "_2"));
    //     gazeboWheels[i].push_back(_world->GetModel("wheel"+std::to_string(i) + "_3"));
    //     gazeboWheels[i].push_back(_world->GetModel("wheel"+std::to_string(i) + "_4"));
    //
    //   }
    //  else{std::cout<<"COULD NOT FIND GAZEBO MODEL: vehicle"<<i<<std::endl;}
    // }
    //create vector of wheels for easy reference
    //setup chrono vehicle
    //set initial conditions
// std::cout<<"Reached Line 120\n";
    for(int i=0; i<num_vehicles; i++){
      // std::cout<<"Value of i="<<i<<std::endl;
      chronoVehiclesInitLoc.push_back(ChVector<> (-10*i, 0, 1.0));
      chronoVehiclesInitRot.push_back(ChQuaternion<> (1, 0, 0, 0));
    }
    // ChVector<> initLoc(0, 0, 1.0);
    // ChQuaternion<> initRot(1, 0, 0, 0);
// std::cout<<"Reached Line 128\n";
    //create the chrono vehicle
    for(int i=0; i<num_vehicles; i++){
      // std::cout<<"Value of i, second loop = "<<i<<std::endl;
      // std::cout<<"num_vehicles = "<<num_vehicles<<std::endl;

      veh = ChSharedPtr<vehicle::WheeledVehicle>(new vehicle::WheeledVehicle(vehicle::GetDataFile(vehicle_file)));
      veh->Initialize(ChCoordsys<>(chronoVehiclesInitLoc[i], chronoVehiclesInitRot[i]));

      //in development for use with chrono path following
      // ChBezierCurve* path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
      //
      //
      //
      // ChPathFollowerDriver driver_follower(my_hmmwv.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
      //         vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
      //
      //

      terrain = ChSharedPtr<vehicle::RigidTerrain>(new vehicle::RigidTerrain(veh->GetSystem(), vehicle::GetDataFile(rigidterrain_file)));

      powertrain = ChSharedPtr<vehicle::SimplePowertrain>(new vehicle::SimplePowertrain(vehicle::GetDataFile(simplepowertrain_file)));
      powertrain->Initialize();

      num_axles = veh->GetNumberAxles();
      num_wheels = 2 * num_axles;
      tires = std::vector<ChSharedPtr<vehicle::RigidTire>>(num_wheels);

      for (int i = 0; i < num_wheels; i++) {
          //create the tires from the tire file
          tires[i] = ChSharedPtr<vehicle::RigidTire>(new vehicle::RigidTire(vehicle::GetDataFile(rigidtire_file)));
          tires[i]->Initialize(veh->GetWheelBody(i));
      }

      tire_forces = vehicle::TireForces(num_wheels);
      wheel_states = vehicle::WheelStates(num_wheels);
      //vehicle->GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_APGD);
      veh->GetSystem()->SetIterLCPmaxItersSpeed(iters);
      //vehicle->GetSystem()->SetIterLCPmaxItersStab(100);
      veh->GetSystem()->SetMaxPenetrationRecoverySpeed(10.0);

      //update the chrono system vectors
      chronoVehicles.push_back(veh);
      chronoPowertrains.push_back(powertrain);
      chronoTires.push_back(tires);
      chronoTireForces.push_back(tire_forces);
      chronoWheelStates.push_back(wheel_states);
    }

    //Add the terrain mesh to Chrono
    SetChronoDataPath("../data/");

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
         boost::bind(&chrono_gazebo::OnUpdate, this));

  }
  public: void OnUpdate()
   {

     for(int i=0; i<num_vehicles; i++){

       //saySensor->GetRanges(ranges);
       raySensors[i]->GetRanges(ranges);


       //control vehicle speed
       //this will maintain an approximate speed of max_speed m/s

       //connect to the ray data to stop vehicle in case of obstruction
       //this will only use a rectangle in the direction of the wheels to detect obstacles.

       center = steering_input[i]*50 + 50;
       minRange = 100000.0;
       for(int j=0; j<ranges.size(); j++){
         if(ranges[j]*sin(abs(j-center)*3.14159/180.0) <=2.5){
           if(ranges[j]<minRange)
             minRange = ranges[j];
         }
       }
       maxSpeed = maxSpeedInput;
       if(minRange<10000.0){
         //linear so 6m/s at 20m at 0m/s at 2.5m
         maxSpeed = .343*minRange - .857;
       }

       if(chronoVehicles[i]->GetVehicleSpeedCOM() >= maxSpeed +1.0){
         braking_input = 0.5;
         throttle_input = 0.0;
       }
       else if(chronoVehicles[i]->GetVehicleSpeedCOM() <= maxSpeed /2.0){
         braking_input = 0.0;
         throttle_input = 0.2;
       }
       else if(chronoVehicles[i]->GetVehicleSpeedCOM() >= maxSpeed){
         braking_input = 0.3;
         throttle_input = 0.1;
       }
       else{
         braking_input = 0.15;
         throttle_input = 0.2;
       }
       // std::cout<<"Min Range: " << minRange<<std::endl;
       // std::cout<<"Max Speed: " << maxSpeed<<std::endl;
       // std::cout<<"Throttle Input: " << throttle_input<<std::endl;
       // std::cout<<"Braking Input: " << braking_input<<std::endl;
       // std::cout<<"Steering Input: " << steering_input<<std::endl;
       // std::cout<<std::endl;

        //collect module information

        powertrain_torque = chronoPowertrains[i]->GetOutputTorque();
        driveshaft_speed = chronoVehicles[i]->GetDriveshaftSpeed();
        for (int j = 0; j < num_wheels; j++) {
          chronoTireForces[i][j] = chronoTires[i][j]->GetTireForce();
          chronoWheelStates[i][j] = chronoVehicles[i]->GetWheelState(j);
        }

        //Update modules (process inputs from other modules)
        time = chronoVehicles[i]->GetSystem()->GetChTime();
        //driver->Update(time);
        //pursuitDriver->Update(chronoVehicles[i]icle);
        chronoPowertrains[i]->Update(time, throttle_input, driveshaft_speed);
        chronoVehicles[i]->Update(time, steering_input[i], braking_input, powertrain_torque, chronoTireForces[i]);
        terrain->Update(time);
        for (int j = 0; j < num_wheels; j++)
          chronoTires[i][j]->Update(time, chronoWheelStates[i][j], *terrain);

        // Advance simulation for one timestep for all modules
        //driver->Advance(step_size);
        chronoPowertrains[i]->Advance(step_size);
        chronoVehicles[i]->Advance(step_size);
        terrain->Advance(step_size);
        for (int j = 0; j < num_wheels; j++){
          chronoTires[i][j]->Advance(step_size);
        }

        //Communication and updates between Chrono and Gazebo
        gazeboVehicles[i]->SetWorldPose(math::Pose(
            math::Vector3(
                chronoVehicles[i]->GetChassisPos().x,
                chronoVehicles[i]->GetChassisPos().y,
                chronoVehicles[i]->GetChassisPos().z
            ),
            math::Quaternion(
                chronoVehicles[i]->GetChassisRot().e0,
                chronoVehicles[i]->GetChassisRot().e1,
                chronoVehicles[i]->GetChassisRot().e2,
                chronoVehicles[i]->GetChassisRot().e3
            )), "link");
          for(int j=0; j<gazeboWheels[i].size();j++){
            gazeboWheels[i][j]->SetWorldPose(math::Pose(
                math::Vector3(
                    chronoVehicles[i]->GetWheelPos(j).x,
                    chronoVehicles[i]->GetWheelPos(j).y,
                    chronoVehicles[i]->GetWheelPos(j).z
                ),
                math::Quaternion(
                    chronoVehicles[i]->GetWheelRot(j).e0,
                    chronoVehicles[i]->GetWheelRot(j).e1,
                    chronoVehicles[i]->GetWheelRot(j).e2,
                    chronoVehicles[i]->GetWheelRot(j).e3
                )), "link");
          }
     }

        //std::cout<<"updating the cinderblocks\n";
   }

   void UpdateDriverInput0(const std_msgs::Float64::ConstPtr& _msg)
    {
      std::cout<<"Updating steering input 0\n\n";
      this->steering_input[0] = _msg->data;
    }
    void UpdateDriverInput1(const std_msgs::Float64::ConstPtr& _msg)
     {
       std::cout<<"Updating steering input 1\n\n";
       this->steering_input[1] = _msg->data;
     }

    // custom callback queue thread
    void QueueThread()
    {
      static const double timeout = 0.01;

      while (this->rosnode_->ok())
      {
        this->queue_.callAvailable(ros::WallDuration(timeout));
      }
    }
    ~chrono_gazebo(){
      // Custom Callback Queue
      this->queue_.clear();
      this->queue_.disable();
      this->rosnode_->shutdown();
      this->callback_queue_thread_.join();
      delete this->rosnode_;
    }

private: ros::NodeHandle* rosnode_;
    ros::Subscriber sub0_;
    ros::Subscriber sub1_;
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

    std::string simplepowertrain_file = "generic/powertrain/SimplePowertrain.json";

    // Driver input file (if not using Irrlicht)
    std::string driver_file = "generic/driver/Sample_Maneuver.txt";


    //chrono driver files
    // std::string steering_controller_file = "generic/driver/SteeringController.json";
    // std::string speed_controller_file = "generic/driver/SpeedController.json";
    // std::string path_file = "paths/curve.txt";
    // std::string path_file = "paths/ISO_double_lane_change.txt";

    //chrono vehicle components
    ChSharedPtr<vehicle::WheeledVehicle> veh;
    ChSharedPtr<vehicle::RigidTerrain> terrain;
    ChSharedPtr<vehicle::SimplePowertrain> powertrain;
    std::vector<ChSharedPtr<vehicle::RigidTire> > tires;
    //ChSharedPtr<ChDataDriver> driver;
    //std::unique_ptr<PurePursuitDriver> pursuitDriver; //*****************

    std::vector<double> ranges;
    vehicle::TireForces   tire_forces;
    vehicle::WheelStates  wheel_states;

    //chrono vehicle parameters, values should not matter unless no driver
    double         driveshaft_speed;
    double         powertrain_torque;
    double         throttle_input = 0.2;
    std::vector<double> steering_input = {0.0, 0.0};
    double         braking_input = 0.1;
    int num_axles;
    int num_wheels;
    double center;
    double minRange;
    double maxSpeed;
    double maxSpeedInput = 8.333;

    //define the step size
    double step_size = 0.001;
    double iters = 10;
    double time = 0.0;

    //vector of vehicle and associated models
    int num_vehicles = 2;
    std::vector<ChSharedPtr<vehicle::WheeledVehicle>> chronoVehicles;
    std::vector<ChSharedPtr<vehicle::SimplePowertrain>> chronoPowertrains;
    std::vector<std::vector<ChSharedPtr<vehicle::RigidTire>>> chronoTires;
    std::vector<vehicle::TireForces> chronoTireForces;
    std::vector<vehicle::WheelStates> chronoWheelStates;
    std::vector<ChVector<>> chronoVehiclesInitLoc;
    std::vector<ChQuaternion<>> chronoVehiclesInitRot;

    //pointers to reference the models shared by chrono and gazebo
    boost::shared_ptr<sensors::RaySensor> raySensor0;
    boost::shared_ptr<sensors::RaySensor> raySensor1;
    std::vector<boost::shared_ptr<sensors::RaySensor>> raySensors;
    std::vector<ChSharedPtr<ChBody>> chronoModels;
    std::vector<ChSharedPtr<ChBody>> chCinderBlocks;
    std::vector<physics::ModelPtr> gzCinderBlocks;
    std::vector<ChSharedPtr<ChBody>> chLogs;
    std::vector<physics::ModelPtr> gzLogs;

    //vector for gazebo's vehicles
    std::vector<std::vector<physics::ModelPtr>> gazeboWheels;
    std::vector<physics::ModelPtr> gazeboVehicles;

    physics::WorldPtr _world;

    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(chrono_gazebo)
