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
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>(
      "/track_point",1,
      boost::bind( &chrono_gazebo::UpdateDriverInput,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(so);

    // Custom Callback Queue
    this->callback_queue_thread_ = boost::thread( boost::bind( &chrono_gazebo::QueueThread,this ) );
    //END LINE FOLLOW LOAD

    //CONNECT TO LASER
    this->raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->_world->GetName() + "::vehicle::chassis::laser"));
        // this->world->GetName() + "::" + this->model->GetScopedName()
        // + "::pelvis::"
        // "imu_sensor"));
    if (!this->raySensor)
    std::cout << "laser not found!\n\n";
    //END CONNECT TO LASER

    //create model pointers
    if(_world->GetModel("vehicle") != NULL){
        _model1 = _world->GetModel("vehicle");
    }
    //create vector of wheels for easy reference
    wheels.push_back(_world->GetModel("wheel1"));
    wheels.push_back(_world->GetModel("wheel2"));
    wheels.push_back(_world->GetModel("wheel3"));
    wheels.push_back(_world->GetModel("wheel4"));


    //setup chrono vehicle
    //set initial conditions
    ChVector<> initLoc(0, 0, 1.0);
    ChQuaternion<> initRot(1, 0, 0, 0);

    //create the chrono vehicle
    veh = ChSharedPtr<vehicle::WheeledVehicle>(new vehicle::WheeledVehicle(vehicle::GetDataFile(vehicle_file)));
    veh->Initialize(ChCoordsys<>(initLoc, initRot));

    terrain = ChSharedPtr<vehicle::RigidTerrain>(new vehicle::RigidTerrain(veh->GetSystem(), vehicle::GetDataFile(rigidterrain_file)));
    //std::cout<<"added terrain mesh"<<std::endl;

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

    //Add the terrain mesh to Chrono
    SetChronoDataPath("../data/");

    //std::cout << "Path: " << GetChronoDataPath() << std::endl;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
         boost::bind(&chrono_gazebo::OnUpdate, this));
    //std::cout<<"Terminated Load Function..."<<std::endl;
  }
  public: void OnUpdate()
   {
     //control vehicle speed
     //this will maintain an approximate speed of 7 m/s

     //connect to the ray data to stop vehicle in case of obstruction
     //this will only use a rectangle in the direction of the wheels to detect obstacles.
     maxSpeed = 6.0;
     raySensor->GetRanges(ranges);
     center = steering_input*50 + 50;
     minRange = 100000.0;
     for(int i=0; i<ranges.size(); i++){
       if(ranges[i]*sin(abs(i-center)*3.14159/180.0) <=2.5){
         if(ranges[i]<minRange)
           minRange = ranges[i];
       }
     }
     if(minRange<10000.0){
       //linear so 6m/s at 20m at 0m/s at 2.5m
       maxSpeed = .343*minRange - .857;
     }

     if(veh->GetVehicleSpeedCOM() >= maxSpeed +1.0){
       braking_input = 0.5;
       throttle_input = 0.0;
     }
     else if(veh->GetVehicleSpeedCOM() <= maxSpeed /2.0){
       braking_input = 0.0;
       throttle_input = 0.2;
     }
     else if(veh->GetVehicleSpeedCOM() >= maxSpeed){
       braking_input = 0.3;
       throttle_input = 0.1;
     }
     else{
       braking_input = 0.15;
       throttle_input = 0.2;
     }


     for(int i =0; i<1; i++){
       //collect module information

       powertrain_torque = powertrain->GetOutputTorque();
       driveshaft_speed = veh->GetDriveshaftSpeed();
       for (int i = 0; i < num_wheels; i++) {
          tire_forces[i] = tires[i]->GetTireForce();
          wheel_states[i] = veh->GetWheelState(i);
       }

       //Update modules (process inputs from other modules)
       time = veh->GetSystem()->GetChTime();
       //driver->Update(time);
       //pursuitDriver->Update(vehicle);
       powertrain->Update(time, throttle_input, driveshaft_speed);
       veh->Update(time, steering_input, braking_input, powertrain_torque, tire_forces);
       terrain->Update(time);
       for (int i = 0; i < num_wheels; i++)
          tires[i]->Update(time, wheel_states[i], *terrain);

       // Advance simulation for one timestep for all modules
       //driver->Advance(step_size);
       powertrain->Advance(step_size);
       veh->Advance(step_size);
       terrain->Advance(step_size);
       for (int i = 0; i < num_wheels; i++)
          tires[i]->Advance(step_size);
     }

       //Communication and updates between Chrono and Gazebo
       _model1->SetWorldPose(math::Pose(
           math::Vector3(
               veh->GetChassisPos().x,
               veh->GetChassisPos().y,
               veh->GetChassisPos().z
           ),
           math::Quaternion(
               veh->GetChassisRot().e0,
               veh->GetChassisRot().e1,
               veh->GetChassisRot().e2,
               veh->GetChassisRot().e3
           )), "link");
        for(int j=0; j<wheels.size();j++){
          wheels[j]->SetWorldPose(math::Pose(
              math::Vector3(
                  veh->GetWheelPos(j).x,
                  veh->GetWheelPos(j).y,
                  veh->GetWheelPos(j).z
              ),
              math::Quaternion(
                  veh->GetWheelRot(j).e0,
                  veh->GetWheelRot(j).e1,
                  veh->GetWheelRot(j).e2,
                  veh->GetWheelRot(j).e3
              )), "link");
        }
        //std::cout<<"updating the cinderblocks\n";
   }

   void UpdateDriverInput(const std_msgs::Float64::ConstPtr& _msg)
    {
      this->steering_input = _msg->data;
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
    ros::Subscriber sub_;
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
    double         steering_input = 0.0;
    double         braking_input = 0.1;
    int num_axles;
    int num_wheels;
    double center;
    double minRange;
    double maxSpeed;

    //define the step size
    double step_size = 0.001;
    double iters = 100;
    double time = 0.0;

    //pointers to reference the models shared by chrono and gazebo
    boost::shared_ptr<sensors::RaySensor> raySensor;
    std::vector<ChSharedPtr<ChBody>> chronoModels;
    std::vector<ChSharedPtr<ChBody>> chCinderBlocks;
    std::vector<physics::ModelPtr> gzCinderBlocks;
    std::vector<ChSharedPtr<ChBody>> chLogs;
    std::vector<physics::ModelPtr> gzLogs;
    std::vector<physics::ModelPtr> wheels;
    physics::WorldPtr _world;
    physics::ModelPtr _model1;
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(chrono_gazebo)
