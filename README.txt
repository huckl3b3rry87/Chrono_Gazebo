This is a Chrono-Gazebo simulation of a vehicle. This simulates an autonomous vehicle using sensors from gazebo and vehicle dynamics from chrono::Vehicle.

NOTE: master branch out of date - asher-dev is the branch with the latest updates

INSTALL:
  Install ROS (only Indigo tested) from deb or source
    Instructions: http://wiki.ros.org/ROS/Installation
  
  Install gazebo_ros packages from source
    Instructions: http://gazebosim.org/tutorials?tut=ros_installing&ver=1.9%2B&cat=connect_ros
  
  Install Gazebo7 from source - DO NOT install bullet (causes issues for now)
    Instructions: http://gazebosim.org/tutorials?tut=install_from_source&ver=default&cat=install
  
  Install Chrono
    Instructions:
      clone https://github.com/projectchrono/chrono
      using cmake: 
        source should be the top directory
        build directory can be wherever
        enable module_vehicle 
      make; sudo make install

BUILD:
  clone this repository
  create build directory inside gazonoVehicle - this will give correct path for vehicle data
  cmake ..; make
  Add build directory to Gazebo plugin path:
  from inside build directory:
  
  repeat build process for virtualTransportation
  -in addition, need to install osmscout: https://github.com/Framstag/libosmscout


RUN:
  $./launch from inside gazonoVehicle directy (does not currently work - needs updating to latest Chrono and gazebo7
  $./launch from inside virtualTransportation directory
  
  Can/should modify launch script to run desired configuration
  
  The world file saves camera images by default to relative directories Captures/TopCam/... and 
  Captures/SideCam/... The world file can be changed to save to different directory or not save at all.

OTHER NOTES:

CURRENTLY TESTED ON UBUNTU TRUSTY 14.04 and ArchLinux
Build mode for Chrono and Gazebo should be RELEASE for speed
Run with gzserver istead of gazebo for speed/larger configurations


Strange issues:
  Every now and then, gazebo will crash upon startup. Rerunning should solve issue

If errors occur, likely causes are not having correct files sourced,
incorrect location of data, ...
