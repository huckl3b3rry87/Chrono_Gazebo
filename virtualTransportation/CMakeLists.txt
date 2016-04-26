CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

find_package(gazebo REQUIRED)

FIND_PACKAGE(Chrono COMPONENTS Vehicle CONFIG)

find_library(OSMSCOUT osmscout)
message(STATUS "osmscout ${OSMSCOUT}")

#message(STATUS "CHRONOVEHICLE_INCLUDES ${CHRONO_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(SYSTEM ${CHRONO_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
SET(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
INCLUDE_DIRECTORIES(${SRC_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

add_library(chrono_gazebo SHARED
	${SRC_DIR}/GcLocalVehicle.cc
	${SRC_DIR}/GcLocalVehicle.hh
	${SRC_DIR}/GcNetworkUtil.cc 
	${SRC_DIR}/GcNetworkUtil.hh
	${SRC_DIR}/GcNetworkVehicle.cc
	${SRC_DIR}/GcNetworkVehicle.hh
	${SRC_DIR}/GcTypes.hh
	${SRC_DIR}/GcUtil.cc
	${SRC_DIR}/GcUtil.hh
	${SRC_DIR}/GcVehicle.hh
	${SRC_DIR}/GcVehicleBuilder.cc
	${SRC_DIR}/GcVehicleBuilder.hh
	${SRC_DIR}/OSM2Gc.cc
	${SRC_DIR}/OSM2Gc.hh
	chrono_gazebo.cc )

add_executable(GcServer
	${SRC_DIR}/GcServer.cc
	${SRC_DIR}/GcNetworkUtil.cc)

TARGET_LINK_LIBRARIES(chrono_gazebo ${CHRONO_LIBRARIES} ${GAZEBO_LIBRARIES} ${OSMSCOUT}) 

message(STATUS "CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}")
install(TARGETS chrono_gazebo
  DESTINATION lib
  COMPONENT shlib
)
