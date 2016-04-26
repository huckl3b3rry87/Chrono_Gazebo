/*
 * GcTypes.hh
 *
 *  Created on: Apr 16, 2016
 *      Author: leon
 */

#ifndef SRC_GCTYPES_HH_
#define SRC_GCTYPES_HH_

#include <chrono_vehicle/ChPowertrain.h>
#include <chrono_vehicle/ChTerrain.h>
#include <chrono_vehicle/driver/ChPathFollowerACCDriver.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h>
#include <chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h>
#include <memory>

namespace gc {

typedef std::shared_ptr<chrono::vehicle::ChTerrain> ChTerrainPtr;
typedef std::shared_ptr<chrono::vehicle::ChWheeledVehicle> ChWheeledVehiclePtr;
typedef std::shared_ptr<chrono::vehicle::ChPowertrain> ChPowertrainPtr;
typedef std::shared_ptr<chrono::vehicle::ChRigidTire> ChRigidTirePtr;
typedef std::shared_ptr<chrono::vehicle::ChPathFollowerACCDriver> ChDriverPtr;

}
#endif /* SRC_GCTYPES_HH_ */
