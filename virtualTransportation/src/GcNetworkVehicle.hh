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
//gcVehicle defines a vehicle based on chrono_vehicle and gazebo relying on the
//dynamics and definitions provided by chrono and the world geometry, sensors,
//and visuals within gazebo
//
// =============================================================================

#ifndef SRC_GCNETWORKVEHICLE_HH_
#define SRC_GCNETWORKVEHICLE_HH_

//includes

#include <gazebo/physics/PhysicsTypes.hh>
#include <vector>

#include "GcVehicle.hh"

class GcNetworkVehicle: public GcVehicle {

public:

	// constructor
	GcNetworkVehicle(int id,
			int sockfd,
			const gazebo::physics::WorldPtr world);

	void Advance(double step) override;

	bool Init() override;

	int GetId() const override {
		return m_id;
	}

private:
	int m_id;
	int m_sockfd;
	bool m_initialized = false;

	// gazebo components
	gazebo::physics::WorldPtr m_world;
	gazebo::physics::ModelPtr m_gazeboVehicle;
	std::vector<gazebo::physics::LinkPtr> m_gazeboWheels;

};

#endif /* SRC_GCNETWORKVEHICLE_HH_ */
