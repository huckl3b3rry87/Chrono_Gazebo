/*
 * GcUtil.cc
 *
 *  Created on: Apr 16, 2016
 *      Author: leon
 */

#include "GcUtil.hh"

namespace gc {

gazebo::math::Pose GetGcPose(const chrono::ChVector<> vec,
		const chrono::ChQuaternion<> quat) {
	return gazebo::math::Pose(gazebo::math::Vector3(vec.x, vec.y, vec.z),
			gazebo::math::Quaternion(quat.e0, quat.e1, quat.e2, quat.e3));
}

gazebo::math::Pose GetGcPose(const double posrot[7]) {
	return gazebo::math::Pose(
			gazebo::math::Vector3(posrot[0], posrot[1], posrot[2]),
			gazebo::math::Quaternion(posrot[3], posrot[4], posrot[5], posrot[6]));
}
}

