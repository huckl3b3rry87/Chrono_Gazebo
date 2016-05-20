/*
 * GcUtil.hh
 *
 *  Created on: Apr 16, 2016
 *      Author: leon
 */

#ifndef SRC_GCUTIL_HH_
#define SRC_GCUTIL_HH_

#include <core/ChQuaternion.h>
#include <core/ChVector.h>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>

namespace gc {

/* Get a Gazebo Pose from a Chrono vector and Chrono quaternion. */
gazebo::math::Pose GetGcPose(const chrono::ChVector<> vec,
		const chrono::ChQuaternion<> quat);

/* Get a Gazebo Pose from 7 doubles in the order of a vector (3d) and a quaternion (4d). */
gazebo::math::Pose GetGcPose(const double posrot[7]);

}
#endif /* SRC_GCUTIL_HH_ */
