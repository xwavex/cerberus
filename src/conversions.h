#ifndef _CONVERSIONS_H_
#define _CONVERSIONS_H_

#include <typeinfo>
#include "math_utils.h"
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <assert.h>

#include <fcl/config.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include <urdf/model.h>

/// @todo this should be inlines

namespace cogimon {

//KDL vector to eigen
void convert(const KDL::Vector &in, eVector3 &out);

void convert(const urdf::Vector3 &v, KDL::Vector &out);

// construct rotation
void convert(const urdf::Rotation &r, KDL::Rotation &out);

// construct pose
void convert(const urdf::Pose &in, KDL::Frame &out);

//fcl::Transform3<double> kdl2fcl(KDL::Frame &in);

}
#endif
