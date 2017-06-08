#include "conversions.h"

//TODO: this should be inlines
namespace cogimon {

//fcl::Transform3<double> kdl2fcl(const KDL::Frame &in) {
//	fcl::Transform3d out;
//	double x, y, z, w;
//	in.M.GetQuaternion(x, y, z, w);
//	fcl::Vector3d t(in.p[0], in.p[1], in.p[2]);
//	fcl::Quaterniond q(w, x, y, z);
//	out.rotate(q);
//	out.translate(t);
//	return out;
//}

//KDL vector to eigen
void convert(const KDL::Vector &in, eVector3 &out) {
	out << in.x(), in.y(), in.z();
}

void convert(const urdf::Vector3 &v, KDL::Vector &out) {
	out = KDL::Vector(v.x, v.y, v.z);
}

// construct rotation
void convert(const urdf::Rotation &r, KDL::Rotation &out) {
	out = KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
void convert(const urdf::Pose &in, KDL::Frame &out) {
	KDL::Rotation r;
	KDL::Vector p;
	convert(in.rotation, r);
	convert(in.position, p);

	out = KDL::Frame(r, p);
}

typedef boost::shared_ptr<const urdf::Link> linkPtr;
typedef boost::shared_ptr<const urdf::Joint> jointPtr;

}

