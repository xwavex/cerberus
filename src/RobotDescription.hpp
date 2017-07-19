#ifndef __ROBOT_DESCRIPTION_FCL_HPP_
#define __ROBOT_DESCRIPTION_FCL_HPP_

#include <assimp/scene.h>
#include <assimp/mesh.h>

#include <vector>
#include <fcl/config.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include <cogimon-kdl-parser/kdl_parser.hpp>
#include <cogimon-kdl-parser/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <XBotCoreModel.h>
#include <srdfdom_advr/model.h>

#include "conversions.h"

class RobotDescription
{

public:
	RobotDescription();
	~RobotDescription();
	bool initialize(const std::string &URDF_path, const std::string &SRDF_path);
	std::shared_ptr<urdf::Model> getUrdfModel_ptr();


	/** ####### REAL-TIME CRITICAL PART ####### */
	bool transformAllLinkCollisionObjects(const KDL::JntArray &joints_kdl);
	std::shared_ptr<fcl::CollisionObjectd> getCollisionObjectForLink(const std::string &link_name);


	int getNumberOfJointsInChain(const std::string &chain_name);
	std::string getActiveKinematicChainName();
	int getNumberOfLinks();
	std::vector<std::string> getAllLinkNames();
	KDL::Frame getOrigin();
	void setOrigin(const KDL::Frame origin);

private:
	/** ####### REAL-TIME CRITICAL PART ####### */
	void kdl2fcl(const KDL::Frame &in, fcl::Transform3d &out);

//	const boost::shared_ptr<fcl::Transform3f> parseCapsule(
//	        const boost::shared_ptr<const urdf::Link>& link,
//	        boost::shared_ptr<fcl::CollisionObject>& collision_object);

	std::string _urdf_path;
	std::string _srdf_path;
	bool loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path);
	bool _models_loaded;
	std::string _active_kinematic_chain_name;
	XBot::XBotCoreModel _xbotcore_model;
	std::vector<std::string> _link_names;

	KDL::Frame _origin;

	// Setting transformation for geometry
	KDL::Frame _fkin_frame;

	std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > _collision_objects;
	std::map<std::string, fcl::Transform3d> _origin_transforms;
	std::map<std::string, fcl::Transform3d> _joint_transforms;
	std::map<std::string, urdf::Vector3> _joint_axis;
	std::map<std::string, KDL::Frame> _link_frames;

	boost::shared_ptr< KDL::TreeFkSolverPos_recursive > robot_fk_;

	std::string _urdf_folder;
	std::shared_ptr<urdf::Model> robot_model;

	bool parseCollisionObjects(std::shared_ptr<urdf::Model> robot_model, const KDL::Tree &tree_, const XBot::XBotCoreModel &_xbotcore_model, const std::string &activeChainName, const std::vector<std::string> &link_names);

	void get_tree_segment_names_recursive(const KDL::SegmentMap::const_iterator& it,
			std::vector<std::string> &link_names, bool no_fixed_joints);

	void get_tree_segment_names(KDL::Tree tree, std::vector<std::string> &link_names,
			bool non_fixed_joints);
};

#endif
