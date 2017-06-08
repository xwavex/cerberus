#include <cogimon-kdl-parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <cogimon-kdl-parser/model.h>
#include <iostream>
#include <kdl/treefksolverpos_recursive.hpp>

#include <rst-rt/robot/JointState.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/ai_assert.h>
#include <assimp/scene.h>
#include <assimp/mesh.h>

#include <XBotCoreModel.h>
#include <srdfdom_advr/model.h>

#include "conversions.h"

#include <boost/shared_ptr.hpp>

#include "RobotDescription.hpp"

#include <fcl/narrowphase/distance.h>

using namespace KDL;
using namespace std;
using namespace urdf;

using namespace Assimp;

XBot::XBotCoreModel _xbotcore_model;
//Vector with fcl collision geometries
std::map<std::string, std::shared_ptr<fcl::CollisionGeometryd> > shapes_;
std::map<std::string, std::shared_ptr<fcl::Capsuled> > capsules_;
std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > collision_objects_;

//Vector with local transform of the collision shape in the corresponding link
std::map<std::string,KDL::Frame> shape_frames;

string activeChainName;
//get link names IMPORTANT!
	std::vector<std::string> link_names;



void printLink(const SegmentMap::const_iterator& link, const std::string& prefix)
{
  cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has "
       << GetTreeElementChildren(link->second).size() << " children" << endl;
  for (unsigned int i=0; i < GetTreeElementChildren(link->second).size(); i++)
      printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
}


int main(int argc, char** argv) {

	std::vector<RobotDescription> robots;

	RobotDescription r1;
	r1.initialize("/home/dwigand/citk/systems/cogimon-minimal-nightly/share/gazebo/models/cogimon/kuka-lwr-4plus/model.urdf", "/home/dwigand/citk/systems/cogimon-minimal-nightly/share/gazebo/models/cogimon/kuka-lwr-4plus/model.srdf");

	RobotDescription r2;
	r2.initialize("/home/dwigand/citk/systems/cogimon-minimal-nightly/share/gazebo/models/cogimon/kuka-lwr-4plus/model.urdf", "/home/dwigand/citk/systems/cogimon-minimal-nightly/share/gazebo/models/cogimon/kuka-lwr-4plus/model.srdf");

	robots.push_back(r1);

	// HARDCODE position offset in X
	KDL::Frame a(KDL::Vector(1.5,0,0));
	r2.setOrigin(a);
	robots.push_back(r2);


	// TODO update joint states
	KDL::JntArray joints_kdl;

	//get current joint position based on joint numbers in chain!
	rstrt::robot::JointState currentJointState = rstrt::robot::JointState(7);
	currentJointState.angles[0] = 1.35;
	currentJointState.angles[1] = 0.23;
	currentJointState.angles[2] = 0.44;
	currentJointState.angles[3] = 1.2;
	currentJointState.angles[4] = -0.3;
	currentJointState.angles[5] = -0.03;
	currentJointState.angles[6] = 0.0;
	std::cout << currentJointState << std::endl;

	joints_kdl.resize(7);

//	assert(in.size() == out.rows());
	for(unsigned int i=0; i<7; ++i){
		joints_kdl(i) = currentJointState.angles[i];
	}

	// TODO second one
	KDL::JntArray joints_kdl2;
	//get current joint position based on joint numbers in chain!
	rstrt::robot::JointState currentJointState2 = rstrt::robot::JointState(7);
	currentJointState2.angles[0] = 0.0;
	currentJointState2.angles[1] = 0.0;
	currentJointState2.angles[2] = 0.0;
	currentJointState2.angles[3] = 0.0;
	currentJointState2.angles[4] = 0.0;
	currentJointState2.angles[5] = 0.0;
	currentJointState2.angles[6] = 0.0;
	joints_kdl2.resize(7);
	for(unsigned int i=0; i<7; ++i){
		joints_kdl2(i) = currentJointState2.angles[i];
	}

	std::vector<KDL::JntArray> joint_updates;
	joint_updates.reserve(2);
	joint_updates.push_back(joints_kdl);
	joint_updates.push_back(joints_kdl2);




	for (int i = 0; i < robots.size(); i++) {
		// update joint positions in all robots
		robots[i].transformAllLinkCollisionObjects(joint_updates[i]);
	}

	double minDist = 10000000000000;
	fcl::DistanceResult<double> resultFinal;
	std::string link1, link2;


	for (int i = 0; i < robots.size(); i++) {
		std::vector<std::string> robot_i_link_names = robots[i].getAllLinkNames();

		for (int j = 0; j < robots.size(); j++) {
			std::vector<std::string> robot_j_link_names = robots[j].getAllLinkNames();

			if (i != j) { // avoid self collision test between robots

				fcl::DistanceRequest<double> request;
				request.gjk_solver_type = fcl::GST_INDEP;
				request.enable_nearest_points = true;

				for (int link_i = 0; link_i < robot_i_link_names.size(); link_i++) {
					for (int link_j = 0; link_j < robot_j_link_names.size(); link_j++) {
						// result will be returned via the collision result structure
						fcl::DistanceResult<double> result;

						fcl::CollisionObjectd* objI = robots[i].getCollisionObjectForLink(robot_i_link_names[link_i]).get();
						fcl::CollisionObjectd* objJ = robots[j].getCollisionObjectForLink(robot_j_link_names[link_j]).get();

						if (objI == 0) {
							cerr << "Collision Object " << robot_i_link_names[link_i] << " from robot " << i << "is NULL" << endl; return -1;
						}
						if (objJ == 0) {
							cerr << "Collision Object " << robot_j_link_names[link_j] << " from robot " << j << "is NULL" << endl; return -1;
						}


						// perform distance test
						fcl::distance(objI, objJ, request, result);

						if (result.min_distance < minDist) {
							minDist = result.min_distance;
							resultFinal = result;
							link1 = robot_i_link_names[link_i];
							link2 = robot_j_link_names[link_j];
						}

						std::cout << "Collision Result: robot " << i << " vs. robot " << j << " between link " << robot_i_link_names[link_i] << " and link " << robot_j_link_names[link_j]
							<< ":\n"
							<< " min_distance: " << result.min_distance << "\n"
							<< " nearest_points[0]: " << result.nearest_points[0] << "\n"
							<< " nearest_points[1]: " << result.nearest_points[1] << "\n"
							<< std::endl;

//						// p1Homo, p2Homo newly computed points by FCL
//						// absolutely computed w.r.t. base-frame
//						fcl::Transform3f p1Homo(result.nearest_points[0]);
//						fcl::Transform3f p2Homo(result.nearest_points[1]);
					}
				}

			}
		}

		cout << "\n\n\n";
	}

	cout << "\n\n\n";
	std::cout << "resultFinal Collision Result: robot " << link1 << " : " << link2
								<< " min_distance: " << resultFinal.min_distance << "\n"
								<< " nearest_points[0]: " << resultFinal.nearest_points[0] << "\n"
								<< " nearest_points[1]: " << resultFinal.nearest_points[1] << "\n"
								<< std::endl;


}




