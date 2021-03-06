/** 
 * Author: Dennis Leroy Wigand
 * Date:   28 Feb 2017
 *
 */

#ifndef CERBERUS_ROBOT_COLLISION_DETECTION_HPP
#define CERBERUS_ROBOT_COLLISION_DETECTION_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <string>

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>

#include <rst-rt/geometry/Pose.hpp>

#include <boost/lexical_cast.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

#include "conversions.h"

#include "RobotDescription.hpp"

#include <fcl/narrowphase/distance.h>

namespace cerberus {

class RobotCollisionDetection: public RTT::TaskContext {
public:
	RobotCollisionDetection(std::string const & name);

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	bool addRobot(std::string robot_name, std::string urdf, std::string srdf,
			rstrt::geometry::Pose pose);
	void setDOFsize(unsigned int DOFsize);

	void computeDistance();
//    bool setActiveKinChain(std::string robot_name, std::string chain_name);

private:
	// TODO generate output ports accordingly!

	// Declare input ports and their datatypes
	RTT::InputPort<rstrt::robot::JointState> in_jointFeedback_robot_1_port;
	RTT::InputPort<rstrt::robot::JointState> in_jointFeedback_robot_2_port;

	RTT::InputPort<rstrt::dynamics::JointTorques> in_jointTorqueCmd_robot_1_port;
	RTT::InputPort<rstrt::dynamics::JointTorques> in_jointTorqueCmd_robot_2_port;

	// Variables
	rstrt::robot::JointState in_jointFeedback_robot_1_var;
	rstrt::robot::JointState in_jointFeedback_robot_2_var;

	rstrt::dynamics::JointTorques in_jointTorqueCmd_robot_1_var;
	rstrt::dynamics::JointTorques in_jointTorqueCmd_robot_2_var;

	// Data flows
	RTT::FlowStatus in_jointFeedback_robot_1_flow;
	RTT::FlowStatus in_jointFeedback_robot_2_flow;

	RTT::FlowStatus in_jointTorqueCmd_robot_1_flow;
	RTT::FlowStatus in_jointTorqueCmd_robot_2_flow;

	std::map<std::string, std::shared_ptr<RobotDescription> > _robots;

	RTT::OutputPort<bool> out_emergency_port;

	unsigned int DOFsize;
	// in meters
	double threshold;
	long long ss, ee;
	bool stopPeerResult;

	KDL::JntArray joints_kdl_1, joints_kdl_2;
	std::map<std::string, KDL::JntArray> joint_updates;
};

}
#endif

