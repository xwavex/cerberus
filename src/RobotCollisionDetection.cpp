/** 
 * Author: Dennis Leroy Wigand
 * Date:   28 Feb 2017
 *
 */

#include <rtt/Component.hpp> // needed for the macro at the end of this file
#include <boost/lexical_cast.hpp>
#include "RobotCollisionDetection.hpp"

namespace cerberus {

RobotCollisionDetection::RobotCollisionDetection(std::string const & name) :
		RTT::TaskContext(name), in_jointFeedback_robot_1_flow(RTT::NoData), in_jointFeedback_robot_2_flow(
				RTT::NoData), in_jointTorqueCmd_robot_1_flow(RTT::NoData), in_jointTorqueCmd_robot_2_flow(
				RTT::NoData), DOFsize(7), threshold(0.20), ss(0), ee(0), stopPeerResult(
				false) {

	addOperation("setDOFsize", &RobotCollisionDetection::setDOFsize, this).doc(
			"set DOF size");
	addOperation("computeDistance", &RobotCollisionDetection::computeDistance,
			this);

	addOperation("addRobot", &RobotCollisionDetection::addRobot, this);

	addProperty("threshold", threshold).doc("The threshold for the distance to activate the emergency stop");
}

bool RobotCollisionDetection::configureHook() {
	in_jointFeedback_robot_1_var = rstrt::robot::JointState(DOFsize);
	in_jointFeedback_robot_1_port.setName("in_jointFeedback_robot_1_port");
	in_jointFeedback_robot_1_port.doc("Input port for the robot 1 feedback");
	ports()->addPort(in_jointFeedback_robot_1_port);
	in_jointFeedback_robot_1_flow = RTT::NoData;

	in_jointFeedback_robot_2_var = rstrt::robot::JointState(DOFsize);
	in_jointFeedback_robot_2_port.setName("in_jointFeedback_robot_2_port");
	in_jointFeedback_robot_2_port.doc("Input port for the robot 2 feedback");
	ports()->addPort(in_jointFeedback_robot_2_port);
	in_jointFeedback_robot_2_flow = RTT::NoData;

	out_emergency_port.setName("out_emergency_port");
	out_emergency_port.doc(
			"Output port for sending true, when the robot is about to collide (threshold-based)");
	out_emergency_port.setDataSample(false);
	ports()->addPort(out_emergency_port);

	joints_kdl_1.resize(DOFsize);
	joints_kdl_1.data.fill(0);

	joints_kdl_2.resize(DOFsize);
	joints_kdl_2.data.fill(0);
	return true;
}

bool RobotCollisionDetection::startHook() {
	return true;
}

void RobotCollisionDetection::computeDistance() {
	if (in_jointFeedback_robot_1_flow != RTT::NoData) {
		for (unsigned int i = 0; i < DOFsize; i++) {
			joints_kdl_1(i) = in_jointFeedback_robot_1_var.angles(i);
		}
	}

	if (in_jointFeedback_robot_2_flow != RTT::NoData) {
		for (unsigned int i = 0; i < DOFsize; i++) {
			joints_kdl_2(i) = in_jointFeedback_robot_2_var.angles(i);
		}
	}

	joint_updates["robot1"] = joints_kdl_1;
	joint_updates["robot2"] = joints_kdl_2;

	for (auto const& robot : _robots) {
		// update joint positions in all robots
		robot.second->transformAllLinkCollisionObjects(
				joint_updates[robot.first]);
	}

// test results
	double minDist = 10000000000000;
	fcl::DistanceResult<double> resultFinal;
	std::string link1, link2, r1name, r2name;

	for (auto const& robot_I : _robots) {
		std::vector<std::string> robot_i_link_names =
				robot_I.second->getAllLinkNames();
		for (auto const& robot_J : _robots) {
			std::vector<std::string> robot_j_link_names =
					robot_J.second->getAllLinkNames();
			if (robot_I.second != robot_J.second) { // avoid self collision test between robots
				fcl::DistanceRequest<double> request;
				request.gjk_solver_type = fcl::GST_INDEP;
				request.enable_nearest_points = true;

				for (int link_i = 0; link_i < robot_i_link_names.size();
						link_i++) {
					for (int link_j = 0; link_j < robot_j_link_names.size();
							link_j++) {
						// result will be returned via the collision result structure
						fcl::DistanceResult<double> result;

						fcl::CollisionObjectd* objI =
								robot_I.second->getCollisionObjectForLink(
										robot_i_link_names[link_i]).get();
						fcl::CollisionObjectd* objJ =
								robot_J.second->getCollisionObjectForLink(
										robot_j_link_names[link_j]).get();

						if (objI == 0) {
							RTT::log(RTT::Error) << "Collision Object "
									<< robot_i_link_names[link_i]
									<< " from robot " << robot_I.first
									<< "is NULL" << RTT::endlog();
							return;
						}
						if (objJ == 0) {
							RTT::log(RTT::Error) << "Collision Object "
									<< robot_j_link_names[link_j]
									<< " from robot " << robot_J.first
									<< "is NULL" << RTT::endlog();
							return;
						}

						// perform distance test
						fcl::distance(objI, objJ, request, result);

						if (result.min_distance < minDist) {
							minDist = result.min_distance;
							resultFinal = result;
							link1 = robot_i_link_names[link_i];
							link2 = robot_j_link_names[link_j];
							r1name = robot_I.first;
							r2name = robot_J.first;
						}

//						RTT::log(RTT::Warning) << "Collision Result: robot "
//								<< robot_I.first << " vs. robot "
//								<< robot_J.first << " between link "
//								<< robot_i_link_names[link_i] << " and link "
//								<< robot_j_link_names[link_j] << ":\n"
//								<< " min_distance: " << result.min_distance
//								<< "\n" << " nearest_points[0]: "
//								<< result.nearest_points[0] << "\n"
//								<< " nearest_points[1]: "
//								<< result.nearest_points[1] << "\n"
//								<< RTT::endlog();
					}
				}

			}
		}
	}

	RTT::log(RTT::Error) << "resultFinal Collision Result: robot " << r1name
			<< " vs. " << r2name << " between " << link1 << " : " << link2
			<< " min_distance: " << resultFinal.min_distance << "\n"
			<< " nearest_points[0]: " << resultFinal.nearest_points[0] << "\n"
			<< " nearest_points[1]: " << resultFinal.nearest_points[1] << "\n"
			<< RTT::endlog();

	if (resultFinal.min_distance < threshold) {
		out_emergency_port.write(true);
		RTT::log(RTT::Warning) << "Activate Emergency Trigger!"
				<< RTT::endlog();

		if (this->getPeerList().size() > 0) {
			for (auto p : getPeerList()) {
				stopPeerResult = this->getPeer(p)->stop();
				if (stopPeerResult) {
					RTT::log(RTT::Warning) << "Successfully stopped peer " << p
							<< "!";
				} else {
					RTT::log(RTT::Error) << "Could NOT stop peer " << p << "!";
				}
			}
		}
	}
}

void RobotCollisionDetection::updateHook() {
	ss = RTT::os::TimeService::Instance()->getTicks();

	in_jointFeedback_robot_1_flow = in_jointFeedback_robot_1_port.read(
			in_jointFeedback_robot_1_var);
	in_jointFeedback_robot_2_flow = in_jointFeedback_robot_2_port.read(
			in_jointFeedback_robot_2_var);

	computeDistance();

	ee = RTT::os::TimeService::Instance()->getTicks();
	RTT::log(RTT::Warning) << "Calculation took "
			<< 1E-6 * RTT::os::TimeService::ticks2nsecs(ee - ss) << " [ms]"
			<< RTT::endlog();
}

void RobotCollisionDetection::stopHook() {
}

void RobotCollisionDetection::cleanupHook() {
}

bool RobotCollisionDetection::addRobot(std::string robot_name, std::string urdf,
		std::string srdf, rstrt::geometry::Pose pose) {
	std::shared_ptr<RobotDescription> robot(new RobotDescription());
	bool error_result = robot->initialize(urdf, srdf);
	if (!error_result) {
		RTT::log(RTT::Error) << "Adding robot " << robot_name << " failed!"
				<< RTT::endlog();
		return false;
	}

	KDL::Frame origin(
			KDL::Rotation::Quaternion(pose.rotation.rotation[1],
					pose.rotation.rotation[2], pose.rotation.rotation[3],
					pose.rotation.rotation[0]),
			KDL::Vector(pose.translation.translation[0],
					pose.translation.translation[1],
					pose.translation.translation[2]));
	robot->setOrigin(origin);

	KDL::JntArray joints_kdl(7);
	robot->transformAllLinkCollisionObjects(joints_kdl);

	std::vector<std::string> robot_link_names = robot->getAllLinkNames();
	RTT::log(RTT::Debug) << "robot_link_names.size() = "
			<< robot_link_names.size() << RTT::endlog();
	for (int i = 0; i < robot_link_names.size(); i++) {
		fcl::CollisionObjectd* collObj = robot->getCollisionObjectForLink(
				robot_link_names[i]).get();
	}

	RTT::log(RTT::Info) << "Finished loading " << robot_name << "!"
			<< RTT::endlog();
	_robots[robot_name] = robot;
	return true;
}

void RobotCollisionDetection::setDOFsize(unsigned int DOFsize) {
	this->DOFsize = DOFsize;
}

}
//this macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cerberus::RobotCollisionDetection)
