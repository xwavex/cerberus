/** 
 * Author: Dennis Leroy Wigand
 * Date:   28 Feb 2017
 *
 */

#include "TemplateComponent.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file
#include <boost/lexical_cast.hpp>


TemplateComponent::TemplateComponent(std::string const & name) :
		RTT::TaskContext(name), in_jointFeedback_robot_1_flow(RTT::NoData), in_jointFeedback_robot_2_flow(
				RTT::NoData), in_jointTorqueCmd_robot_1_flow(RTT::NoData), in_jointTorqueCmd_robot_2_flow(
				RTT::NoData), DOFsize(7), threshold(0.20), ss(0), ee(0) {

	addOperation("setDOFsize", &TemplateComponent::setDOFsize, this).doc(
			"set DOF size");
	addOperation("computeDistance", &TemplateComponent::computeDistance, this);

	addOperation("addRobot", &TemplateComponent::addRobot, this);

	addProperty("threshold", threshold);
//	startTime = 0.0;
}

bool TemplateComponent::configureHook() {
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
	out_emergency_port.doc("Output port for sending true, when the robot is about to collide (threshold-based)");
	out_emergency_port.setDataSample(false);
	ports()->addPort(out_emergency_port);


	joints_kdl_1.resize(DOFsize);
	joints_kdl_1.data.fill(0);

	joints_kdl_2.resize(DOFsize);
	joints_kdl_2.data.fill(0);
	return true;
}

bool TemplateComponent::startHook() {
//	startTime = this->getSimulationTime();
	return true;
}

void TemplateComponent::computeDistance() {
//	RTT::log(RTT::Error) << "1" << RTT::endlog();
	if (in_jointFeedback_robot_1_flow != RTT::NoData) {
//		RTT::log(RTT::Error) << "2" << RTT::endlog();
		for (unsigned int i = 0; i < DOFsize; i++) {
			joints_kdl_1(i) = in_jointFeedback_robot_1_var.angles(i);
		}
//		RTT::log(RTT::Error) << "joints_kdl_1\n" << joints_kdl_1.data << RTT::endlog();
	}

	if (in_jointFeedback_robot_2_flow != RTT::NoData) {
		for (unsigned int i = 0; i < DOFsize; i++) {
			joints_kdl_2(i) = in_jointFeedback_robot_2_var.angles(i);
		}
//		RTT::log(RTT::Error) << "joints_kdl_2\n" << joints_kdl_2.data << RTT::endlog();
	}

	joint_updates["robot1"] = joints_kdl_1;
	joint_updates["robot2"] = joints_kdl_2;

//	RTT::log(RTT::Error) << "3 : size = " << _robots.size() << RTT::endlog();
	for (auto const& robot : _robots) {
		// update joint positions in all robots
		robot.second->transformAllLinkCollisionObjects(joint_updates[robot.first]);
//		RTT::log(RTT::Error) << "3a" << RTT::endlog();
//		RTT::log(RTT::Error) << "robots" << RTT::endlog();
	}
//	RTT::log(RTT::Error) << "4" << RTT::endlog();


//render TODO
//	std::vector<std::string> robot_1_link_names = _robots["robot1"]->getAllLinkNames();
//	for (int i = 0; i < robot_1_link_names.size(); i++) {
//		fcl::CollisionObjectd* collObj = _robots["robot1"]->getCollisionObjectForLink(robot_1_link_names[i]).get();
//		collObj->computeAABB();
//		fcl::AABB<double> aabb = collObj->getAABB();
//	}


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

//	RTT::log(RTT::Error) << "\n\n\n";
	RTT::log(RTT::Error) << "resultFinal Collision Result: robot " << r1name << " vs. " << r2name << " between " << link1 << " : " << link2
									<< " min_distance: " << resultFinal.min_distance << "\n"
									<< " nearest_points[0]: " << resultFinal.nearest_points[0] << "\n"
									<< " nearest_points[1]: " << resultFinal.nearest_points[1] << "\n"
									<< RTT::endlog();

	if (resultFinal.min_distance < threshold) {
		out_emergency_port.write(true);
		RTT::log(RTT::Error) << "Activate Emergency Trigger!" << RTT::endlog();
	}else{
		// perhaps we dont need this!
		out_emergency_port.write(false);
	}

}

void TemplateComponent::updateHook() {
//	double startTime = getSimulationTime();
	ss = RTT::os::TimeService::Instance()->getTicks();
	// this is the actual body of a component. it is called on each cycle
//	if (in_jointFeedback_robot_1_port.connected()) {
	// read data and save state of data into "Flow", which can be "NewData", "OldData" or "NoData".
	in_jointFeedback_robot_1_flow = in_jointFeedback_robot_1_port.read(
			in_jointFeedback_robot_1_var);
	in_jointFeedback_robot_2_flow = in_jointFeedback_robot_2_port.read(
			in_jointFeedback_robot_2_var);


	computeDistance();
//	}

	// out_torques_port.write(out_torques_var);
//	double endTime = getSimulationTime();
	ee = RTT::os::TimeService::Instance()->getTicks();
	RTT::log(RTT::Warning) << "Calculation took " << 1E-9
			* RTT::os::TimeService::ticks2nsecs(ee - ss) << RTT::endlog();
}

void TemplateComponent::stopHook() {
}

void TemplateComponent::cleanupHook() {
}

// void TemplateComponent::preparePorts(){
//     if (portsArePrepared){
//         ports()->removePort("in_torquesA_port");
//         ports()->removePort("in_torquesB_port");
//         ports()->removePort("out_torques_port");
//     }

//     //prepare input
//     in_torquesA_var = rstrt::dynamics::JointTorques(DOFsize);
//     in_torquesA_port.setName("in_torquesA_port");
//     in_torquesA_port.doc("Input port for reading torquesA values");
//     ports()->addPort(in_torquesA_port);
//     in_torquesA_flow = RTT::NoData;

//     in_torquesB_var = rstrt::dynamics::JointTorques(DOFsize);
//     in_torquesB_port.setName("in_torquesB_port");
//     in_torquesB_port.doc("Input port for reading torquesB values");
//     ports()->addPort(in_torquesB_port);
//     in_torquesB_flow = RTT::NoData;

//     //prepare output
//     out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
//     out_torques_var.torques.setZero();
//     out_torques_port.setName("out_torques_port");
//     out_torques_port.doc("Output port for sending torque values");
//     out_torques_port.setDataSample(out_torques_var);
//     ports()->addPort(out_torques_port);

//     portsArePrepared = true;
// }

double TemplateComponent::getSimulationTime() {
	return 1E-9
			* RTT::os::TimeService::ticks2nsecs(
					RTT::os::TimeService::Instance()->getTicks());
}

//bool TemplateComponent::addRobot(std::string robot_name, std::string urdf,
//		std::string srdf) {
//	std::shared_ptr<RobotDescription> robot(new RobotDescription());
//	bool error_result = robot->initialize(urdf, srdf);
//	if (!error_result)
//		return false;
//
//	_robots[robot_name] = robot;
//	return true;
//}

bool TemplateComponent::addRobot(std::string robot_name, std::string urdf,
		std::string srdf, rstrt::geometry::Pose pose) {
	std::shared_ptr<RobotDescription> robot(new RobotDescription());
	bool error_result = robot->initialize(urdf, srdf);
	if (!error_result) {
		RTT::log(RTT::Error) << "Adding robot " << robot_name << " failed!" << RTT::endlog();
		return false;
	}

	KDL::Frame origin(
			KDL::Rotation::Quaternion(pose.rotation.rotation[1],
					pose.rotation.rotation[2], pose.rotation.rotation[3],
					pose.rotation.rotation[0]),
			KDL::Vector(pose.translation.translation[0],
					pose.translation.translation[1],
					pose.translation.translation[2]));
	RTT::log(RTT::Warning) << "KDL::Frame origin " << origin << RTT::endlog();
	robot->setOrigin(origin);


	KDL::JntArray joints_kdl(7);
	robot->transformAllLinkCollisionObjects(joints_kdl);

	std::vector<std::string> robot_link_names = robot->getAllLinkNames();
	RTT::log(RTT::Error) << "robot_link_names.size() = " << robot_link_names.size() << RTT::endlog();
	for (int i = 0; i < robot_link_names.size(); i++) {
//		RTT::log(RTT::Warning) << "1" << RTT::endlog();

		fcl::CollisionObjectd* collObj = robot->getCollisionObjectForLink(robot_link_names[i]).get();

//		std::cout << "outtt TranslationOrigin:\n" << collObj->getTranslation() << std::endl;
//		std::cout << "outtt RotationOrigin:\n" << collObj->getRotation() << std::endl;

//		RTT::log(RTT::Warning) << "4" << RTT::endlog();
	}

	RTT::log(RTT::Warning) << "Finished!" << RTT::endlog();
	_robots[robot_name] = robot;

	return true;
}

void TemplateComponent::setDOFsize(unsigned int DOFsize) {
	this->DOFsize = DOFsize;
}

//this macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TemplateComponent)
