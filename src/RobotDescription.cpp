#include "RobotDescription.hpp"

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

#include <boost/shared_ptr.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <string>
#include <iostream>
#include <fstream>

using namespace KDL;
using namespace std;
using namespace urdf;

using namespace Assimp;

/** ####### REAL-TIME CRITICAL PART (START) ####### */
void RobotDescription::kdl2fcl(const KDL::Frame &in, fcl::Transform3d &out) {
	out.setIdentity();
	double x, y, z, w;
	in.M.GetQuaternion(x, y, z, w);
	fcl::Vector3d t(in.p[0], in.p[1], in.p[2]);
	fcl::Quaterniond q(w, x, y, z);
	out.rotate(q);
	out.translate(t);
}
/** ####### REAL-TIME CRITICAL PART (END) ####### */


/**
* Constructs a bounding box mesh from the given bounding box
*/
RobotDescription::RobotDescription(): _models_loaded(false) {

}

/**
*	Clears all loaded MeshEntries
**/
RobotDescription::~RobotDescription()
{

}

bool RobotDescription::loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path) {
    if(!_models_loaded)
    {
        _urdf_path = URDF_path;
        _srdf_path = SRDF_path;

        std::cout<<"URDF path: "<<_urdf_path<<std::endl;
        std::cout<<"SRDF path: "<<_srdf_path<<std::endl;

        _models_loaded = _xbotcore_model.init(_urdf_path, _srdf_path);

        for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
        	std::cout<<"chain #"<<i<<" "<<_xbotcore_model.get_chain_names()[i]<<std::endl;
            std::vector<std::string> enabled_joints_in_chain_i;
            _xbotcore_model.get_enabled_joints_in_chain(_xbotcore_model.get_chain_names()[i], enabled_joints_in_chain_i);
            for(unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j)
            	std::cout<<"  "<<enabled_joints_in_chain_i[j]<<std::endl;
        }
    }
    else
    	std::cout<<"URDF and SRDF have been already loaded!"<<std::endl;
    return _models_loaded;
}

std::shared_ptr<urdf::Model> RobotDescription::getUrdfModel_ptr() {
	return robot_model;
}

bool RobotDescription::initialize(const std::string &URDF_path, const std::string &SRDF_path) {
	// Parse XBOTCOREMODEL for chains and disables joints
	bool no_error = loadURDFAndSRDF(URDF_path, SRDF_path);
	if (!no_error) {
		cerr << "Error loading " << URDF_path << " and " << SRDF_path << endl;
		return false;
	}

	// Get base directory for URDF:
	boost::filesystem::path urdfFileFS(URDF_path);
	_urdf_folder = urdfFileFS.parent_path().string();

	cout << "Robot Repository Base Directory: " << _urdf_folder << endl;

	robot_model = std::shared_ptr<urdf::Model>(new urdf::Model());

	// Parse to have a plain URDF instance of the loaded model
	if (!robot_model->initFile(URDF_path)) {
		cerr << "Could not generate robot model" << endl; return false;
	}

	// Not sure is this is necessary, because is could already be present in the XBOTCOREMODEL
	Tree my_tree;
	if (!kdl_parser::treeFromUrdfModel(*robot_model, my_tree)) {
		cerr << "Could not extract kdl tree" << endl; return false;
	}

	// Set first chain as default
	const std::vector<std::string> chain_names = _xbotcore_model.get_chain_names();
	if (chain_names.size() > 0) {
		_active_kinematic_chain_name = _xbotcore_model.get_chain_names()[0];
	} else {
		cerr << "Could not find a single kinematic chain" << endl; return false;
	}

	// IMPORTANT get the link names
	get_tree_segment_names(my_tree, _link_names, false);
	for (unsigned int i = 0; i < _link_names.size(); ++i) {
		std::cout << "my links [" << i << "] = " << _link_names[i] << std::endl;
	}

	_origin = KDL::Frame(KDL::Vector(0,0,0));

	robot_fk_.reset(new KDL::TreeFkSolverPos_recursive(my_tree));

	// Parse the collision models for the robot model
	return parseCollisionObjects(robot_model, my_tree, _xbotcore_model, _active_kinematic_chain_name, _link_names);
}



bool RobotDescription::parseCollisionObjects(std::shared_ptr<urdf::Model> robot_model, const KDL::Tree &tree_, const XBot::XBotCoreModel &_xbotcore_model, const std::string &activeChainName, const std::vector<std::string> &link_names) {
	std::cout << "parsing collision objects" << std::endl;
	std::cout << "number of joints of tree "<< tree_.getNrOfJoints() << std::endl;


	for(unsigned int i = 0; i < link_names.size(); ++i) {
		std::cout << "------l------" << std::endl;
		boost::shared_ptr<urdf::Link> link;
		urdf::JointConstSharedPtr joint;

		robot_model->getLink(link_names[i], link);
		joint = robot_model->getJoint(link->parent_joint->name);

		if (link->collision) {
			std::cout << "Collision for link: " << link->name << std::endl;
			std::cout << "Collision for link: " << link->name << " is of type " << link->collision->geometry->type << std::endl;

			if (link->collision->geometry->type == urdf::Geometry::MESH) {

				boost::shared_ptr<::urdf::Mesh> collisionGeometry = boost::dynamic_pointer_cast<::urdf::Mesh>(link->collision->geometry);
				std::string collFileName = collisionGeometry->filename;

				std::size_t found = collFileName.find("package://");
				if (found != std::string::npos) {
					cerr << "Couldn't find mesh for link " << link->name << ". String is malformed and uses ROS terminology (package://). Use gazebo conform (model://) syntax in " << collFileName << endl; return false;
				}

				// replace first occurrence of model:// by ../
				boost::replace_first(collFileName, "model://", "../");

				boost::filesystem::path file(collFileName);
				std::string loadpath;
				if (file.is_absolute()) {
					// simple
					loadpath = collFileName;
				} else if (file.is_relative()) {
					std::stringstream ss;
					ss << _urdf_folder << '/' << collFileName;
					loadpath = ss.str();
				}
				boost::filesystem::path resultPath(loadpath);
				// in boost 1.60 use lexically_normal
				std::string resultingPath = resultPath.normalize().string();
				if (resultingPath == "") {
					cerr << "Mesh path is empty for link " << link->name << endl; return false;
				}


				/** TODO ################## OUTSOURCE INTO MESH (START) ##################*/

				cout << "resultingPath: " << resultingPath << endl;
				if (!boost::filesystem::exists(resultPath)) {
					cerr << "Mesh " << resultingPath << " does not exist for link " << link->name << endl; return false;
				}

				// Import mesh (STL or DEA, etc...)
				Importer importer;
				const aiScene *scene = importer.ReadFile(resultingPath, 0);
				if(!scene) {
					std::cerr << "Unable to load mesh: " << importer.GetErrorString() << std::endl;
					return false;
				}

				std::vector<aiMesh*> meshEntries;
				for(int i = 0; i < scene->mNumMeshes; i++) {
					meshEntries.push_back(scene->mMeshes[i]);
				}


				double scaleX = 1.0;
				double scaleY = 1.0;
				double scaleZ = 1.0;

				// TODO DANGER!!! REAL HARD HACK for scale!
				if (collisionGeometry->scale.x >= 100) {
					scaleX = collisionGeometry->scale.x * 0.01;
				}
				if (collisionGeometry->scale.y >= 100) {
					scaleY = collisionGeometry->scale.y * 0.01;
				}
				if (collisionGeometry->scale.z >= 100) {
					scaleZ = collisionGeometry->scale.z * 0.01;
				}

				for (aiMesh* entry : meshEntries) {
					// set up as collision object!
					// set mesh triangles and vertices indices
					int elementCount = entry->mNumFaces * 3;
					int verticeCnt = entry->mNumVertices;
					int faceCnt = entry->mNumFaces;
					int normalCnt = 0;

					std::vector<fcl::Vector3d > vertices;
					if(entry->HasPositions()) {
						for(int i = 0; i < entry->mNumVertices; ++i) {
							// DLW  scale vertices!
							fcl::Vector3d v(entry->mVertices[i].x * scaleX,
										 entry->mVertices[i].y * scaleY,
										 entry->mVertices[i].z * scaleZ);
							vertices.push_back(v);
						}

						std::cout << "Vertices loaded! " << verticeCnt << std::endl;
					}


					std::vector<fcl::Triangle> triangles;
					if(entry->HasFaces()) {
						for(int i = 0; i < entry->mNumFaces; ++i) {
							fcl::Triangle t(entry->mFaces[i].mIndices[0],
											entry->mFaces[i].mIndices[1],
											entry->mFaces[i].mIndices[2]);
							triangles.push_back(t);
						}

						std::cout << "Faces loaded! " << faceCnt << std::endl;
					}

					float* normals_;
					if(entry->HasNormals()) {
						normalCnt = entry->mNumVertices * 3;
						normals_ = new float[entry->mNumVertices * 3];
						for(int i = 0; i < entry->mNumVertices; ++i) {
							normals_[i * 3] = entry->mNormals[i].x;
							normals_[i * 3 + 1] = entry->mNormals[i].y;
							normals_[i * 3 + 2] = entry->mNormals[i].z;
						}
						std::cout << "Normals loaded!" << std::endl;
					}

					/** TODO ################## OUTSOURCE INTO MESH (END) ##################*/

					std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> model = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

					// add the mesh data into the BVHModel structure
					model->beginModel();
					model->addSubModel(vertices, triangles);
					model->endModel();
//					model->computeLocalAABB();

					std::shared_ptr<fcl::CollisionObjectd> collisionObj_ = std::shared_ptr<fcl::CollisionObjectd>(new fcl::CollisionObjectd(model));


					// Transform collision object now by Transformation from URDF! TODO
					urdf::Vector3 position = joint->parent_to_joint_origin_transform.position;
					urdf::Rotation rotation = joint->parent_to_joint_origin_transform.rotation;
					urdf::Vector3 axis = joint->axis;
					double r, p, y;
					rotation.getRPY(r, p, y);
					double pqx, pqy, pqz, pqw;
					link->collision->origin.rotation.getQuaternion(pqx, pqy, pqz, pqw);
					double por, pop, poy;
					link->collision->origin.rotation.getRPY(por, pop, poy);
					std::cout << " p[" << position.x << " " << position.y << " " << position.z << "] r[" << r << " " << p << " " << y << "] op[" <<
							link->collision->origin.position.x << " " << link->collision->origin.position.y << " " << link->collision->origin.position.z << "] or[" << por << " " << pop << " " << poy << "]" << std::endl;


					fcl::Transform3d out;
					out.setIdentity();
					fcl::Vector3d t(link->collision->origin.position.x, link->collision->origin.position.y, link->collision->origin.position.z);
					out.translate(t);
					fcl::Quaterniond q(pqw, pqx, pqy, pqz);
					std::cout << "Quaterniond: pqw = " << pqw << ", pqx = " << pqx << ", pqy = " << pqy << ", pqz = " << pqz << std::endl;
					out.rotate(q);

					fcl::Transform3d out2;
					out2.setIdentity();
					fcl::Vector3d t2(position.x, position.y, position.z);
					double pqx2, pqy2, pqz2, pqw2;
					rotation.getQuaternion(pqx2, pqy2, pqz2, pqw2);
					out2.translate(t2);
					fcl::Quaterniond q2(pqw2, pqx2, pqy2, pqz2);
					std::cout << "Quaterniond2: pqw2 = " << pqw2 << ", pqx2 = " << pqx2 << ", pqy2 = " << pqy2 << ", pqz2 = " << pqz2 << std::endl;
					out2.rotate(q2);

//					std::cout << "out:\n" << out.rotation() << std::endl;
//					std::cout << "out2:\n" << out2.rotation() << std::endl;
//					std::cout << "out2*out:\n" << (out2 * out).rotation() << std::endl;
//					std::cout << "axis:\n" << axis.x << ", " << axis.y << ", " << axis.z << std::endl;

					_joint_transforms[link->name] = out2;
					_joint_axis[link->name] = axis;

					_origin_transforms[link->name] = out;

//					collisionObj_->setTransform(_origin_transforms[link->name]);
					_collision_objects[link->name] = collisionObj_;

//					// RENDERING STUFF!
//					RenderEntry r;
//					r.bindGLBuffers(vertices, triangles, normalCnt, normals_);
//					_render_objects[link->name] = r;


					cogimon::convert(link->collision->origin, _link_frames[link->name]);
					cout << "Loading collision geometry for link (" << link->name << ") was successful! (" << resultingPath << ")" << std::endl;


//					std::cout << "TranslationOrigin:\n" << collisionObj_->getTranslation() << std::endl;
//					std::cout << "RotationOrigin:\n" << collisionObj_->getRotation() << std::endl;
				}
			}
		} else {
			std::cout << "Collision not defined for link: " << link->name << std::endl;
		}

	}

	return true;
}

//void RobotDescription::RenderEntry::render() {
//	glBindVertexArray(vao);
//	glDrawElements(GL_TRIANGLES, elementCount, GL_UNSIGNED_INT, NULL);
//	glBindVertexArray(0);
//}
//
//void RobotDescription::RenderEntry::bindGLBuffers(std::vector<fcl::Vector3d > vertices_, std::vector<fcl::Triangle> indices_, int normalCnt, float* normals_) {
//	vbo[VERTEX_BUFFER] = NULL;
//	vbo[TEXCOORD_BUFFER] = NULL;
//	vbo[NORMAL_BUFFER] = NULL;
//	vbo[INDEX_BUFFER] = NULL;
//
//	glGenVertexArrays(1, &vao);
//	glBindVertexArray(vao);
//
//	if (vertices_.size() > 0) {
//		float* vertices = new float[vertices_.size() * 3];
//		for(int i = 0; i < vertices_.size(); ++i) {
//			vertices[i * 3] = vertices_[i](0);
//			vertices[i * 3 + 1] = vertices_[i](1);
//			vertices[i * 3 + 2] = vertices_[i](2);
//		}
//
//		glGenBuffers(1, &vbo[VERTEX_BUFFER]);
//		glBindBuffer(GL_ARRAY_BUFFER, vbo[VERTEX_BUFFER]);
//		glBufferData(GL_ARRAY_BUFFER, 3 * vertices_.size() * sizeof(GLfloat), vertices, GL_STATIC_DRAW);
//
//		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
//		glEnableVertexAttribArray(0);
//	}
//
//	if (normalCnt > 0) {
//		glGenBuffers(1, &vbo[NORMAL_BUFFER]);
//		glBindBuffer(GL_ARRAY_BUFFER, vbo[NORMAL_BUFFER]);
//		glBufferData(GL_ARRAY_BUFFER, 3 * vertices_.size() * sizeof(GLfloat), normals_, GL_STATIC_DRAW);
//
//		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, NULL);
//		glEnableVertexAttribArray(2);
//	}
//
//	elementCount = indices_.size() * 3;
//	if (indices_.size() > 0) {
//		unsigned int* indices = new unsigned int[indices_.size() * 3];
//		for(int i = 0; i <indices_.size(); ++i) {
//			indices[i * 3] = indices_[i][0];
//			indices[i * 3 + 1] = indices_[i][1];
//			indices[i * 3 + 2] = indices_[i][2];
//		}
//
//		glGenBuffers(1, &vbo[INDEX_BUFFER]);
//		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[INDEX_BUFFER]);
//		glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * indices_.size() * sizeof(GLuint), indices, GL_STATIC_DRAW);
//
//		glVertexAttribPointer(3, 3, GL_UNSIGNED_INT, GL_FALSE, 0, NULL);
//		glEnableVertexAttribArray(3);
//	}
//
//	glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glBindVertexArray(0);
//}

void RobotDescription::get_tree_segment_names_recursive(const SegmentMap::const_iterator& it,
		std::vector<std::string> &link_names, bool no_fixed_joints) {

	const TreeElement& currentElement = it->second;

	if (currentElement.segment.getJoint().getType() == KDL::Joint::None
			&& no_fixed_joints) {
	} else {
		link_names.push_back(currentElement.segment.getName());
	}

	for (std::vector<SegmentMap::const_iterator>::const_iterator child_it =
			currentElement.children.begin();
			child_it != currentElement.children.end(); ++child_it) {
		get_tree_segment_names_recursive(*child_it, link_names,
				no_fixed_joints);
	}
}

void RobotDescription::get_tree_segment_names(Tree tree, std::vector<std::string> &link_names,
		bool non_fixed_joints) {
	SegmentMap::const_iterator root_it = tree.getRootSegment();
	const TreeElement& currentElement = root_it->second;

	// In order to remove world link
	if (currentElement.segment.getJoint().getType() == KDL::Joint::None) {
	} else {
		link_names.push_back(currentElement.segment.getName());
	}

	for (std::vector<SegmentMap::const_iterator>::const_iterator child_it =
			currentElement.children.begin();
			child_it != currentElement.children.end(); ++child_it) {
		get_tree_segment_names_recursive(*child_it, link_names,
				non_fixed_joints);
	}

}

/** ####### REAL-TIME CRITICAL PART (START) ####### */
bool RobotDescription::transformAllLinkCollisionObjects(const KDL::JntArray &joints_kdl) {
	fcl::Transform3d baseIteration;
	baseIteration.setIdentity();

	fcl::Transform3d fkin_transform;
	kdl2fcl(_origin, fkin_transform);
	baseIteration = baseIteration * fkin_transform;

	for (unsigned int i = 0; i < _link_names.size(); ++i) {
//			std::cout << "check! links [" << i << "] = " << _link_names[i] << std::endl;
			std::shared_ptr<fcl::CollisionObjectd> collObj = _collision_objects[_link_names[i]];
			if (collObj) {
//				std::cout << "old TranslationOrigin:\n" << collObj->getTranslation() << std::endl;
//				std::cout << "old RotationOrigin:\n" << collObj->getRotation() << std::endl << std::endl;
//
//				std::cout << "old TranslationBASE:\n" << baseIteration.translation() << std::endl;
//				std::cout << "old RotationBASE:\n" << baseIteration.rotation() << std::endl;

				double angleValue = 0.0;
				if (i > 0) {
					angleValue = joints_kdl.data[i-1];
				}

				Eigen::AngleAxis<double> axisTransform(angleValue, Eigen::Vector3d(_joint_axis[_link_names[i]].x,_joint_axis[_link_names[i]].y, _joint_axis[_link_names[i]].z));
				std::cout << "axisTransform:\n" << axisTransform.axis() << ",\n" << axisTransform.angle() << std::endl;

				baseIteration = (baseIteration * _joint_transforms[_link_names[i]]);
				baseIteration.rotate(axisTransform);

//				result.setIdentity();
//				result.translate(baseIteration.translation());
//				result.rotate(_origin_transforms[_link_names[i]].rotation());
				collObj->setTransform(baseIteration * _origin_transforms[_link_names[i]]);
//
////				baseIteration = collObj->getTransform();
//				std::cout << "new TranslationOrigin:\n" << collObj->getTranslation() << std::endl;
//				std::cout << "new RotationOrigin:\n" << collObj->getRotation() << std::endl << std::endl;
//
//				std::cout << "new TranslationBASE:\n" << baseIteration.translation() << std::endl;
//				std::cout << "new RotationBASE:\n" << baseIteration.rotation() << std::endl;
			} else {
				std::cerr << "Collision object for link " << _link_names[i] << " not found!" << std::endl;
			}
	}


//	for (auto const& coll : _collision_objects) {
//		std::string coll_link_name = coll.first;
//
//		cout << "Test JntToCart for " << coll_link_name << endl;
//
//		int error_result = robot_fk_->JntToCart(joints_kdl, _fkin_frame, coll_link_name);
//		if (error_result < 0) {
//			cerr << "Failed to calculate JntToCart for link " << coll_link_name << endl;
//			return false;
//		}
//
//		std::cout << "KDL Frame:\n" << _fkin_frame << std::endl;
//		fcl::Transform3d fkin_transform;
////		kdl2fcl(_origin * _fkin_frame, fkin_transform);
//		kdl2fcl(_fkin_frame, fkin_transform);
//
//		std::shared_ptr<fcl::CollisionObjectd> collObj = _collision_objects[coll_link_name];
//
//		std::cout << "b TranslationOrigin:\n" << collObj->getTranslation() << std::endl;
//		std::cout << "b RotationOrigin:\n" << collObj->getRotation() << std::endl;
//
//		collObj->setTransform(_origin_transforms[coll_link_name] * baseIteration);
//		baseIteration = collObj->getTransform();
//
//		std::cout << "a TranslationOrigin:\n" << collObj->getTranslation() << std::endl;
//		std::cout << "a RotationOrigin:\n" << collObj->getRotation() << std::endl;
//
//		collObj->computeAABB();
//		fcl::AABB<double> a = collObj->getAABB();
//		std::cout << "center:\n" << a.center() << "\nwidth:\n" << a.width() << "\nheight:\n" << a.height() << "\ndepth:\n" << a.depth() << std::endl;
//
//		cout << coll_link_name << " t1:\n Translation:\n" << collObj->getTranslation() << "\n Rotation:\n" << collObj->getRotation() << endl << endl;
//	}
	return true;
}
/** ####### REAL-TIME CRITICAL PART (END) ####### */

int RobotDescription::getNumberOfJointsInChain(const std::string &chain_name) {
	return _xbotcore_model.get_joint_num(chain_name);
}

std::string RobotDescription::getActiveKinematicChainName() {
	return _active_kinematic_chain_name;
}

// TODO map access can be very slow!
std::shared_ptr<fcl::CollisionObjectd> RobotDescription::getCollisionObjectForLink(const std::string &link_name) {
	return _collision_objects[link_name];
}

int RobotDescription::getNumberOfLinks() {
	return _collision_objects.size();
}

std::vector<std::string> RobotDescription::getAllLinkNames() {
	return _link_names;
}

KDL::Frame RobotDescription::getOrigin() {
	return _origin;
}

void RobotDescription::setOrigin(const KDL::Frame origin) {
	_origin = origin;
//	for (auto const& coll : _collision_objects) {
//		fcl::Transform3d fkin_transform;
//		kdl2fcl(_origin, fkin_transform);
//		coll.second->setTransform(fkin_transform);
//	}
}

