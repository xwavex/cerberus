#include "Mesh.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <iostream>
#include <memory>

#include <fcl/common/types.h>
#include <fcl/fcl.h>


/**
*	Constructor, loading the specified aiMesh
**/
Mesh::MeshEntry::MeshEntry(aiMesh *mesh) {
	elementCount = mesh->mNumFaces * 3;
	verticeCnt = mesh->mNumVertices;
	faceCnt = mesh->mNumFaces;

	if(mesh->HasPositions()) {
		vertices_ = new float[mesh->mNumVertices * 3];
		for(int i = 0; i < mesh->mNumVertices; ++i) {
			vertices_[i * 3] = mesh->mVertices[i].x;
			vertices_[i * 3 + 1] = mesh->mVertices[i].y;
			vertices_[i * 3 + 2] = mesh->mVertices[i].z;
		}

		std::cout << "Vertices loaded!" << std::endl;
	}

	if(mesh->HasNormals()) {
		normalCnt = mesh->mNumVertices * 3;
		normals_ = new float[mesh->mNumVertices * 3];
		for(int i = 0; i < mesh->mNumVertices; ++i) {
			normals_[i * 3] = mesh->mNormals[i].x;
			normals_[i * 3 + 1] = mesh->mNormals[i].y;
			normals_[i * 3 + 2] = mesh->mNormals[i].z;
		}
		std::cout << "Normals loaded!" << std::endl;
	}


	if(mesh->HasFaces()) {
		indices_ = new unsigned int[mesh->mNumFaces * 3];
		for(int i = 0; i < mesh->mNumFaces; ++i) {
			indices_[i * 3] = mesh->mFaces[i].mIndices[0];
			indices_[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
			indices_[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
		}

		std::cout << "Faces loaded!" << std::endl;
	}

}


Mesh::MeshEntry::MeshEntry(float bbMinX, float bbMinY, float bbMinZ, float bbMaxX, float bbMaxY, float bbMaxZ) {
	elementCount = 12 * 3;
	verticeCnt = 36;
	faceCnt = 12;
	normalCnt = 36;

	/*vertices_ = new float[3*8]{
		bbMinX, bbMinY, bbMinZ, // (1)
		bbMaxX, bbMinY, bbMinZ, // (2)
		bbMinX, bbMinY, bbMaxZ, // (3)
		bbMaxX, bbMinY, bbMaxZ, // (4)
		bbMinX, bbMaxY, bbMinZ, // (5)
		bbMaxX, bbMaxY, bbMinZ, // (6)
		bbMinX, bbMaxY, bbMaxZ, // (7)
		bbMaxX, bbMaxY, bbMaxZ  // (8)
	};*/
	vertices_ = new float[3 * 36]{
		bbMaxX, bbMaxY, bbMinZ,
		bbMaxX, bbMinY, bbMinZ,
		bbMinX, bbMinY, bbMinZ,
		bbMinX,  bbMinY,bbMinZ,
		bbMinX, bbMaxY, bbMinZ,
		bbMaxX, bbMaxY, bbMinZ,
		bbMaxX, bbMaxY, bbMaxZ,
		bbMinX, bbMaxY, bbMaxZ,
		bbMinX, bbMinY,bbMaxZ,
		bbMinX, bbMinY, bbMaxZ,
		bbMaxX, bbMinY,bbMaxZ,
		bbMaxX, bbMaxY, bbMaxZ,
		bbMaxX, bbMaxY, bbMinZ,
		bbMaxX, bbMaxY, bbMaxZ,
		bbMaxX, bbMinY,bbMaxZ,
		bbMaxX , bbMinY,bbMaxZ,
		bbMaxX , bbMinY, bbMinZ,
		bbMaxX, bbMaxY, bbMinZ,
		bbMaxX , bbMinY, bbMinZ,
		bbMaxX, bbMinY,bbMaxZ,
		bbMinX, bbMinY,bbMaxZ,
		bbMinX, bbMinY,bbMaxZ,
		bbMinX, bbMinY,bbMinZ,
		bbMaxX, bbMinY, bbMinZ,
		bbMinX, bbMinY,bbMinZ,
		bbMinX, bbMinY,bbMaxZ,
		bbMinX, bbMaxY, bbMaxZ,
		bbMinX, bbMaxY, bbMaxZ,
		bbMinX, bbMaxY, bbMinZ,
		bbMinX, bbMinY,bbMinZ,
		bbMaxX, bbMaxY, bbMaxZ,
		bbMaxX, bbMaxY, bbMinZ,
		bbMinX, bbMaxY, bbMinZ,
		bbMinX, bbMaxY, bbMinZ,
		bbMinX, bbMaxY, bbMaxZ,
		bbMaxX, bbMaxY, bbMaxZ,
	};
	std::cout << "Vertices loaded!" << std::endl;

	normals_ = new float[36 * 3]{
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0
	};

	indices_ = new unsigned int[3 * 12]{
		0, 1, 2,
		3,  4, 5,
		6,  7, 8,
		9,  10,  11,
		12,  13,  14,
		15,  16 , 17,
		18,  19,  20,
		21,  22,  23,
		24,  25,  26,
		27,  28,  29,
		30,  31,  32,
		33,  34, 35
	};
	std::cout << "Faces loaded!" << std::endl;

}

void Mesh::MeshEntry::setupCollisionObject() {
	// set mesh triangles and vertices indices
	std::vector<fcl::Vector3d> vertices(verticeCnt);
	std::vector<fcl::Triangle> triangles(faceCnt);

	for (int i = 0; i < verticeCnt; i++) {
		vertices[i] = fcl::Vector3d(vertices_[i * 3], vertices_[i * 3 + 1], vertices_[i * 3 + 2]);
	}
	for (int i = 0; i < faceCnt; i++) {
		triangles[i] = fcl::Triangle(indices_[i * 3], indices_[i * 3 + 1], indices_[i * 3 + 2]);
	}

	// BVHModel is a template class for mesh geometry, for default OBBRSS template is used
	fcl::Quaternion<double> Rotation;
	fcl::Vector3d Translate;

	std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> model = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

	// add the mesh data into the BVHModel structure
	model->beginModel();
	model->addSubModel(vertices, triangles);
	model->endModel();
	model->computeLocalAABB();

	collisionObj_ = new fcl::CollisionObject<double>(model);
}

/**
*	Mesh constructor, loads the specified filename if supported by Assimp
**/
Mesh::Mesh(const char *filename)
{
	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(filename, 0);
	if(!scene) {
		std::cout << "Unable to load mesh: " << importer.GetErrorString() << std::endl;
		return;
	}

	for(int i = 0; i < scene->mNumMeshes; ++i) {
		meshEntries.push_back(new Mesh::MeshEntry(scene->mMeshes[i]));
	}

	for (Mesh::MeshEntry* entry : meshEntries) {
		entry->setupCollisionObject();
	}
	_loadSuccess = true;
}

/**
* Constructs a bounding box mesh from the given bounding box
*/
Mesh::Mesh(float bbMinX, float bbMinY, float bbMinZ, float bbMaxX, float bbMaxY, float bbMaxZ) {
	meshEntries.push_back(new Mesh::MeshEntry(bbMinX, bbMinY, bbMinZ, bbMaxX, bbMaxY, bbMaxZ));

	for (Mesh::MeshEntry* entry : meshEntries) {
		entry->setupCollisionObject();
	}
}
/**
*	Clears all loaded MeshEntries
**/
Mesh::~Mesh(void)
{
	for(int i = 0; i < meshEntries.size(); ++i) {
		delete meshEntries.at(i);
	}
	meshEntries.clear();
}

