#ifndef __ASSIMP_FCL_MESH_HPP_
#define __ASSIMP_FCL_MESH_HPP_

#include <assimp/scene.h>
#include <assimp/mesh.h>

#include <vector>
#include <fcl/config.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

class Mesh
{
public :
	struct MeshEntry {

		unsigned int elementCount;
		int verticeCnt;
		int faceCnt;
		int normalCnt;
		float* vertices_;
		float* normals_;
		unsigned int* indices_;

		fcl::CollisionObject<double>* collisionObj_;

		MeshEntry(aiMesh *mesh);
		MeshEntry(float bbMinX, float bbMinY, float bbMinZ, float bbMaxX, float bbMaxY, float bbMaxZ);

		void setupCollisionObject();
		void load(aiMesh *mesh);
	};

	std::vector<MeshEntry*> meshEntries;

public:
	Mesh(float bbMinX, float bbMinY, float bbMinZ, float bbMaxX, float bbMaxY, float bbMaxZ);
	Mesh(const char *filename);
	~Mesh(void);
	bool _loadSuccess = false;
	bool isLoadSuccess() {
		return _loadSuccess;
	}
	fcl::CollisionObject<double>* getCollisionObject() {
		return meshEntries[0]->collisionObj_;
	}
};

#endif
