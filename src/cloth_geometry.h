#ifndef CLOTH_GEOMETRY_H
#define CLOTH_GEOMETRY_H

#include <ostream>
#include <vector>
#include <string>

#include <map>
#include <limits>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <mmdadapter.h>
#include <unordered_set>


using namespace std;
class TextureToRender;

struct BoundingBox {
	BoundingBox()
		: min(glm::vec3(-std::numeric_limits<float>::max())),
		max(glm::vec3(std::numeric_limits<float>::max())) {}
	glm::vec3 min;
	glm::vec3 max;
};

struct Joint {
	Joint()
		: joint_index(-1),
		  parent_index(-1),
		  position(glm::vec3(0.0f)),
		  init_position(glm::vec3(0.0f))
	{
	}
	Joint(int id, glm::vec3 wcoord, int parent)
		: joint_index(id),
		  parent_index(parent),
		  position(wcoord),
		  init_position(wcoord),
		  init_rel_position(init_position)
	{
	}

	int joint_index;
	int parent_index;
	glm::mat4 transform_matrix;
	glm::vec3 position;             // position of the joint
	glm::fquat orientation;         // rotation w.r.t. initial configuration
	glm::fquat rel_orientation;     // rotation w.r.t. it's parent. Used for animation.
	glm::vec3 init_position;        // initial position of this joint
	glm::vec3 init_rel_position;    // initial relative position to its parent
	std::vector<int> children;

	
};

struct Configuration {
	std::vector<glm::vec3> trans;
	std::vector<glm::fquat> rot;

	const auto& transData() const { return trans; }
	const auto& rotData() const { return rot; }
};

struct KeyFrame {
	std::vector<glm::fquat> rel_rot;

	static void interpolate(const KeyFrame& from,
	                        const KeyFrame& to,
	                        float tau,
	                        KeyFrame& target);
	static void interpolateSQUAD(const KeyFrame& from, const KeyFrame& to, const KeyFrame& prev, const KeyFrame& after, float tau, KeyFrame& target);
	
};

struct LineMesh {
	std::vector<glm::vec4> vertices;
	std::vector<glm::uvec2> indices;
};

struct Skeleton {
	std::vector<Joint> joints;

	Configuration cache;

	void refreshCache(Configuration* cache = nullptr);
	const glm::vec3* collectJointTrans() const;
	const glm::fquat* collectJointRot() const;
	void skeletonUpdate(KeyFrame& frame) ;
	void updateChildren(Joint& parent);
	// FIXME: create skeleton and bone data structures
};

struct Mesh {
	Mesh();
	~Mesh();
	std::vector<glm::vec4> vertices;
	/*
	 * Static per-vertex attrributes for Shaders
	 */
	std::vector<int32_t> joint0;
	std::vector<int32_t> joint1;
	std::vector<float> weight_for_joint0; // weight_for_joint1 can be calculated
	std::vector<glm::vec3> vector_from_joint0;
	std::vector<glm::vec3> vector_from_joint1;
	std::vector<glm::vec4> vertex_normals;
	std::vector<glm::vec4> face_normals;
	std::vector<glm::vec2> uv_coordinates;
	std::vector<glm::uvec3> faces;
	std::vector<KeyFrame> keyframes;
	std::vector<Material> materials;
	BoundingBox bounds;
	Skeleton skeleton;
	std::vector<TextureToRender*> textures;
	int refreshFrame = -1;
	bool preLoaded = false;
	bool squadIt = false;
	int deleteFrame = -1;
	void loadPmd(const std::string& fn);
	int getNumberOfBones() const;
	glm::vec3 getCenter() const { return 0.5f * glm::vec3(bounds.min + bounds.max); }
	const Configuration* getCurrentQ() const; // Configuration is abbreviated as Q
	void updateAnimation(float t = -1.0);
	glm::mat4 getBoneTransformMatrix(int jointID);
	Joint getJoint(int joint_ID);
	glm::fquat vectorQuat(glm::vec3 start, glm::vec3 dest);
	void rotate_bone(int bone_index, glm::fquat& rotate_quat, bool child);
	glm::vec3 getJointPos(int joint_index);
	void addKeyframe();
	void setFirstFrame();
	void deleteThisFrame(int current_frame);
	void overwriteThisFrame(int current_frame);
	void saveAnimationTo(const std::string& fn);
	void changeCurrentSkeleton(int current_frame);
	void loadAnimationFrom(const std::string& fn);
	

private:
	void computeBounds();
	void computeNormals();
	Configuration currentQ_;
};



#define G 9.8f
#define PI 3.1416f

struct SpringNode {
public:
	SpringNode(int index, glm::vec3 init_pos, glm::vec3 curr_pos, float mass, bool init_fixed = false);
	~SpringNode();

	glm::vec3 position;
	glm::vec3 init_position;
	glm::vec3 velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 force;

	std::vector<SpringNode*> neighbors;
	std::vector<float> init_nb_dists;

	float mass;
	bool fixed;
	bool teared = false;
	int index;


};


// struct AnchorNode {
// public:
// 	AnchorNode();
// 	~AnchorNode();
// 	SpringNode* node;
// 	glm::vec3 position;	// anchor position
// };

class MassSpringSystem {
public:
	MassSpringSystem(int x_size, int z_size);
	~MassSpringSystem();

	glm::vec3 computeSingleForce(const SpringNode* curr_node, const SpringNode* nb_node);

	void setNodeFixed(int idx);
	void setNodeMovable(int idx);

	void refreshCache();	// copy node positions to opengl buffer
	const glm::vec3* collectNodePositions();

	void animate(float delta_t);
	void checkTear(SpringNode* curr_node, int nb_idx, float max_deform_rate, std::vector<SpringNode*>& teared_new_nodes);

	void resetSystem();
	void randomDisturb();
	float getPeriod() {return T_;}

	std::vector<glm::vec3> node_positions;
	std::vector<glm::uvec2> line_indices;	// indices for line mesh
	std::vector<SpringNode*> nodes_;


private:
	bool isIndexValid(int x, int z);
	int getNodeIndex(int x, int z);
	const float node_mass_ = 10.0;

	const float spring_k_ = 100.0;

	float T_ = 2 * PI * std::sqrt(node_mass_ / spring_k_);

	// const float energy_loss_ = 0.95;
	const float damper_ = (0.05) * (2 * std::sqrt(node_mass_ * spring_k_));
	const float grid_width_ = 1.0;
	int x_size, z_size;
	



};


struct Spring;
struct Triangle;


struct Particle {
	Particle(glm::vec3 init_position, float mass, glm::vec2 uv_coords, int grid_x, int grid_z);
	~Particle();
	
	void resetForce();
	void addForce(glm::vec3 f);

	void setFixed();
	void setMovable();

	glm::vec3 position_;
	glm::vec3 init_position_;
	glm::vec3 force_;
	glm::vec3 velocity_;

	glm::vec2 uv_coords_;
	glm::vec2 grid_coords;

	int grid_x_, grid_z_;

	std::vector<Spring*> springs_;
	float mass_;
	bool fixed_;

};

struct Triangle {
	// std::vector<Spring*> springs;
	std::vector<Particle*> particles_;

};

struct Spring {
	Spring(Particle* p1, Particle* p2, float k);
	~Spring();

	void computeForceQuantity();
	void applyForce();

	std::vector<Particle*> particles_;	// two particles
	std::vector<Triangle*> triangles_;	// two triangles

	Particle* p1_;
	Particle* p2_;
	
	float force_quantity_;

	Spring* bend_spring_ = nullptr;
	float init_length_;
	float k_;
};

class Cloth {

public:
	Cloth(int x_size, int z_size);
	~Cloth();
	void animate(float delta_t);




	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> cloth_uv_coords;
	// std::vector<glm::uvec3> faces;

	std::vector<glm::vec3> spring_vertices;	// used to linemesh springs. For debug use. 
	std::vector<glm::vec3> bend_spring_vertices;

private:
	int getParticleIdx(int x, int z);
	bool gridCoordValid(int x, int z);
	void refreshCache();



	std::vector<Particle*> particles_;
	std::unordered_set<Triangle*> triangles_;
	std::unordered_set<Spring*> springs_;
	// std::vector<Triangle*> triangles_;
	// std::vector<Spring*> springs_;

	int x_size_, z_size_;

	const float grid_width_ = 10.0;

	const float struct_k_ = 1.0;
	const float bend_k_ = 1.0;
	
	const float damper_ = 0.1;


	const float bend_sheer_k = 1.0;
	const float particle_mass_ = 0.1;
	const float init_height_ = 0.0;






};


#endif
