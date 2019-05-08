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
#include <glm/gtx/string_cast.hpp>
#include <queue>
#include <map>


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


#define G (4*9.8)
#define PI 3.1416
#define SPRING_CYLINDER_RADIUS 0.1f
#define PARTICLE_RADIUS 0.5f


struct Spring;
struct Triangle;

// Nodes in the spring.
struct Particle {
	Particle(glm::vec3 init_position, glm::vec3 curr_position, float mass, glm::vec2 uv_coords, int grid_x = -1, int grid_z = -1);
	Particle(glm::vec3 init_position, glm::vec3 curr_position, float mass, glm::vec2 uv_coords, bool is_secondary);
	Particle(const Particle& old_obj);
	~Particle();
	
	void resetForce();	// clear all forces except for gravity.
	void addForce(glm::vec3 f);
	void setFixed();
	void setMovable();
	void move(glm::vec3 dist);
	void resetNormals();
	glm::vec3 position_;
	glm::vec3 init_position_;
	glm::vec3 force_;
	glm::vec3 velocity_;
	glm::vec3 old_position_;
	glm::vec2 uv_coords_;
	glm::vec3 vertex_normal_;
	std::unordered_set<Spring*> springs_;
	std::vector<glm::vec3> face_normals_;	
	int grid_x_, grid_z_;
	float mass_;
	bool fixed_ = false;
	bool is_secondary_ = false;
	bool duplicated_ = false;

};

struct Triangle {
	Triangle(Particle* p1, Particle* p2, Particle* p3);
	Triangle();
	~Triangle();
	std::vector<Particle*> particles_;	// length == 3. Three particles.
	glm::vec3 face_normal_;
};

struct Spring {

	Spring(Particle* p1, Particle* p2, float k, bool is_secondary = false);	// k is the spring constant
	~Spring();

	void computeForceQuantity();	// compute the force quantity, and store it in force_quantity_
	void applyForce();	// compute two force vectors, and apply them to two particles connected to the spring.
	void replaceTriangle(Triangle* t_old, Triangle* t_new);
	void replaceParticle(Particle* p_old, Particle* p_new);
	void removeBendSpring();

	// std::vector<Particle*> nb_particles_;	// two particles neighboring to but not owned by the spring.
	std::vector<Triangle*> triangles_;	// a spring is neighbor to either 1 or 2 triangles.

	Particle* p1_;
	Particle* p2_;
	Spring* bend_spring_ = nullptr;	// A bending spring (if there is one) related to this structural spring.
									// If this spring itself is a bending spring, this attribute will simply be nullptr.
	
	float force_quantity_;
	float init_length_;
	float k_;
	float max_deform_rate_ = 0.3f;
	float min_length_, max_length_;
	bool is_secondary_ = false;
	bool constrained_ = false;

};

class Cloth {

public:
	Cloth(int x_size, int z_size);
	~Cloth();
	void animate(float delta_t);	// recalculate the forces, velocities and positions. Finally update cache
	Particle* getCurrentParticle() {return picked_particle_;}
	void resetCloth();
	// void bfsConstrain(std::queue<Particle*>& q);


	// The following vectors are cache for GPU rendering.
	std::vector<glm::vec3> vertices;		// for rendering the cloth
	std::vector<glm::vec2> cloth_uv_coords;	// for texture mapping the the future.
	std::vector<glm::vec3> struct_spring_vertices;	// used to linemesh springs. For debug use. 
	std::vector<glm::vec3> bend_spring_vertices;	// used to linemesh springs. For debug use. 
	std::vector<glm::vec3> vertex_normals;
	glm::vec3 pick_ray_start = glm::vec3(0.0f); 
	glm::vec3 pick_ray_end = glm::vec3(0.0f);
	bool to_tear = false;


private:
	int getParticleIdx(int x, int z);
	bool gridCoordValid(int x, int z);	
	void refreshCache();	// update the cache for rendiering
	void setInitAnchorNodes();
	void tear(Spring* s);
	void collisionWithFloor();
	Particle* getNeighborParticle(Triangle* t1, Spring* s);
	bool containsStructSpring(Particle* p1, Particle* p2);
	Spring* addStructSpring(Particle* p1, Particle* p2, float k, bool is_secondary);
	Spring* getStructSpring(Particle* p1, Particle* p2);
	void removeStructSpring(Spring* s);

	void setCurrentSpring();
	void setCurrentParticle();
	void groupNeighbors(Particle* p, std::map<int, std::unordered_set<Particle*>>& groups);
	void duplicateParticles(Particle* p, std::map<int, std::unordered_set<Particle*>>& groups, std::vector<Particle*>& new_particles);
	
	int findRoot(std::vector<int>& uf, int idx);	// a helper function for union-find algorithm

	void bfsConstrain(std::queue<Particle*>& q);

	std::vector<Particle*> particles_;
	std::unordered_set<Triangle*> triangles_;	//stored in a hashset for constant time access, modify and delete
	std::unordered_set<Spring*> springs_;		//stored in a hashset for constant time access, modify and delete
	std::map<Particle*, std::map<Particle*, Spring*>> spring_map_; // key: particle pairs. Value: springs.

	Spring* picked_spring_ = nullptr;
	Particle* picked_particle_ = nullptr;
	int x_size_, z_size_;
	float time_ = 0.0f;
	const float grid_width_ = 1.0;
	const float struct_k_ = 100.0;	// spring constant of bending springs
	const float bend_sheer_k_ = 20.0;		// spring constant of bending springs. (there bending springs also used as sheering springs)
	const float damper_ = 0.30;
	const float particle_mass_ = 0.1;	// init mass of every particle.
	const float init_height_ = 50.0;		// init height of the cloth. .e. init z position of all particles)

};



#endif
