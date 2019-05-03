#include "config.h"
#include "cloth_geometry.h"
#include "texture_to_render.h"
#include <fstream>
#include <queue>
#include <iostream>
#include <stdlib.h>     
#include <time.h>       
#include <glm/gtx/string_cast.hpp>
#include <stdexcept>
#include <glm/gtx/io.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <map>

/*
 * For debugging purpose.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
	size_t count = std::min(v.size(), static_cast<size_t>(10));
	for (size_t i = 0; i < count; ++i) os << i << " " << v[i] << "\n";
	os << "size = " << v.size() << "\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const BoundingBox& bounds)
{
	os << "min = " << bounds.min << " max = " << bounds.max;
	return os;
}



const glm::vec3* Skeleton::collectJointTrans() const
{
	return cache.trans.data();
}

const glm::fquat* Skeleton::collectJointRot() const
{
	return cache.rot.data();
}

// FIXME: Implement bone animation.

void Skeleton::refreshCache(Configuration* target)
{
	if (target == nullptr)
		target = &cache;
	target->rot.resize(joints.size());
	target->trans.resize(joints.size());
	for (size_t i = 0; i < joints.size(); i++) {
		target->rot[i] = joints[i].orientation;
		// printf("%f %f %f%f the refresh cache boys are back in town\n", joints[i].orientation.x, joints[i].orientation.y, joints[i].orientation.z, joints[i].orientation.w);
		target->trans[i] = joints[i].position;
	}
}


Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

glm::fquat Mesh::vectorQuat(glm::vec3 start, glm::vec3 dest){
	start = glm::normalize(start);
	dest = glm::normalize(dest);

	float cosTheta = dot(start, dest);
	glm::vec3 rotationAxis;

	if (cosTheta < -1 + 0.001f){
		// special case when vectors in opposite directions:
		// there is no "ideal" rotation axis
		// So guess one; any will do as long as it's perpendicular to start
		rotationAxis = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), start);
		if (glm::length2(rotationAxis) < 0.01 ) {// bad luck, they were parallel, try again! 
			rotationAxis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), start);
		}

		rotationAxis = glm::normalize(rotationAxis);
		return glm::angleAxis(glm::radians(180.0f), rotationAxis);
	}

	rotationAxis = glm::cross(start, dest);

	float s = sqrt( (1+cosTheta)*2 ) + 0.000001;
	float invs = 1 / s;

	return glm::normalize(glm::fquat(
		s * 0.5f, 
		rotationAxis.x * invs,
		rotationAxis.y * invs,
		rotationAxis.z * invs
	));

}


void Mesh::loadPmd(const std::string& fn)
{
	MMDReader mr;
	mr.open(fn);
	mr.getMesh(vertices, faces, vertex_normals, uv_coordinates);
	computeBounds();
	mr.getMaterial(materials);
	
	int jointID = 0;
	int parentID;
	glm::vec3 wcoord;
	// printf("%d\n ", parentID);
	while(mr.getJoint(jointID, wcoord, parentID)) {

			// printf("after %d\n ", parentID);
			Joint *curr = new Joint(jointID, wcoord, parentID);
			skeleton.joints.push_back(*curr);
			jointID++;
			// std::cout << "joint " << jointID << ": (" << wcoord.x << ", " << wcoord.y << ", " << wcoord.z << ")" << std::endl;
			// std::cout << "pos " << jointID << ": (" << curr->init_position.x << ", " << curr->init_position.y << ", " << curr->init_position.z << ")" << std::endl;
			// std::cout << "rel pos " << jointID << ": (" << curr->init_rel_position.x << ", " << curr->init_rel_position.y << ", " << curr->init_rel_position.z << ")" << std::endl;
		}
		// printf("%d OOF OUCH OWIE \n", getNumberOfBones());

	for(int x = 0; x < skeleton.joints.size(); x++){
		Joint& lig = skeleton.joints[x];
		lig.orientation = glm::fquat(1.0, 0.0, 0.0,0.0);
		lig.rel_orientation = glm::fquat(1.0, 0.0, 0.0,0.0);
		if(lig.parent_index == -1){
			
		} else {
		
			Joint& parent_joint = skeleton.joints[lig.parent_index];
			// if(curr_joint.parent_index == -1){
			// 	printf("boom");
			// }
			float length = glm::length(lig.position - parent_joint.position);
			glm::mat4 scale = glm::mat4(1.0f);	
			scale[1][1] = length;

			// glm::fquat q;
			// glm::vec3 a = glm::cross(glm::vec3(0.0, 1.0, 0.0), curr_joint.position - parent_joint.position);
			// q.x = a.x;
			// q.y = a.y;
			// q.z = a.z;
			// q.w = sqrt(glm::length2(glm::vec3(0.0, 1.0, 0.0)) * glm::length2(curr_joint.position - parent_joint.position)) + glm::dot(glm::vec3(0.0, 1.0, 0.0), curr_joint.position - parent_joint.position);
			glm::fquat rotate_quat = Mesh().vectorQuat(glm::vec3(0.0, 1.0, 0.0), lig.position - parent_joint.position);
			// glm::vec4 orient = glm::vec4(parent_joint.position - curr_joint.position, 1.0f);
			// glm::fquat orieen = glm::fquat(orient);
			glm::mat4 rotate_matrix = glm::toMat4(rotate_quat);

			
			glm::mat4 translate = glm::mat4(1.0f);
			translate[3] = glm::vec4(parent_joint.position, 1.0f);
			// printf("%f %f %f \n", parent_joint.position[0], parent_joint.position[1],parent_joint.position[2]);
			// printf("%f %f %f %f\n", translate_matrix[3][0], translate_matrix[3][1],translate_matrix[3][2],translate_matrix[3][3]);

			
			
			skeleton.joints[lig.parent_index].children.push_back(lig.joint_index);
			lig.transform_matrix = translate * rotate_matrix * scale;
		}
	}

	std::vector<SparseTuple> tuples;
	mr.getJointWeights(tuples);

	
	// for(SparseTuple& tuple : tuples) {
		
	// 	int vid = tuple.vid;
		
	// 	weight_for_joint0.push_back(tuple.weight0);
	// 	joint0.push_back(tuple.jid0);
	// 	vector_from_joint0.push_back(glm::vec3(vertices[vid]) - skeleton.joints[tuple.jid0].position);
		
	// 	if(tuple.jid1 == -1) {
	// 		// printf("needed\n");

	// 		// printf("%d here is other joint and its parent %d\n", tuple.jid0, skeleton.joints[tuple.jid0].parent_index);
	// 		joint1.push_back(0);	
	// 		vector_from_joint1.push_back(glm::vec3(0.0, 0.0, 0.0));
	// 	}
	// 	else {
	// 		joint1.push_back(tuple.jid1);
	// 		vector_from_joint1.push_back(glm::vec3(vertices[vid]) - skeleton.joints[tuple.jid1].position);
	// 	}

		for (int i = 0; i < tuples.size(); i++) {
		SparseTuple tuple = tuples[i];
		int jid0 = tuple.jid0;
		int jid1 = tuple.jid1;
		int vid = tuple.vid;
		// Joint joint = skeleton.joints[jid0];
		
		float weight = tuple.weight0;
		weight_for_joint0.push_back(weight);

		// for (int b = 0; b < joint.outBones.size(); b++) {
		// 	int bid = joint.outBones[b];
		// 	dok[vid][bid] = weight;
		// }
		if(jid0 == -1 && jid1 == -1){
			joint0.push_back(0);	
			vector_from_joint0.push_back(glm::vec3(0.0, 0.0, 0.0));
			joint1.push_back(0);	
			vector_from_joint1.push_back(glm::vec3(0.0, 0.0, 0.0));
		} else if (jid1 == -1){
			joint0.push_back(tuple.jid0);
			vector_from_joint0.push_back(glm::vec3(vertices[vid]) - skeleton.joints[tuple.jid0].position);
			joint1.push_back(0);	
			vector_from_joint1.push_back(glm::vec3(0.0, 0.0, 0.0));
		} else {
			joint0.push_back(tuple.jid0);
			vector_from_joint0.push_back(glm::vec3(vertices[vid]) - skeleton.joints[tuple.jid0].position);
			joint1.push_back(tuple.jid1);
			vector_from_joint1.push_back(glm::vec3(vertices[vid]) - skeleton.joints[tuple.jid1].position);
		}
//}
	}

	updateAnimation();
	// FIXME: load skeleton and blend weights from PMD file,
	//        initialize std::vectors for the vertex attributes,
	//        also initialize the skeleton as needed
}



void Mesh::rotate_bone(int bone_index,  glm::fquat& rotate_quat, bool child) {
	// printf("bone ind %d\n", bone_index);
	// Joint& curr_joint = skeleton.joints[bone_index];
	// printf("why\n");
	Joint curr_joint = skeleton.joints[bone_index];
	
	
	if(!child){
		skeleton.joints[bone_index].rel_orientation = glm::normalize(rotate_quat * curr_joint.rel_orientation);
	}
	skeleton.joints[bone_index].orientation = glm::normalize(rotate_quat * curr_joint.orientation);
	if(child){
		Joint parent_joint = skeleton.joints[curr_joint.parent_index];
		float length = glm::length(curr_joint.position - parent_joint.position);
		glm::mat4 scale = glm::mat4(1.0f);	
		scale[1][1] = length;
		skeleton.joints[bone_index].position = parent_joint.position + parent_joint.orientation * (curr_joint.init_position - parent_joint.init_position);
		glm::fquat rotate_quat = Mesh().vectorQuat(glm::vec3(0.0, 1.0, 0.0), curr_joint.position - parent_joint.position);
			// glm::vec4 orient = glm::vec4(parent_joint.position - curr_joint.position, 1.0f);
			// glm::fquat orieen = glm::fquat(orient);
		glm::mat4 rotate_matrix = glm::toMat4(rotate_quat);

			
		glm::mat4 translate = glm::mat4(1.0f);
		translate[3] = glm::vec4(parent_joint.position, 1.0f);
		curr_joint.transform_matrix =  translate * rotate_matrix * scale;
		skeleton.joints[bone_index].transform_matrix = translate * rotate_matrix * scale;
	}
	for(int child_index : curr_joint.children) {
		rotate_bone(child_index, rotate_quat, true);
	}
}





int Mesh::getNumberOfBones() const
{
	return skeleton.joints.size();
}

void Mesh::computeBounds()
{
	bounds.min = glm::vec3(std::numeric_limits<float>::max());
	bounds.max = glm::vec3(-std::numeric_limits<float>::max());
	for (const auto& vert : vertices) {
		bounds.min = glm::min(glm::vec3(vert), bounds.min);
		bounds.max = glm::max(glm::vec3(vert), bounds.max);
	}
}

void interpolate(const KeyFrame& from, const KeyFrame& to, float tau, KeyFrame& mixed){
	for(int i = 0; i < from.rel_rot.size(); i++) {
		glm::fquat kf_rot = glm::mix(from.rel_rot[i], to.rel_rot[i], tau);
		mixed.rel_rot.push_back(kf_rot);
}
}


void interpolateSQUAD(const KeyFrame& from, const KeyFrame& to, const KeyFrame& prev, const KeyFrame& after, float tau, KeyFrame& mixed){

	// printf("%f tau tau tau tau\n", tau);
	// printf("who is from %d\n", )
	float clampedT = glm::fract(tau);
	
	// printf("who is 4chan %f\n", clampedT);
	for(int i = 0; i < from.rel_rot.size(); i++) {
		glm::fquat kf_rot = glm::fquat();
		if( from.rel_rot[i].x  == to.rel_rot[i].x&& from.rel_rot[i].y == to.rel_rot[i].y && from.rel_rot[i].z == to.rel_rot[i].z && from.rel_rot[i].w == to.rel_rot[i].w){
			kf_rot = to.rel_rot[i];
		} else{
			glm::fquat s1 = from.rel_rot[i] * glm::exp(-0.25f * (glm::log(glm::inverse(from.rel_rot[i]) * to.rel_rot[i]) + glm::log(glm::inverse(from.rel_rot[i]) * prev.rel_rot[i])));
		glm::fquat s2 = to.rel_rot[i] * glm::exp(-0.25f * (glm::log(glm::inverse(to.rel_rot[i]) * after.rel_rot[i]) + glm::log(glm::inverse(to.rel_rot[i]) * from.rel_rot[i])));
		// if(i == 18){
		// 	printf("s1 is %f %f %f %f\n", s1.x, s1.y, s1.z, s1.w);
		// 	printf("s2 is %f %f %f %f\n", s2.x, s2.y, s2.z, s2.w);
		// }
		
		 kf_rot = glm::squad(from.rel_rot[i], to.rel_rot[i], s1, s2, clampedT);//glm::mix(from.rel_rot[i], to.rel_rot[i], tau);
		if(i == -1){
			printf("from %f %f %f %f\n", from.rel_rot[i].x, from.rel_rot[i].y, from.rel_rot[i].z, from.rel_rot[i].w);
		printf("to %f %f %f %f\n", to.rel_rot[i].x, to.rel_rot[i].y, to.rel_rot[i].z, to.rel_rot[i].w);
			printf("s1 is %f %f %f %f\n", s1.x, s1.y, s1.z, s1.w);
			printf("s2 is %f %f %f %f\n", s2.x, s2.y, s2.z, s2.w);
		printf("rot is %f %f %f %f\n", kf_rot.x, kf_rot.y, kf_rot.z, kf_rot.w);
		 }
		}
		// if(i == 18){
		// 	printf("from %f %f %f %f\n", from.rel_rot[i].x, from.rel_rot[i].y, from.rel_rot[i].z, from.rel_rot[i].w);
		// printf("to %f %f %f %f\n", to.rel_rot[i].x, to.rel_rot[i].y, to.rel_rot[i].z, to.rel_rot[i].w);
		// 	printf("s1 is %f %f %f %f\n", s1.x, s1.y, s1.z, s1.w);
		// 	printf("s2 is %f %f %f %f\n", s2.x, s2.y, s2.z, s2.w);
		// printf("rot is %f %f %f %f\n", kf_rot.x, kf_rot.y, kf_rot.z, kf_rot.w);
		//  }
		
		mixed.rel_rot.push_back(kf_rot);
}
}

void Skeleton::skeletonUpdate(KeyFrame& frame) {
	for(int i = 0; i < joints.size(); i++) {
		if(i == 18){
			// printf("orient here is what %f %f %f %f\n", frame.rel_rot[i].x, frame.rel_rot[i].y, frame.rel_rot[i].z, frame.rel_rot[i].w );
		// 	printf("pos here is what %f %f %f \n", child_joint.position.x, child_joint.position.y, child_joint.position.z );
		}
		if(joints[i].parent_index == -1) {
			joints[i].orientation = frame.rel_rot[i];
		}
		joints[i].rel_orientation = frame.rel_rot[i];
	}
	for(int i = 0; i < joints.size(); i++) {
		
			updateChildren(joints[i]);
		
	}
}


void Mesh::changeCurrentSkeleton(int current_frame){
	if(current_frame != -1){
		// printf("hello %d \n", current_frame);
		skeleton.skeletonUpdate(keyframes[current_frame]);
	}
	skeleton.refreshCache(&currentQ_);
}

void Skeleton::updateChildren(Joint& parent){
	for(int child_index : parent.children) {
		
		// if(child_index == 19){
		// 	printf("PARENT orient here is what %f %f %f \n", parent.orientation.x, parent.orientation.y, parent.orientation.z );
		// 	printf("orient here is what %f %f %f \n", child_joint.orientation.x, child_joint.orientation.y, child_joint.orientation.z );
		// 	printf("pos here is what %f %f %f \n", child_joint.position.x, child_joint.position.y, child_joint.position.z );
		// }
		Joint& child_joint = joints[child_index];
		
		
		child_joint.position = parent.position + parent.orientation * (child_joint.init_position - parent.init_position);
		
			// glm::vec4 orient = glm::vec4(parent_joint.position - curr_joint.position, 1.0f);
			// glm::fquat orieen = glm::fquat(orient);
		float length = glm::length(child_joint.position - parent.position);
		glm::mat4 scale = glm::mat4(1.0f);	
		scale[1][1] = length;
		glm::fquat rotate_quat = Mesh().vectorQuat(glm::vec3(0.0, 1.0, 0.0), child_joint.position - parent.position);
		glm::mat4 rotate_matrix = glm::toMat4(rotate_quat);

			
		glm::mat4 translate = glm::mat4(1.0f);
		translate[3] = glm::vec4(parent.position, 1.0f);
		child_joint.transform_matrix =  translate * rotate_matrix * scale;
		joints[child_index].transform_matrix = translate * rotate_matrix * scale;
		child_joint.orientation = child_joint.rel_orientation * parent.orientation;
		updateChildren(child_joint);
}
}

void Mesh::updateAnimation(float t)
{
	int frame_index = floor(t);

	if(frame_index != -1 && frame_index + 1 < keyframes.size()) {

		
		KeyFrame frame;
		// printf("from is now %d\n", frame_index);
		// interpolateSQUAD(keyframes[frame_index], keyframes[frame_index + 1], 
		// 	keyframes[(int)glm::clamp<int>(frame_index - 1, 0, keyframes.size() - 1)], 
		// 	keyframes[(int)glm::clamp<int>(frame_index + 2, 0, keyframes.size() - 1)], t, frame);
		frame.rel_rot.clear();
		float tau = t - frame_index;
		interpolate(keyframes[frame_index], keyframes[frame_index + 1], tau, frame);
// 		printf(" PREV VAL IS%d\n", (int)glm::clamp<int>(frame_index - 1, 0, keyframes.size() - 1));
// printf(" CUR VAL IS%d\n", frame_index);
// printf(" NEXT VAL IS%d\n", frame_index + 1);
// printf("NEXT NEXT VAL IS%d\n", (int)glm::clamp<int>(frame_index + 2, 0, keyframes.size() - 1));
		if(squadIt){
			interpolateSQUAD(keyframes[frame_index], keyframes[frame_index + 1], 
			keyframes[(int)glm::clamp<int>(frame_index - 1, 0, keyframes.size() - 1)], 
			keyframes[(int)glm::clamp<int>(frame_index + 2, 0, keyframes.size() - 1)], t, frame);
		}
		
		skeleton.skeletonUpdate(frame);
	}
	skeleton.refreshCache(&currentQ_);
	
}

glm::mat4 Mesh::getBoneTransformMatrix(int jointID){
	

	return skeleton.joints[jointID].transform_matrix;
}

Joint Mesh::getJoint(int joint_index)
{
	return skeleton.joints[joint_index];
}

glm::vec3 Mesh::getJointPos(int joint_index)
{
	return skeleton.joints[joint_index].position;
}

void Mesh::addKeyframe(){
	KeyFrame frame;
	// std::vector<glm::fquat> frame_set;
	for(int i = 0; i < getNumberOfBones(); i++) {
		frame.rel_rot.push_back(skeleton.joints[i].rel_orientation);
	}
	// printf("bahall\n");
	keyframes.push_back(frame);
}

void Mesh::overwriteThisFrame(int current_frame){
	if(current_frame != -1){
		KeyFrame& curr = keyframes[current_frame];
		for(int k = 0; k < curr.rel_rot.size(); k++){
			curr.rel_rot[k] = skeleton.joints[k].rel_orientation;
		}
	}
	refreshFrame = current_frame;
}

void Mesh::deleteThisFrame(int current_frame){
	KeyFrame& curr = keyframes[current_frame];
	keyframes.erase(keyframes.begin() + current_frame);
	TextureToRender* curT = textures[current_frame];
	textures.erase(textures.begin() + current_frame);
	delete curT;
	// delete curr;
	
}

void Mesh::setFirstFrame(){
	skeleton.skeletonUpdate(keyframes[0]);
}

const Configuration*
Mesh::getCurrentQ() const
{
	return &currentQ_;
}

SpringNode::SpringNode(int init_index, glm::vec3 init_pos, glm::vec3 curr_pos, float init_mass, bool init_fixed)
				:index(init_index), init_position(init_pos), position(curr_pos), mass(init_mass), fixed(init_fixed)
{
	force = glm::vec3(0.0, -mass * G, 0.0);
}

SpringNode::~SpringNode()
{

}

MassSpringSystem::MassSpringSystem(int init_x_size, int init_z_size)
									:x_size(init_x_size), z_size(init_z_size)
{
	// create nodes
	for(int x = 0; x < x_size; x++) {
		for(int z = 0; z < z_size; z++) {
			glm::vec3 node_pos(x * grid_width_, 10.0, z * grid_width_);
			SpringNode* curr_node = new SpringNode(getNodeIndex(x, z), node_pos, node_pos, node_mass_, false);
			nodes_.push_back(curr_node);
		}
	}

	// connect nodes.
	for(int x = 0; x < x_size; x++) {
		for(int z = 0; z < z_size; z++) {
			SpringNode* curr_node = nodes_[getNodeIndex(x, z)];
			for(int delta_x = -1; delta_x <= 1; delta_x++) {
				for(int delta_z = -1; delta_z <= 1; delta_z++) {
					if(!isIndexValid(x + delta_x, z + delta_z) || delta_x == 0 && delta_z == 0) {
						continue;
					}
					curr_node->neighbors.push_back(nodes_[getNodeIndex(x + delta_x, z + delta_z)]);
					line_indices.push_back(glm::uvec2(getNodeIndex(x, z), getNodeIndex(x + delta_x, z + delta_z)));
				}
			}
		}
	}

	// set border fixed
	for(int x = 0; x < x_size; x++) {
		for(int z = 0; z < z_size; z++) {
			SpringNode* curr_node = nodes_[getNodeIndex(x, z)];
			// if(x == 0) {
			if(x == 0 || x == x_size - 1 || z == 0 || z == z_size - 1) {
				nodes_[getNodeIndex(x, z)]->fixed = true;
			}
			std::cout << "x = " << x << ", z = " << z << ", fixed? " << nodes_[getNodeIndex(x, z)]->fixed << std::endl;
		}
	}


	std::cout << "system created, T = " << T_ << std::endl;
	

	srand (time(NULL));
	
	refreshCache();

}

MassSpringSystem::~MassSpringSystem() 
{
	for(SpringNode* node : nodes_) {
		delete node;
	}

}

bool MassSpringSystem::isIndexValid(int x, int z) {
	return x >= 0 && x < x_size && z >= 0 && z < z_size;
}

int MassSpringSystem::getNodeIndex(int x, int z) {
	return x * z_size + z;
}



glm::vec3 MassSpringSystem::computeSingleForce(const SpringNode* curr_node, const SpringNode* nb_node) {
	float dist = glm::length(curr_node->position - nb_node->position);
	float init_dist = glm::length(curr_node->init_position - nb_node->init_position);
	glm::vec3 force = spring_k_ * (dist - init_dist) * glm::normalize(nb_node->position - curr_node->position);
	return force;
}



void MassSpringSystem::refreshCache() {
	std::cout << "to refresh cache. current nodes size: " << nodes_.size()  << std::endl;
	node_positions.resize(nodes_.size());
	line_indices.clear();

	// std::cout << "refresh cache" << std::endl;
	for(int i = 0; i < nodes_.size(); i++) {
		node_positions[i] = nodes_[i]->position;
		for(SpringNode* nb_node : nodes_[i]->neighbors) {
			// std::cout << "line index: (" << nodes_[i]->index << ", " << nb_node->index << ")" << std::endl;
			line_indices.push_back(glm::uvec2(nodes_[i]->index, nb_node->index));
		}
	}


}

const glm::vec3* MassSpringSystem::collectNodePositions() {
	return node_positions.data();
}

void MassSpringSystem::animate(float delta_t) {	// update system states and refresh cache.
	// update force
	for(int i = 0; i < nodes_.size(); i++) {
		SpringNode* curr_node = nodes_[i];
		if(curr_node->fixed) continue;	// anchor node, not movable
		curr_node->force = glm::vec3(0.0, -curr_node->mass * G, 0.0);
		for(int j = 0; j < curr_node->neighbors.size(); j++) {
			SpringNode* nb_node = curr_node->neighbors[j];
			curr_node->force += computeSingleForce(curr_node, nb_node);
		}
	}

	// https://stackoverflow.com/32776571/c-iterate-through-an-expanding-container/32776728
	// update velocity and position (semi-Implicit Euler)
	std::vector<SpringNode*> teared_new_nodes; 
	for(int i = 0; i < nodes_.size(); i++) {
		SpringNode* curr_node = nodes_[i];
		if(!curr_node->fixed) {
			glm::vec3 damper_force = - damper_ * curr_node->velocity;
			// std::cout << "damper force: " << glm::to_string(damper_force) 
			// 	<< ", spring force: " << glm::to_string(curr_node.force) << std::endl;
			curr_node->velocity += (curr_node->force + damper_force) / curr_node->mass * delta_t;
			curr_node->position += curr_node->velocity * delta_t;
		}
		
		// if(!curr_node->teared) {
		// 	for(int nb_idx = 0; nb_idx < curr_node->neighbors.size(); nb_idx++) {
		// 		SpringNode* nb_node = curr_node->neighbors[nb_idx];
		// 		checkTear(curr_node, nb_idx, 1.5, teared_new_nodes);
		// 	}
		// }
		
	}
	for(SpringNode* new_node : teared_new_nodes) {
		nodes_.push_back(new_node);
	}
	
	// std::cout << "before refresh cache" << std::endl;
	refreshCache();
	// std::cout << "done refresh cache" << std::endl;
}

void MassSpringSystem::checkTear(SpringNode* curr_node, int nb_idx, float max_deform_rate, 
									std::vector<SpringNode*>& teared_new_nodes) {
	SpringNode* nb_node = curr_node->neighbors[nb_idx];
	float curr_length = glm::length(curr_node->position - nb_node->position);
	float init_length = glm::length(curr_node->init_position - nb_node->init_position);
	if(std::abs(curr_length) > max_deform_rate * init_length) {
		glm::vec3 new_node_curr_pos = (curr_node->position + nb_node->position) / 2.0f;
		glm::vec3 new_node_init_pos = (curr_node->init_position + nb_node->init_position) / 2.0f;

		// SpringNode* new_node = new SpringNode(nodes_.size(), new_node_init_pos, new_node_curr_pos, curr_node->mass / 2.0, false);
		// new_node->teared = true;
		// std::cout << "push new node neighbor" << std::endl;
		
		// new_node->neighbors.push_back(curr_node);
		// std::cout << "done push new node neighbor" << std::endl;
	
		// teared_new_nodes.push_back(new_node);
		// std::cout << "done push new node" << std::endl;
	
		// curr_node->neighbors[nb_idx] = nodes_[nodes_.size() - 1];
		curr_node->neighbors.erase(curr_node->neighbors.begin() + nb_idx);
		// std::cout << "done update neighbor" << std::endl;

		// curr_node->neighbors.erase(curr_node->neighbors.begin() + nb_idx);
	}
}

void MassSpringSystem::resetSystem() {	// update system states and refresh cache.
	// update force
	for(int i = 0; i < nodes_.size(); i++) {
		SpringNode* curr_node = nodes_[i];
		if(curr_node->fixed) continue;	// anchor node, not movable
		curr_node->position = curr_node->init_position;
		curr_node->velocity = glm::vec3(0.0);
		curr_node->force = glm::vec3(0.0, -curr_node->mass * G, 0.0);
	}
	
	// refreshCache();
}

void MassSpringSystem::randomDisturb() {	// update system states and refresh cache.
	int x = std::floor((double)rand() / RAND_MAX * x_size);
	int z = std::floor((double)rand() / RAND_MAX * z_size);
	std::cout << "random change x: " << x << ", z: " << z << std::endl;
	SpringNode* curr_node = nodes_[getNodeIndex(x, z)];
	if(curr_node->fixed) {
		randomDisturb();
		return;
	}
	curr_node->position += (double)rand() / RAND_MAX * grid_width_ * 5.0;
	
	// refreshCache();
}



Particle::Particle(glm::vec3 init_position, float mass, glm::vec2 uv_coords, int grid_x, int grid_z):
			init_position_(init_position), position_(init_position), mass_(mass), uv_coords_(uv_coords), grid_x_(grid_x), grid_z_(grid_z)
{

}

Particle::~Particle() {

}

Spring::Spring(Particle* p1, Particle* p2, Spring* bend_spring):
			p1_(p1), p2_(p2), bend_spring_(bend_spring_)
{
	init_length_ = glm::length(p1_->position_ - p2_->position_);
}


Spring::~Spring() 
{
}


Cloth::Cloth(int x_size, int z_size):
		x_size_(x_size), z_size_(z_size)
{
	// build grid
	for(int x = 0; x < x_size_; x++) {
		float z_offset = (x % 2 == 0)? 0.0 : (0.5 * grid_width_);
		for(int z = 0; z < z_size_; z++) {
			glm::vec3 position(x * grid_width_, init_height_, z * grid_width_ + z_offset);
			glm::vec2 uv_coords(1.0 * x / (x_size_ - 1), 1.0 * z / (z_size_ - 1));
			// std::cout << "uv = " << glm::to_string(uv_coords) << std::endl;
			Particle* particle = new Particle(position, particle_mass_, uv_coords, x, z);
			particles_.push_back(particle);
			// std::cout << "particles " << glm::to_string(particle->position_) << std::endl;
		}
	}
	// std::cout << "cloth built, particle number: " << particles_.size() << std::endl;

	// create triangles
	for(int x = 0; x < x_size_; x++) {
		for(int z = 0; z < z_size_; z++) {
			if(x % 2 == 0) {

				if(gridCoordValid(x, z + 1) && gridCoordValid(x + 1, z)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x + 1, z)]);
					triangles_.push_back(triangle);
				}
				if(gridCoordValid(x, z + 1) && gridCoordValid(x - 1, z)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x - 1, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangles_.push_back(triangle);
				}
			}
			else {
				if(gridCoordValid(x - 1, z + 1) && gridCoordValid(x, z + 1)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x - 1, z + 1)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangles_.push_back(triangle);
				}

				if(gridCoordValid(x + 1, z + 1) && gridCoordValid(x, z + 1)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x + 1, z + 1)]);
					triangles_.push_back(triangle);
				}
			}
		}
	}

	std::map<Particle*, std::map<Particle*, Spring*>> springMap;
	for(Triangle* triangle : triangles_) {
		for(int idx = 0; idx < 3; idx++) {
			Particle* p1 = triangle->particles_[idx];
			Particle* p2 = triangle->particles_[(idx + 1) % 3];
			if(springMap[p1][p2] == nullptr && springMap[p2][p1] == nullptr) {
				Spring* s = new Spring(p1, p2);	// problem: how find bending spring?
				s->triangles_.push_back(triangle);
				p1->springs_.push_back(s);
				p2->springs_.push_back(s);
				springs_.push_back(s);
				springMap[p1][p2] = s;
				springMap[p2][p1] = s;

			}
			else {
				springMap[p1][p2]->triangles_.push_back(triangle);
			}
		}
	}

	std::cout << "triangle number of spring: " << std::endl;
	for(Spring* spring : springs_) {
		std::cout << spring->triangles_.size() << ", ";
	}
	std::cout << std::endl;

	
	std::cout << "spring per particle: " << std::endl;
	for(Particle* particle : particles_) {
		std::cout << particle->springs_.size() << ", ";
	}
	std::cout << std::endl;
	

	// // structure springs
	// for(int x = 0; x < x_size_; x++) {
	// 	if(x % 2 == 0) {
	// 		for(int z = 0; z < z_size_; z++) {
	// 			if(gridCoordValid(x + 1, z - 1)) {
	// 				Spring* s = new Spring(particles_[getParticleIdx(x + 1, z - 1)], particles_[getParticleIdx(x, z)]);
	// 				springs_.push_back(s);
	// 				particles_[getParticleIdx(x + 1, z - 1)]->springs_.push_back(s);
	// 			}
	// 			if(gridCoordValid(x + 1, z)) {
	// 				Spring* s = new Spring(particles_[getParticleIdx(x + 1, z)], particles_[getParticleIdx(x, z)]);
	// 				springs_.push_back(s);
	// 				particles_[getParticleIdx(x + 1, z)]->springs_.push_back(s);
	// 			}
	// 			if(gridCoordValid(x, z + 1)) {
	// 				Spring* s = new Spring(particles_[getParticleIdx(x, z + 1)], particles_[getParticleIdx(x, z)]);
	// 				springs_.push_back(s);
	// 				particles_[getParticleIdx(x, z + 1)]->springs_.push_back(s);
	// 			}
	// 		}
	// 	}
	// 	else {
	// 		for(int z = 0; z < z_size_; z++) {
	// 			if(gridCoordValid(x + 1, z - 1)) {
	// 				Spring* s = new Spring();
	// 				springs_.push_back(s);
	// 			}
	// 			if(gridCoordValid()) {
	// 				Spring* s = new Spring();
	// 				springs_.push_back(s);
	// 			}
	// 			if(gridCoordValid()) {
	// 				Spring* s = new Spring();
	// 				springs_.push_back(s);
	// 			}
	// 		}
	// 	}
	// }

	refreshCache();

}


Cloth::~Cloth() {

}

void Cloth::refreshCache() {
	// vertices and uv_coords
	vertices.clear();
	cloth_uv_coords.clear();
	for(Triangle* triangle : triangles_) {
		for(Particle* p : triangle->particles_) {
			// std::cout << "particle pushed to cache: " << glm::to_string(p->position_) << std::endl;
			vertices.push_back(p->position_);
			cloth_uv_coords.push_back(p->uv_coords_);
		}
	}


	// spring linemesh
	spring_vertices.clear();
	for(Spring* s : springs_) {
		spring_vertices.push_back(s->p1_->position_);
		spring_vertices.push_back(s->p2_->position_);
	}




}

void Cloth::animate(float delta_t) {
	refreshCache();
}


int Cloth::getParticleIdx(int x, int z) {
	return x * z_size_ + z;
}
bool Cloth::gridCoordValid(int x, int z) {
	return x >= 0 && x < x_size_ && z >= 0 && z < z_size_;
}










