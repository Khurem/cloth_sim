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

float line_point_distance(glm::vec3& line_start, glm::vec3& line_end, glm::vec3& point) {
    // std::cout << "start: " << glm::to_string(line_start) << ", end: " << glm::to_string(line_end) << std::endl;
    glm::vec3 sp = point - line_start;
    glm::vec3 se = line_end - line_start;
    float prj_len = glm::dot(glm::normalize(sp), glm::normalize(se)) * glm::length(sp);
    float sp_len = glm::length(sp);

    // std::cout << "len1: " << prj_len << ", len2: " << sp_len << std::endl;

    float res = sqrt(sp_len * sp_len - prj_len * prj_len);
    // std::cout << "line point distance: " << res << std::endl;
    return res;
}

float line_segment_distance_copy(const glm::vec3& line1_start, const glm::vec3& line1_end, 
							const glm::vec3& line2_start, const glm::vec3& line2_end)
{
	glm::vec3 u = line1_end - line1_start;
	glm::vec3 v = line2_end - line2_start;
	glm::vec3 w = line1_start - line2_start;

    float    a = glm::dot(u,u);         // always >= 0
    float    b = glm::dot(u,v);
    float    c = glm::dot(v,v);         // always >= 0
    float    d = glm::dot(u,w);
    float    e = glm::dot(v,w);
    float    D = a*c - b*b;        // always >= 0
    float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < 0.00000001f	) { // the lines are almost parallel
		// printf("this is needed\n");
		sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (std::abs(sN) < 0.00000001f	 ? 0.0 : sN / sD);
    tc = (std::abs(tN) < 0.00000001f	 ? 0.0 : tN / tD);

    // get the difference of the two closest points
    glm::vec3   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    return glm::length(dP);   // return the closest distance

}


Particle::Particle(glm::vec3 init_position, glm::vec3 curr_position, float mass, glm::vec2 uv_coords, int grid_x, int grid_z):
			init_position_(init_position), position_(curr_position), mass_(mass), uv_coords_(uv_coords),
			grid_x_(grid_x), grid_z_(grid_z), is_secondary_(false) , old_position_(curr_position)
{
	resetForce();
	setMovable();
	// setFixed();
}

Particle::Particle(glm::vec3 init_position, glm::vec3 curr_position, float mass, glm::vec2 uv_coords, bool is_secondary):
			init_position_(init_position), position_(curr_position), mass_(mass), uv_coords_(uv_coords),
			grid_x_(-1), grid_z_(-1), is_secondary_(is_secondary) , old_position_(curr_position)
{
	resetForce();
	setMovable();
}

Particle::Particle (const Particle &old_obj):
			init_position_(old_obj.init_position_), position_(old_obj.position_), force_(old_obj.force_), velocity_(old_obj.velocity_), 
			uv_coords_(old_obj.uv_coords_), grid_x_(old_obj.grid_x_), grid_z_(old_obj.grid_z_), mass_(old_obj.mass_), 
			fixed_(old_obj.fixed_), is_secondary_(old_obj.is_secondary_), duplicated_(true) , old_position_(old_obj.old_position_)

{
	setMovable();
}

Particle::~Particle() {

}

void Particle::move(glm::vec3 dist) {
	position_ += dist;
}

void Particle::resetForce() {
	force_ = glm::vec3(0.0f, - 1.0 * mass_ * G, 0.0f);
}


void Particle::addForce(glm::vec3 f) {
	force_ += f;
}

void Particle::setFixed() {
	fixed_ = true;
}

void Particle::setMovable() {
	fixed_ = false;
}

Triangle::Triangle(Particle* p1, Particle* p2, Particle* p3) {
	particles_.push_back(p1);
	particles_.push_back(p2);
	particles_.push_back(p3);
}

Triangle::Triangle() {

}

Triangle::~Triangle() {

}

Spring::Spring(Particle* p1, Particle* p2, float k, bool is_secondary):
			p1_(p1), p2_(p2), k_(k), is_secondary_(is_secondary)
{
	init_length_ = glm::length(p1_->position_ - p2_->position_);
	max_length_ = (1 + max_deform_rate_) * init_length_;
	min_length_ = (1 - max_deform_rate_) * init_length_;
}

void Spring::removeBendSpring() {
	if(bend_spring_ != nullptr) {
		delete bend_spring_;
		bend_spring_ = nullptr;
	}
}


Spring::~Spring() 
{
	if(bend_spring_) {
		delete bend_spring_;
	}
}

void Spring::computeForceQuantity() {
	float curr_length = glm::length(p1_->position_ - p2_->position_);
	float init_length = glm::length(p1_->init_position_ - p2_->init_position_);
	float deform_rate = (init_length - curr_length) / init_length;
	force_quantity_ = deform_rate * k_ * init_length_;
	// // float deform_rate = (init_length_ - curr_length) / init_length_;
	// if(fabs(deform_rate) > max_deform_rate_) {	// constrains. Anti-superelastic.
	// 	deform_rate = deform_rate * (fabs(deform_rate) / max_deform_rate_);
	// }
	// force_quantity_ = deform_rate * k_;

}

void Spring::applyForce() {
	glm::vec3 force1 = glm::normalize(p1_->position_ - p2_->position_) * force_quantity_;
	glm::vec3 force2 = -1.0f * force1;
	p1_->addForce(force1);
	p2_->addForce(force2);
}

void Spring::replaceTriangle(Triangle* t_old, Triangle* t_new) {
	for(int i = 0; i < triangles_.size(); i++) {
		if(triangles_[i] == t_old) {
			triangles_[i] = t_new;
			return;
		}
	}
}

void Spring::replaceParticle(Particle* p_old, Particle* p_new) {
	if(p1_ == p_old) {
		p1_ = p_new;
	}
	else if(p2_ == p_old) {
		p2_ = p_new;
	}
	else {
		std::cout << "the particle you want to replace in the spring doesn't exist" << std::endl;
		throw("the particle you want to replace in the spring doesn't exist");
	}
}

void Cloth::resetCloth() {
	for(Particle* p : particles_) {
		p->position_ = p->init_position_;
		p->velocity_ = glm::vec3(0.0);
	}
	setInitAnchorNodes();
}

void Cloth::setInitAnchorNodes() {

	particles_[getParticleIdx(0, 0)]->setFixed();								//(0, 0)
	// particles_[getParticleIdx(0, z_size_ - 1)]->setFixed();						//(0, 1)
	particles_[getParticleIdx(x_size_ - 1, 0)]->setFixed();						//(1, 0)
	// particles_[getParticleIdx(x_size_ - 1, z_size_ - 1)]->setFixed(); //(1, 1)
	// particles_[getParticleIdx(0, 0)]->setFixed();
	// // particles_[getParticleIdx(0, z_size_ - 1)]->setFixed();
	// particles_[getParticleIdx(x_size_ - 1, 0)]->setFixed();
	// particles_[getParticleIdx(x_size_ - 1, z_size_ - 1)]->setFixed();

	// particles_[getParticleIdx(x_size_ / 2 - 1, 0)]->setFixed();
	// particles_[getParticleIdx(x_size_ / 2 - 1, z_size_ - 1)]->setFixed();
	// for(int x = 0; x < x_size_; x++) {
	// 	particles_[getParticleIdx(x, 0)]->setFixed();
	// }
}

Cloth::Cloth(int x_size, int z_size):
		x_size_(x_size), z_size_(z_size)
{
	// build grid
	float total_x_width = (x_size_ - 1) * grid_width_, total_z_width = (z_size_ - 1 + 0.5) * grid_width_;
	for(int x = 0; x < x_size_; x++) {
		float z_offset = (x % 2 == 0)? 0.0 : (0.5 * grid_width_);
		for(int z = 0; z < z_size_; z++) {
			float pos_x = x * grid_width_, pos_z = z * grid_width_ + z_offset;
			glm::vec3 position(pos_x, init_height_, pos_z);
			glm::vec2 uv_coords(pos_x / total_x_width, pos_z / total_z_width);
			std::cout << "uv = " << glm::to_string(uv_coords) << std::endl;
			Particle* particle = new Particle(position, position, particle_mass_, uv_coords, x, z);
			particles_.push_back(particle);
			// std::cout << "particles " << glm::to_string(particle->position_) << std::endl;
		}
	}
	// std::cout << "cloth built, particle number: " << particles_.size() << std::endl;

	// set two anchor nodes. For experiments.
	setInitAnchorNodes();

	// create triangles
	for(int x = 0; x < x_size_; x++) {
		for(int z = 0; z < z_size_; z++) {
			if(x % 2 == 0) {

				if(gridCoordValid(x, z + 1) && gridCoordValid(x + 1, z)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x + 1, z)]);
					triangles_.insert(triangle);
				}
				if(gridCoordValid(x, z + 1) && gridCoordValid(x - 1, z)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x - 1, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangles_.insert(triangle);
				}
			}
			else {
				if(gridCoordValid(x - 1, z + 1) && gridCoordValid(x, z + 1)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x - 1, z + 1)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangles_.insert(triangle);
				}

				if(gridCoordValid(x + 1, z + 1) && gridCoordValid(x, z + 1)) {
					Triangle* triangle = new Triangle();
					triangle->particles_.push_back(particles_[getParticleIdx(x, z)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x, z + 1)]);
					triangle->particles_.push_back(particles_[getParticleIdx(x + 1, z + 1)]);
					triangles_.insert(triangle);
				}
			}
		}
	}

	// create structural springs
	std::map<Particle*, std::map<Particle*, Spring*>> spring_map_;
	for(Triangle* triangle : triangles_) {
		for(int idx = 0; idx < 3; idx++) {
			Particle* p1 = triangle->particles_[idx];
			Particle* p2 = triangle->particles_[(idx + 1) % 3];
			if(!containsStructSpring(p1, p2)) {
				Spring* s = addStructSpring(p1, p2, struct_k_, false);	// problem: how find bending spring?
				s->triangles_.push_back(triangle);
				// std::cout << "add triangle when create spring" << std::endl;
			}
			else {
				getStructSpring(p1, p2) ->triangles_.push_back(triangle);
				// std::cout << "add triangle to existing spring" << std::endl;
			}
		}
	}


	// add bending springs
	for(Spring* spring : springs_) {
		Particle* p1;
		Particle* p2;
		if(spring->p1_->grid_x_ < spring->p2_->grid_x_) {
			p1 = spring->p1_;
			p2 = spring->p2_;
		}
		else {
			p1 = spring->p2_;
			p2 = spring->p1_;
		}

		// create bending springs
		int bend_x1 = -1, bend_z1 = -1, bend_x2 = -1, bend_z2 = -1;

		if(p1->grid_x_ % 2 == 0) {
			if(p1->grid_x_ == p2->grid_x_) {
				bend_x1 = p1->grid_x_ - 1; 
				bend_z1 = std::min(p1->grid_z_, p2->grid_z_);
				bend_x2 = p1->grid_x_ + 1; 
				bend_z2 = std::min(p1->grid_z_, p2->grid_z_);
			}
			else if(p1->grid_z_ == p2->grid_z_) {
				bend_x1 = p1->grid_x_;
				bend_z1 = p1->grid_z_ + 1;
				bend_x2 = p2->grid_x_;
				bend_z2 = p2->grid_z_ - 1;
				
			}
			else {
				bend_x1 = p1->grid_x_;
				bend_z1 = p1->grid_z_ - 1;
				bend_x2 = p2->grid_x_;
				bend_z2 = p2->grid_z_ + 1;
				
			}
		}
		else {
			if(p1->grid_x_ == p2->grid_x_) {
				bend_x1 = p1->grid_x_ - 1;
				bend_z1 = std::max(p1->grid_z_, p2->grid_z_);
				bend_x2 = p1->grid_x_ + 1;
				bend_z2 = std::max(p1->grid_z_, p2->grid_z_);
				
			}
			else if(p1->grid_z_ == p2->grid_z_) {
				bend_x1 = p1->grid_x_;
				bend_z1 = p1->grid_z_ - 1;
				bend_x2 = p2->grid_x_;
				bend_z2 = p2->grid_z_ + 1;
			}
			else {
				bend_x1 = p1->grid_x_;
				bend_z1 = p1->grid_z_ + 1;
				bend_x2 = p2->grid_x_;
				bend_z2 = p2->grid_z_ - 1;
			}

		}
		if(gridCoordValid(bend_x1, bend_z1) && gridCoordValid(bend_x2, bend_z2)) {
			Spring* bend_spring = new Spring(particles_[getParticleIdx(bend_x1, bend_z1)], 
										particles_[getParticleIdx(bend_x2, bend_z2)],
										bend_sheer_k_);
			spring->bend_spring_ = bend_spring;
		}

	}

	for(Particle* p : particles_) {
		std::cout << "particle spring number: " << p->springs_.size() << std::endl;
	}

	// update cache vertices
	refreshCache();

}


Cloth::~Cloth() {

}

void Cloth::collisionWithFloor(){
	for(Particle* p : particles_) {
		if(p->position_.y < kFloorY){
			// std::cout << "FLOOR HIT ME\n";
			p->position_.y = kFloorY + kFloorEps;
			p->setFixed();
			// p->velocity_ = glm::vec3(0.0f);
			// p->force_ = glm::vec3(0.0f);
		}
	}
}

void Cloth::tear(Spring* s) {
	Particle *p1 = s->p1_, *p2 = s->p2_;	// particles of current springs.
	std::cout << "to remove spring at " << glm::to_string(glm::vec2(p1->grid_x_, p1->grid_z_)) 
				<< ", " << glm::to_string(glm::vec2(p2->grid_x_, p2->grid_z_)) << std::endl;

	Triangle *t1 = nullptr, *t2 = nullptr;	// neighboring triangles. (if any)
	if(s->triangles_.size() >= 1) {
		t1 = s->triangles_[0];
	}	
	if(s->triangles_.size() >= 2) {
		t2 = s->triangles_[1];
	}

	// center of the teared spring.
	glm::vec3 init_center_position = (p1->init_position_ + p2->init_position_) / 2.0f;
	glm::vec3 curr_center_position = (p1->position_ + p2->position_) / 2.0f;
	glm::vec2 center_uv_coords = (p1->uv_coords_ + p2->uv_coords_) / 2.0f;

	// two new particles created because of the tearing
	glm::vec3 pp1_init_pos = init_center_position;
	glm::vec3 pp1_curr_pos = p1->position_ + (curr_center_position - p1->position_) * 0.85f;
	glm::vec2 pp1_uv_coords = center_uv_coords;
	Particle* pp1 = new Particle(pp1_init_pos, pp1_curr_pos, p1->mass_ / 2.0, pp1_uv_coords, true);
	particles_.push_back(pp1);
	Spring* ss1 = addStructSpring(p1, pp1, struct_k_, true);

	glm::vec3 pp2_init_pos = init_center_position;
	glm::vec3 pp2_curr_pos = p2->position_ + (curr_center_position - p2->position_) * 0.85f;
	glm::vec2 pp2_uv_coords = center_uv_coords;
	Particle* pp2 = new Particle(pp2_init_pos, pp2_curr_pos, p2->mass_ / 2.0, pp2_uv_coords, true);
	particles_.push_back(pp2);
	Spring* ss2 = addStructSpring(p2, pp2, struct_k_, true);

	if(t1) {
		Particle* nb_p1 = getNeighborParticle(t1, s);
		Triangle* tt1 = new Triangle(nb_p1, p1, pp1);
		Triangle* tt2 = new Triangle(nb_p1, pp2, p2);

		triangles_.insert(tt1);
		triangles_.insert(tt2);

		ss1->triangles_.push_back(tt1);
		ss2->triangles_.push_back(tt2);

		Spring* ss11 = addStructSpring(pp1, nb_p1, struct_k_, true);
		Spring* ss12 = addStructSpring(pp2, nb_p1, struct_k_, true);
		ss11->triangles_.push_back(tt1);
		ss12->triangles_.push_back(tt2);

		getStructSpring(p1, nb_p1)->replaceTriangle(t1, tt1);
		getStructSpring(p2, nb_p1)->replaceTriangle(t1, tt2);
		for(Spring* nb_p1_s : nb_p1->springs_) {
			nb_p1_s->removeBendSpring();
		}
		triangles_.erase(t1);
		delete t1;
	}
	if(t2) {
		Particle* nb_p2 = getNeighborParticle(t2, s);
		Triangle* tt1 = new Triangle(p1, nb_p2, pp1);
		Triangle* tt2 = new Triangle(pp2, nb_p2, p2);

		triangles_.insert(tt1);
		triangles_.insert(tt2);

		ss1->triangles_.push_back(tt1);
		ss2->triangles_.push_back(tt2);

		Spring* ss21 = addStructSpring(pp1, nb_p2, struct_k_, true);
		Spring* ss22 = addStructSpring(pp2, nb_p2, struct_k_, true);
		ss21->triangles_.push_back(tt1);
		ss22->triangles_.push_back(tt2);	
		
		getStructSpring(p1, nb_p2)->replaceTriangle(t2, tt1);
		getStructSpring(p2, nb_p2)->replaceTriangle(t2, tt2);
		for(Spring* nb_p2_s : nb_p2->springs_) {
			nb_p2_s->removeBendSpring();
		}
		triangles_.erase(t2);
		delete t2;

	}
	removeStructSpring(s);
}

Particle* Cloth::getNeighborParticle(Triangle* t1, Spring* s) {
	for(Particle* p : t1->particles_) {
		if(p != s->p1_ && p != s->p2_) {
			return p;
		}
	}
	std::cout << "get neighbor particle function correctly!" << std::endl;
	throw("get neighbor particle function correctly!");
	return nullptr;
}


void Cloth::refreshCache() {
	// vertices and uv_coords
	vertices.clear();
	cloth_uv_coords.clear();
	vertex_normals.clear();
	for(Triangle* triangle : triangles_) {
		for(Particle* p : triangle->particles_) {
			// std::cout << "particle pushed to cache: " << glm::to_string(p->position_) << std::endl;
			vertices.push_back(p->position_);
			cloth_uv_coords.push_back(p->uv_coords_);
			vertex_normals.push_back(p->vertex_normal_);
		}
	}

	// spring linemesh
	struct_spring_vertices.clear();
	bend_spring_vertices.clear();
	for(Spring* s : springs_) {
		struct_spring_vertices.push_back(s->p1_->position_);
		struct_spring_vertices.push_back(s->p2_->position_);

		if(s->bend_spring_) {
			// std::cout << "push bend spring" << std::endl;
		
			bend_spring_vertices.push_back(s->bend_spring_->p1_->position_);
			bend_spring_vertices.push_back(s->bend_spring_->p2_->position_);

		}
	}
	// std::cout << "end push bend spring" << std::endl;

}

void Cloth::animate(float delta_t) {
	// clear all forces except for gravity
	std::vector<Particle*> splitted_particles;
	for(auto itr = particles_.begin(); itr != particles_.end(); itr++) {
		std::map<int, std::unordered_set<Particle*>> particle_groups;
		groupNeighbors(*itr, particle_groups);
		duplicateParticles(*itr, particle_groups, splitted_particles);
	}

	for(Particle* splitted_p : splitted_particles) {
		particles_.push_back(splitted_p);
	}

	for(Particle* particle : particles_) {
		particle->resetForce();
	}


	// update forces
	for(Spring* struct_s : springs_) {
		struct_s->computeForceQuantity();
		// TODO: if force quantity exceeds limit, break the spring
		struct_s->applyForce();
		if(struct_s->force_quantity_ == 0.0f) {
			// std::cout << "spring force quantity zero" << std::endl;
		}

		if(struct_s->bend_spring_) {
			struct_s->bend_spring_->computeForceQuantity();
			struct_s->bend_spring_->applyForce();
		}
	}


	// update particle velocity and positions
	for(Particle* particle : particles_) {
		// Update velocity and positions
		if(!particle->fixed_) {
			glm::vec3 damper_force = -damper_ * particle->velocity_;
			glm::vec3 acceleration = (particle->force_ + damper_force) / particle->mass_;
			particle->velocity_ += acceleration * delta_t * 0.5f;
			particle->position_ += particle->velocity_ * delta_t;

		}
	}

	std::queue<Particle*> start_particles;
	for(Particle* p : particles_) {
		if(p->fixed_) {
			start_particles.emplace(p);
		}
	}
	bfsConstrain(start_particles);	
	for(Particle* p : particles_) {
		if(!(*p->springs_.begin())->constrained_) {
			start_particles = std::queue<Particle*>();
			start_particles.emplace(p);
			bfsConstrain(start_particles);
		}
	}

	collisionWithFloor();
	// particle positions determined. Compute vertex normals.
	for(Particle* p : particles_) {
		p->face_normals_.clear();
	}
	for(Triangle* t : triangles_) {
		t->face_normal_ = glm::normalize(glm::cross(t->particles_[1]->position_ - t->particles_[0]->position_, 
														t->particles_[2]->position_ - t->particles_[0]->position_));
		for(Particle* p : t->particles_) {
			p->face_normals_.push_back(t->face_normal_);
		}
	}
	for(Particle* p : particles_) {
		glm::vec3 normal_accumulate(0.0f, 0.0f, 0.0f);
		for(glm::vec3& face_normal : p->face_normals_) {
			normal_accumulate += face_normal;
		}
		p->vertex_normal_ = normal_accumulate / (1.0f * p->face_normals_.size());
	}
	setCurrentParticle();
	setCurrentSpring();
	// std::cout << "pick ray start: " << glm::to_string(pick_ray_start) << std::endl;
	if(picked_spring_) {
		if(to_tear && !picked_spring_->is_secondary_) {
			tear(picked_spring_);
		}
	}
	refreshCache();
	time_ += delta_t;
	// std::cout << std::endl;
}

void Cloth::bfsConstrain(std::queue<Particle*>& q) {
	// std::queue<Particle*> q;
	// q.emplace(p_start);
	while(!q.empty()) {
		Particle* p = q.front();
		q.pop();
		for(Spring* s : p->springs_) {
			if(s->constrained_) {
				continue;
			}
			Particle* nb_p = s->p1_ == p ? s->p2_ : s->p1_;
			float len = glm::length(p->position_ - nb_p->position_);
			if(len > s->max_length_) {
				nb_p->position_ = p->position_ + glm::normalize(nb_p->position_ - p->position_) * s->max_length_;
			}
			s->constrained_ = true;
			q.emplace(nb_p);

		}

	}

}



void Cloth::groupNeighbors(Particle* p, std::map<int, std::unordered_set<Particle*>>& groups) {
	std::vector<Particle*> nb_particles;
	std::vector<Spring*> nb_springs;
	for(Spring* s : p->springs_) {
		if(s->p1_ == nullptr || s->p2_ == nullptr) {
			std::cout << "spring has null node" << std::endl;
			throw "spring has null node";

		}
		if(!containsStructSpring(s->p1_, s->p2_)) {
			std::cout << "spring not recorded in map!" << std::endl;
			throw("spring not recorded in map!");
		}
		Particle* nb_particle = nullptr;
		if(s->p1_ == p) {
			nb_particle = s->p2_;
		}
		else if(s->p2_ == p) {
			nb_particle = s->p1_;
		}
		else {
			std::cout << "spring doesn't belong to current particle" << std::endl;
			throw("spring doesn't belong to current particle");
		}
		nb_particles.push_back(nb_particle);
	}
	// we need union-find here!
	std::vector<int> uf;
	uf.resize(nb_particles.size());
	for(int i = 0; i < uf.size(); i++) {
		uf[i] = i;	// if -1, the root of the group
	}
	for(int i = 0; i < nb_particles.size(); i++) {
		Particle *p1 = nb_particles[i];
		for(int j = i + 1; j < nb_particles.size(); j++) {
			Particle *p2 = nb_particles[j];
			if(containsStructSpring(p1, p2)) {
				// std::cout << "spring " << i << " connected to spring " << j << std::endl;
				uf[findRoot(uf, j)] = findRoot(uf, i);
				nb_springs.push_back(getStructSpring(p1, p2));
			}
		}
	}

	for(int i = 0; i < uf.size(); i++) {
		if(uf[i] == i) {
			// std::cout << "gorup " << i << " created" << std::endl;
			groups[i] = std::unordered_set<Particle*>();
		}
	}	

	for(int i = 0; i < uf.size(); i++) {
		if(containsStructSpring(p, nb_particles[i])) {
			int group_num = findRoot(uf, i);
			groups[group_num].insert(nb_particles[i]);
		}
		else {
			std::cout << "neighbor spring grouped doesn't exist" << std::endl;
			throw("neighbor spring grouped doesn't exist");
		}
		
	}
	if(groups.size() > 1) {
		for(Spring* nb_s : nb_springs) {
			nb_s->removeBendSpring();
		}
	}
}

int Cloth::findRoot(std::vector<int>& uf, int idx) {
	while(idx != uf[idx]) {
		idx = uf[idx];
	}
	return idx;
}

void Cloth::duplicateParticles(Particle* p, std::map<int, std::unordered_set<Particle*>>& groups, std::vector<Particle*>& new_particles) {
	if(groups.size() <= 1) {
		return;
	} 
	int group_count = groups.size(); 
	for(auto const& group : groups) {
		if(group_count == 1) break;	// if only one group, don't need to split the origin particle
		group_count--;
		const std::unordered_set<Particle*>& group_particles = group.second;
		Particle* p_copy = new Particle(*p);
		new_particles.push_back(p_copy);
		for(Particle* nb_particle : group_particles) {
			Spring* s = getStructSpring(p, nb_particle);
			// replace the particle in the original spring
			p->springs_.erase(s);
			spring_map_[p][nb_particle] = nullptr;
			spring_map_[nb_particle][p] = nullptr;

			s->replaceParticle(p, p_copy);
			p_copy->springs_.insert(s);
			spring_map_[p_copy][nb_particle] = s;
			spring_map_[nb_particle][p_copy] = s;
			

			for(Triangle* t : s->triangles_) {	// replace the particle in old triangles. At most two triangles
				for(int p_idx = 0; p_idx < t->particles_.size(); p_idx++) {
					if(t->particles_[p_idx] == p) {
						std::cout << "triangle particle replaced by new particle" << std::endl;
						t->particles_[p_idx] = p_copy;
					}
				}
			}

		}
		
	}
}




void Cloth::setCurrentSpring() {
	picked_spring_ = nullptr;
	float min_distance = std::numeric_limits<float>::max();
	for(Spring* s : springs_) {	// iterate all springs, and find the one with min distance
		float curr_distance = line_segment_distance_copy(pick_ray_start, pick_ray_end, s->p1_->position_, s->p2_->position_);
		if(curr_distance < SPRING_CYLINDER_RADIUS && curr_distance < min_distance) {
			min_distance = curr_distance;
			picked_spring_ = s;
		}
	}
}

void Cloth::setCurrentParticle() {
	picked_particle_ = nullptr;
	float min_distance = std::numeric_limits<float>::max();
	for(Particle* p : particles_) {
		float curr_distance = line_point_distance(pick_ray_start, pick_ray_end, p->position_);
		if((curr_distance < PARTICLE_RADIUS) && (curr_distance < min_distance)) {
			min_distance = curr_distance;
			picked_particle_ = p;
		}
	}
	// std::cout << "particle min distance: " << min_distance << std::endl;
}


int Cloth::getParticleIdx(int x, int z) {
	return x * z_size_ + z;
}
bool Cloth::gridCoordValid(int x, int z) {
	return x >= 0 && x < x_size_ && z >= 0 && z < z_size_;
}

bool Cloth::containsStructSpring(Particle* p1, Particle* p2) {
	return (spring_map_[p1][p2] != nullptr) || (spring_map_[p2][p1] != nullptr);
}

Spring* Cloth::addStructSpring(Particle* p1, Particle* p2, float k, bool is_secondary) {
	if(containsStructSpring(p1, p2)) {
		std::cout << "the sprinig you want to create already exists!" << std::endl;
		throw("the sprinig you want to create already exists!");
	}
	Spring* s = new Spring(p1, p2, k, is_secondary);
	spring_map_[p1][p2] = s;
	spring_map_[p2][p1] = s;
	p1->springs_.insert(s);
	p2->springs_.insert(s);

	springs_.insert(s);
	return s;
}
Spring* Cloth::getStructSpring(Particle* p1, Particle* p2) {
	if(!containsStructSpring(p1, p2)) {
		std::cout << "struct spring doesn't exist!" << std::endl;
		throw("struct spring doesn't exist!");
	}
	return spring_map_[p1][p2];
}

void Cloth::removeStructSpring(Spring* s) {
	s->p1_->springs_.erase(s);
	s->p2_->springs_.erase(s);
	springs_.erase(s);
	spring_map_[s->p1_][s->p2_] = nullptr;
	spring_map_[s->p2_][s->p1_] = nullptr;
	delete s;
}



