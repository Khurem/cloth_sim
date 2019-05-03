#include "gui.h"
#include "config.h"
#include <jpegio.h>
#include <iostream>
#include <algorithm>
#include <debuggl.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>

namespace {
	// FIXME: Implement a function that performs proper
	//        ray-cylinder intersection detection
	// TIPS: The implement is provided by the ray-tracer starter code.
}

GUI::GUI(GLFWwindow* window, int view_width, int view_height, int preview_height)
	:window_(window), preview_height_(preview_height)
{
	glfwSetWindowUserPointer(window_, this);
	glfwSetKeyCallback(window_, KeyCallback);
	glfwSetCursorPosCallback(window_, MousePosCallback);
	glfwSetMouseButtonCallback(window_, MouseButtonCallback);
	glfwSetScrollCallback(window_, MouseScrollCallback);

	glfwGetWindowSize(window_, &window_width_, &window_height_);
	if (view_width < 0 || view_height < 0) {
		view_width_ = window_width_;
		view_height_ = window_height_;
	} else {
		view_width_ = view_width;
		view_height_ = view_height;
	}
	float aspect_ = static_cast<float>(view_width_) / view_height_;
	projection_matrix_ = glm::perspective((float)(kFov * (M_PI / 180.0f)), aspect_, kNear, kFar);
	// timer_ = tic();
}

GUI::~GUI()
{
}

void GUI::assignMesh(Mesh* mesh)
{
	mesh_ = mesh;
	center_ = mesh_->getCenter();
}

void GUI::assignMassSpringSystem(MassSpringSystem* system) 
{
	ms_system_ = system;
}

void GUI::keyCallback(int key, int scancode, int action, int mods)
{
#if 0
	if (action != 2)
		std::cerr << "Key: " << key << " action: " << action << std::endl;
#endif
	if (key == GLFW_KEY_J && action == GLFW_RELEASE) {
		to_random_disturb_ = true;
	}

	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window_, GL_TRUE);
		return ;
	}
	if (key == GLFW_KEY_S && (mods & GLFW_MOD_CONTROL)) {
		if (action == GLFW_RELEASE)
			mesh_->saveAnimationTo("animation.json");
		return ;
	}

	if (mods == 0 && captureWASDUPDOWN(key, action))
		return ;
	if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT) {
		float roll_speed = 0.2;
		if (key == GLFW_KEY_RIGHT)
			roll_speed = -roll_speed_;
		else
			roll_speed = roll_speed_;
		// FIXME: actually roll the bone here
		bool drag_bone = drag_state_ && current_button_ == GLFW_MOUSE_BUTTON_LEFT;
		if( current_bone_ != -1) {
			Joint& curr_joint = mesh_->skeleton.joints[current_bone_];
			glm::vec3 curr_joint_pos = mesh_->getJointPos(curr_joint.joint_index);
			glm::vec3 parent_joint_pos = mesh_->getJointPos(curr_joint.parent_index);
			glm::vec3 rotate_axis = glm::normalize(curr_joint_pos - parent_joint_pos);
			glm::fquat rotate_quat = glm::angleAxis(roll_speed, rotate_axis);
			glm::mat4 rotate_matrix = glm::rotate(2 * rotation_speed_, rotate_axis);
		
			glm::fquat rotate2_quat = glm::normalize(glm::toQuat(rotate_matrix));
			mesh_->rotate_bone(curr_joint.parent_index, rotate2_quat, false);
			pose_changed_ == true;
			mesh_->updateAnimation();
	}
	} else if (key == GLFW_KEY_C && action != GLFW_RELEASE) {
		fps_mode_ = !fps_mode_;
	} else if (key == GLFW_KEY_LEFT_BRACKET && action == GLFW_RELEASE) {
		time_speed_ /= 5.0;
	} else if (key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_RELEASE) {
		time_speed_ *= 5.0;
	} else if (key == GLFW_KEY_T && action != GLFW_RELEASE) {
		transparent_ = !transparent_;
	}

	// FIXME: implement other controls here.
	else if (key == GLFW_KEY_U && action != GLFW_RELEASE) {
		mesh_->overwriteThisFrame(current_frame);
		// showPrev = true;
	}  else if(key == GLFW_KEY_DELETE && action != GLFW_RELEASE) {
		if(current_frame != -1) {
			mesh_->deleteThisFrame(current_frame);
			if(mesh_->keyframes.size() == 0){
				current_frame = -1;
			}
		}
	}
	else if (key == GLFW_KEY_SPACE && action != GLFW_RELEASE) {
			mesh_->changeCurrentSkeleton(current_frame);
	}
	else if (key == GLFW_KEY_F && action != GLFW_RELEASE) {
		mesh_->addKeyframe();
		showPrev = true;
	} 
	else if (key == GLFW_KEY_P && action != GLFW_RELEASE) {
		// printf("do we have keyframes ready to roll??%d\n", mesh_->keyframes.size());
		if(!play_) {
			play_ = true;
			
			timer.reset();
			timer.start();
		} else {
			play_ = false;
		}
	} 
	else if(key == GLFW_KEY_R && action != GLFW_RELEASE) {
	
		timer.reset();
		timer.start();
		time_ = 0.0;
		play_ = false;
		if(mesh_->keyframes.size() > 0){
			mesh_->setFirstFrame();
		}
		mesh_->updateAnimation();
		reset_ms_system_ = true;

	}else if (key == GLFW_KEY_V && action != GLFW_RELEASE) {
		if(mesh_->keyframes.size() > 0){
			if (!isPlaying()) {
			play_ = true;
			
			timer.reset();
			timer.start();
			time_ = 0.0;
			video = true;
		} else {
			play_ = false;
		} 
		}
		
	} else if(key == GLFW_KEY_O && action != GLFW_RELEASE){
		mesh_->squadIt = !mesh_->squadIt;
	
	} else if(key == GLFW_KEY_K && action != GLFW_RELEASE){
		upSample = !upSample;
	}
}

float line_segment_distance(const glm::vec3& line1_start, const glm::vec3& line1_end, 
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

// float GUI::angle_between_two_directs_2D(glm::vec2 direct1, glm::vec2 direct2) {
// 	direct1 = glm::normalize(direct1);
// 	direct2 = glm::normalize(direct2);
// 	float cos_theta = glm::dot(direct1, direct2);
// 	cos_theta = std::max(cos_theta, -1.0f + 0.00001f);
// 	cos_theta = std::min(cos_theta, 1.0f - 0.00001f);
// 	float theta = glm::acos(cos_theta);
// 	float angle_direct = direct1.x * direct2.y - direct1.y * direct2.x;	// cross product, determine rot direct
// 	if(angle_direct < 0.0f - 0.001f) {
// 		return theta;
// 	}
// 	return -theta;

// }


void GUI::mousePosCallback(double mouse_x, double mouse_y)
{
	last_x_ = current_x_;
	last_y_ = current_y_;
	current_x_ = mouse_x;
	current_y_ = window_height_ - mouse_y;
	float delta_x = current_x_ - last_x_;
	float delta_y = current_y_ - last_y_;
	if (sqrt(delta_x * delta_x + delta_y * delta_y) < 1e-15)
		return;
	
	glm::vec3 mouse_direction = glm::normalize(glm::vec3(delta_x, delta_y, 0.0f));
	glm::vec2 mouse_start = glm::vec2(last_x_, last_y_);
	glm::vec2 mouse_end = glm::vec2(current_x_, current_y_);
	glm::uvec4 viewport = glm::uvec4(0, 0, view_width_, view_height_);
	glm::vec2 mouse_delta = mouse_end - mouse_start;

	bool drag_camera = drag_state_ && current_button_ == GLFW_MOUSE_BUTTON_RIGHT;
	bool drag_bone = drag_state_ && current_button_ == GLFW_MOUSE_BUTTON_LEFT;

	bool drag_scroll = scrolling && current_button_ == GLFW_MOUSE_BUTTON_LEFT;
	
	if (drag_scroll) {
		// printf("i'm here, hellow");
		if (mesh_->textures.size() > 3) {

			int theight = mesh_->textures.size() * preview_height_;
			if (current_y_ > (theight - frame_shift - window_height_)/theight * window_height_ && current_y_ < (theight - frame_shift)/theight * window_height_) {
				int max_shift = preview_height_ * mesh_->keyframes.size() - window_height_;
				float delta = delta_y / window_height_ * theight;
				if (frame_shift - delta >=0 && ((int)frame_shift<= max_shift   || delta >= 0)){
					frame_shift -= delta;
				} else {
					return;
				}
			}
		}
	}

if (mouse_x > view_width_)
		return ;

	if (drag_camera) {
		// printf("%d scroll in motion here \n", drag_scroll);
		glm::vec3 axis = glm::normalize(
				orientation_ *
				glm::vec3(mouse_direction.y, -mouse_direction.x, 0.0f)
				);
		orientation_ =
			glm::mat3(glm::rotate(rotation_speed_, axis) * glm::mat4(orientation_));
		look_ = glm::column(orientation_, 2);
		up_ = glm::column(orientation_, 1);
		tangent_ = glm::column(orientation_, 0);
		
	} else if (drag_bone && current_bone_ != -1) {
		// FIXME: Handle bone rotation
		
		int parent = mesh_->skeleton.joints[current_bone_].parent_index;
		// printf("bonezo and parenzo %d %d \n", current_bone_, parent_index);
		glm::vec3 parent_pos = mesh_->getJointPos(parent);
		glm::vec3 projected_parent_pos = glm::project(parent_pos,
											view_matrix_,
											projection_matrix_,
											viewport);

		
		// float slope1= start_direct_2D[1] / start_direct_2D[0];
		// float slope2 = end_direct_2D[1] / end_direct_2D[0];
		//float angle_2D = angle_between_two_directs_2D(start_direct_2D, end_direct_2D);
		// float angel = glm::atan(slope2 - slope1)/ (1 - (slope1 * slope2));
		// float anglea = glm::pow(glm::atan(end_.y,end_.x),2) - glm::pow(glm::atan(start_.y,start_.x),2);
		//use atan2 here instead of atan
		
		glm::vec3 start1_ = glm::vec3(glm::normalize(glm::vec2(last_x_ - projected_parent_pos.x, last_y_ - projected_parent_pos.y)), 0.0);
		// printf("%f %f rottafsdfsdfsdfs\n", start_direct_2D[0], start_direct_2D[1]);
		glm::vec3 end1_ = glm::vec3(glm::normalize(glm::vec2(current_x_ - projected_parent_pos.x, current_y_ - projected_parent_pos.y)),0.0);
		// glm::vec3 rotation_axis = glm::normalize(glm::cross(mouse_vector, xAxis));
		glm::vec2 start_ = glm::normalize(glm::vec2(last_x_ - projected_parent_pos.x, last_y_ - projected_parent_pos.y));
		// printf("%f %f rottafsdfsdfsdfs\n", start_direct_2D[0], start_direct_2D[1]);
		glm::vec2 end_ = glm::normalize(glm::vec2(current_x_ - projected_parent_pos.x, current_y_ - projected_parent_pos.y));
		float dotstend = glm::dot(start_, end_);
		float angler = glm::acos(dotstend);
		float other =  -glm::atan(start_.x*end_.y-start_.y*end_.x,start_.x*end_.x+start_.y*end_.y);
		float dir = start_.x * end_.y - start_.y * end_.x;
		if(dir >= 0.0f - 0.001f) {
		angler = -angler;
		}
		glm::vec3 axos = glm::cross(look_, mouse_direction);
		float mixFactor = rotation_speed_;
		
		glm::vec3 rotate_axis = glm::normalize(parent_pos - eye_);
		// glm::fquat roat =  mesh_->quaternion_between_two_directs(start1_, end1_);
		
		glm::fquat rotate_quat = glm::angleAxis(other, rotate_axis);
		
			glm::vec3 diff_rotate = glm::normalize(glm::cross(end1_ - start1_, eye_));
			
			// 	angle_diff = angle_diff;
			// } else {
			// 	angle_diff = -angle_diff;
			// }
			//glm::vec3 diff_rotate = glm::normalize(glm::cross(look_, glm::vec3(mouse_direction.y, -mouse_direction.x, 0.0f)));
			if(other < 0.0){
			rotate_axis *= -1;
		}
			glm::mat4 rotate_matrix = glm::rotate(rotation_speed_, rotate_axis);
		
		glm::fquat rotate2_quat = glm::normalize(glm::toQuat(rotate_matrix));
		mesh_->rotate_bone(parent, rotate2_quat, false);
		
		pose_changed_ = true;
			return ;
	}

	// FIXME: highlight bones that have been moused over
	current_bone_ = -1;
	glm::vec3 mouse = glm::unProject(glm::vec3(current_x_, current_y_, 1.0f),view_matrix_,projection_matrix_,viewport);
	glm::vec3 r_Direction = glm::normalize(mouse - eye_);
	glm::vec3 r_End = eye_ + 100.0f * r_Direction;
	float min_distance = std::numeric_limits<float>::max();
	for(int i = 0; i < mesh_->getNumberOfBones(); i++) {	
		int parent_ID = mesh_->skeleton.joints[i].parent_index;
		if(parent_ID == -1) {
			continue;	// this joint is a root.
		}
		glm::vec3 bone_start = mesh_->getJoint(i).position;

		glm::vec3 bone_end = mesh_->getJoint(parent_ID).position;
		// float disto = 1.0;
		// if(disto < min_distance) {
		// 	printf("never once did it reach this point\n");
		// 	min_distance = disto;
		// 	current_bone_ = i;
		// }
		float curr_distance = line_segment_distance(eye_, r_End, bone_start, bone_end);
		if(curr_distance < kCylinderRadius && curr_distance < min_distance) {
			min_distance = curr_distance;
			current_bone_ = i;
		// printf("current bone is %d\n", mesh_->skeleton.joints[current_bone_].parent_index);
}
	}
}

void GUI::mouseButtonCallback(int button, int action, int mods)
{
	if (current_x_ <= view_width_) {
		drag_state_ = (action == GLFW_PRESS);
		current_button_ = button;
		return ;
	} else if (current_x_ > window_width_ - 20 && current_x_ < window_width_) {
		
		scrolling = (action == GLFW_PRESS);
		
		scrolling = scrolling && current_button_ == GLFW_MOUSE_BUTTON_LEFT;
		// printf("chello %d\n", drag_scroll);
		current_button_ = button;
	} else{
		scrolling = false;
	}
	// FIXME: Key Frame Selection

	if (current_x_ > view_width_ && (current_x_ < view_width_ + preview_height_ *4/3) && action == GLFW_PRESS) {
		int prev = current_frame;
		current_frame = floor((view_height_ + frame_shift - current_y_)/ preview_height_);
		if (current_frame < 0 || current_frame >= (int)mesh_->keyframes.size()){
			current_frame = prev;
			return;
		} else {
			borderWall = true;
		}
}
}

void GUI::mouseScrollCallback(double dx, double dy)
{
	if (current_x_ < view_width_ || current_x_ > view_width_ + preview_height_ *4/3)
		return;
	// FIXME: Mouse Scrolling
	int max_shift = preview_height_ * mesh_->keyframes.size() - window_height_;
	if (frame_shift -35.0f * dy >= 0 ) {
		if(frame_shift <= max_shift){
			frame_shift -= 35.0f * dy;
		} else if (dy >= 0){
			frame_shift -= 35.0f * dy;
		}
		
	}  else {
		return;
}
}

void GUI::updateMatrices()
{
	// Compute our view, and projection matrices.
	if (fps_mode_)
		center_ = eye_ + camera_distance_ * look_;
	else
		eye_ = center_ - camera_distance_ * look_;

	view_matrix_ = glm::lookAt(eye_, center_, up_);
	light_position_ = glm::vec4(eye_, 1.0f);

	aspect_ = static_cast<float>(view_width_) / view_height_;
	projection_matrix_ =
		glm::perspective((float)(kFov * (M_PI / 180.0f)), aspect_, kNear, kFar);
	model_matrix_ = glm::mat4(1.0f);
}

MatrixPointers GUI::getMatrixPointers() const
{
	MatrixPointers ret;
	ret.projection = &projection_matrix_;
	ret.model= &model_matrix_;
	ret.view = &view_matrix_;
	return ret;
}

bool GUI::setCurrentBone(int i)
{
	if (i < 0 || i >= mesh_->getNumberOfBones())
		return false;
	current_bone_ = i;
	return true;
}

float GUI::getCurrentPlayTime() 
{
	// float diff_time = toc(&timer_);
	timer.stop();
	float rbag = timer.count<std::chrono::milliseconds>();
	// printf("the tale of two times, one tic and the other cxx %f\n", rbag);
	time_ += glm::pow(10.0, -2.7) * timer.count<std::chrono::milliseconds>();
	
	return time_;
}


bool GUI::captureWASDUPDOWN(int key, int action)
{
	if (key == GLFW_KEY_W) {
		if (fps_mode_)
			eye_ += zoom_speed_ * look_;
		else
			camera_distance_ -= zoom_speed_;
		return true;
	} else if (key == GLFW_KEY_S) {
		if (fps_mode_)
			eye_ -= zoom_speed_ * look_;
		else
			camera_distance_ += zoom_speed_;
		return true;
	} else if (key == GLFW_KEY_A) {
		if (fps_mode_)
			eye_ -= pan_speed_ * tangent_;
		else
			center_ -= pan_speed_ * tangent_;
		return true;
	} else if (key == GLFW_KEY_D) {
		if (fps_mode_)
			eye_ += pan_speed_ * tangent_;
		else
			center_ += pan_speed_ * tangent_;
		return true;
	} else if (key == GLFW_KEY_DOWN) {
		if (fps_mode_)
			eye_ -= pan_speed_ * up_;
		else
			center_ -= pan_speed_ * up_;
		return true;
	} else if (key == GLFW_KEY_UP) {
		if (fps_mode_)
			eye_ += pan_speed_ * up_;
		else
			center_ += pan_speed_ * up_;
		return true;
	}
	return false;
}


// Delegrate to the actual GUI object.
void GUI::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->keyCallback(key, scancode, action, mods);
}

void GUI::MousePosCallback(GLFWwindow* window, double mouse_x, double mouse_y)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->mousePosCallback(mouse_x, mouse_y);
}

void GUI::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->mouseButtonCallback(button, action, mods);
}

void GUI::MouseScrollCallback(GLFWwindow* window, double dx, double dy)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->mouseScrollCallback(dx, dy);
}
/*glm::vec3 da = r_End - eye_;
		glm::vec3 db = bone_end - bone_start;
		glm::vec3 dc = bone_start - eye_;
		//double s = dc.Cross(db).Dot(da.Cross(db)) / da.Cross(db).LengthSquared;
		float s = glm::dot(glm::cross(dc, db), glm::cross(da, db))/ glm::length2(glm::cross(da, db));
		float pl = glm::abs(glm::dot(dc, glm::cross(da, db)));
		if(s <= 1.0f && s >= 0.0f){
			// printf("%f sadasdasd\n", s);
			glm::vec3 inters = eye_ + s * da;
			//if ((intersection - segment.Start).LengthSquared + (intersection - segment.End).LengthSquared <= segment.LengthSquared + lengthErrorThreshold)
			if(glm::length2(inters - bone_start) + glm::length2(inters - bone_end) <= glm::length2(bone_end - bone_start) + 0.001){
				// std::cout << "point1 " << ": (" << inters.x << ", " << inters.y << ", " << inters.z << ")" << std::endl;
			}
		}
		
		// glm::vec3 t = glm::cross((bone_start - eye_), (bone_end -bone_start))/ glm::cross((r_End - eye_), (bone_end - bone_start));
		// glm::vec3 u = glm::cross((bone_start - eye_), (r_End - eye_))/ glm::cross((r_End - eye_), (bone_end - bone_start));
		// glm::vec3 crops = glm::cross((r_End - eye_), (bone_end - bone_start));
		glm::vec3 col1 = bone_start - eye_;
		glm::vec3 col2 = bone_end - bone_start;
		glm::vec3 col3 = glm::cross(r_End- eye_, col2);
		// float quee = glm::abs(glm::dot(col1, glm::cross(r_End - eye_, col2)));
		
		// float tree = (glm::dot(glm::cross(col1, col2) , glm::cross(r_End - eye_, col2))) / glm::length2(col3);
		// printf("%f sdfsdf\n", tree);
		glm::mat3 det = glm::mat3(col1, col2, col3);
		float deteri = glm::determinant(det);
		float denom = glm::length2(col3);
		float disto = std::numeric_limits<float>::max();
		float to = deteri/denom;
		glm::vec3 newCol2 = r_End - eye_;
		glm::mat3 det2 = glm::mat3(col1, newCol2, col3);
		float deurt = glm::determinant(det2);
		float so = deurt/denom;
		// printf("%f %f to and then so\n", to, so);
		// if(crops.x != 0  && crops.y != 0&& crops.z!= 0 ){

		// 	if(t.x >= 0 && t.x <= 1 &&t.y >= 0 && t.y <= 1 && t.z >= 0 && t.z <= 1 &&
		// 	u.x >= 0 && u.x <= 1 &&u.y >= 0 && u.y <= 1 && u.z >= 0 && u.z <= 1){
		// 	std::cout << "joint " << ": (" << t.x << ", " << t.y << ", " << t.z << ")" << std::endl;
		// 	glm::vec3 point1 = eye_ + t * r_Direction;
		// 	glm::vec3 point2 = bone_start + u * glm::normalize(bone_end - bone_start);
		// 	std::cout << "point1 " << ": (" << point1.x << ", " << point1.y << ", " << point1.z << ")" << std::endl;
		// 	std::cout << "point2" << ": (" << point2.x << ", " << point2.y << ", " << point2.z << ")" << std::endl;
		// 	}
			
		// }
		// printf("%f %f t and s vals are \n", to, so);
		
			// std::cout << "joint " << ": (" << t.x << ", " << t.y << ", " << t.z << ")" << std::endl;
			glm::vec3 point1 = eye_ + to * (r_End - eye_);
			glm::vec3 point2 = bone_start + so* (bone_end - bone_start);
			
			if(to <= 1.0 && to > 0.0 && glm::abs(point2.x - point1.x) <= 0.25 && glm::abs(point2.y - point1.y) <= 0.25 && glm::abs(point2.z - point1.z) <= 0.23){
			// std::cout << "point1 " << ": (" << point1.x << ", " << point1.y << ", " << point1.z << ")" << std::endl;
			// std::cout << "point2" << ": (" << point2.x << ", " << point2.y << ", " << point2.z << ")" << std::endl;
				disto = glm::distance(eye_, point1);
			}
		
		// std::cout << "joint " << ": (" << t.x << ", " << t.y << ", " << t.z << ")" << std::endl;
		
		// float curr_distance = line_segment_distance(eye_, click_ray_end, bone_start_position, bone_end_position);*/