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

Triangle::Triangle(Point* p1, Point* p2, Point* p3) {
	if(p1 != nullptr){
		points.push_back(p1);
		points.push_back(p2);
		points.push_back(p3);
	}
	
}


Triangle::~Triangle() {

}
Point::Point(glm::vec3 init_position, glm::vec3 curr_position, float mass, glm::vec2 uv_coords, int grid_x, int grid_z)
{
	init_position_ = init_position;
	position_ = curr_position;
	mass_ = mass;
	uv_coords_ = uv_coords;
	grid_x_ = grid_x;
	grid_z_ = grid_z;
	old_position_ = curr_position;
	force_ =  glm::vec3(0.0f, 0.0f, 0.0f);
	fixed_ = false;
	
}





Spring::Spring(Point* p1, Point* p2, float k):
			p1_(p1), p2_(p2), k_(k)
{
	init_length_ = glm::length(p1_->position_ - p2_->position_);
}



Spring::~Spring() 
{
	if(bend_spring_) {
		delete bend_spring_;
	}
}

BendSpring::BendSpring(Point* p1, Point* p2, float k):
			p1_(p1), p2_(p2), k_(k)
{
	init_length_ = glm::length(p1_->position_ - p2_->position_);
}



BendSpring::~BendSpring() 
{

	free;
	
}

void Spring::calcForce() {
	// this->fork = glm::vec3(0.0f, 0.0f,0.0f);
	float curr = glm::length(p1_->position_ - p2_->position_);

	//(glm::dot(p1_->velocity_ - p2_->velocity_, p1_->position_ - p2_->position_)/ curr) * 
	glm::vec3 normLen = (p1_->position_ - p2_->position_) / curr;
	float velPos = 0.60f * glm::dot(p1_->velocity_ - p2_->velocity_,p1_->position_ - p2_->position_) / curr;
	float restComp = 20.0f * (curr - glm::length(p1_->init_position_ - p2_->init_position_));
	this->fork = -((velPos + restComp) * normLen);
	

}

void BendSpring::calcForce() {
	// this->fork = glm::vec3(0.0f, 0.0f,0.0f);
	float curr = glm::length(p1_->position_ - p2_->position_);

	//(glm::dot(p1_->velocity_ - p2_->velocity_, p1_->position_ - p2_->position_)/ curr) * 
	glm::vec3 normLen = (p1_->position_ - p2_->position_) / curr;
	float velPos = 1.10f * glm::dot(p1_->velocity_ - p2_->velocity_,p1_->position_ - p2_->position_) / curr;
	float restComp = 30.0f * (curr - glm::length(p1_->init_position_ - p2_->init_position_));
	this->fork = -((velPos + restComp) * normLen);
	

}


void Cloth::resetCloth() {
	
	springs_.clear();
	triangles_.clear();
	delete this;
	free;
}


Cloth::Cloth(int x_size, int z_size):
		x_size_(x_size), z_size_(z_size)
{
	// build grid
	float total_x_width = glm::round((x_size_ - 1) );
	float total_z_width = glm::round((z_size_ - 1 ));
	for(int x = 0; x < x_size_; x++) {
		float z_offset = (x % 2 == 0)? 0.0 : (0.5 );
		for(int z = 0; z < z_size_; z++) {
			float pos_x = x , pos_z = z + z_offset;
			glm::vec3 position(pos_x, init_height_, pos_z);
			glm::vec2 uv_coords(pos_x / total_x_width, pos_z / total_z_width);
			Point* point = new Point(position, position, point_mass_, uv_coords, x, z);
			points.push_back(point);
		}
	}
	
	
	
	points[0 * z_size_ + 0]->fixed_ = true;	
	points[(x_size_ - 1) * z_size_]->fixed_ = true;	
	points[(x_size_ - 1)/2 * z_size_]->fixed_ = true;	

	for(int x = 0; x < x_size_; x++) {
		for(int z = 0; z < z_size_; z++) {
			Triangle* triangle = new Triangle(nullptr, nullptr, nullptr);
			Triangle* triangle2 = new Triangle(nullptr, nullptr, nullptr);
			int zee = 1;
			bool flip = true;
			bool oneD = false;
			bool twoD = false;
			if(x % 2 == 0) {
				zee = 0;
				flip = false;
			}
			// x >= 0 && x < x_size_ && z >= 0 && z < z_size_
				
				if((x >= 0 && x < x_size_ && (z + 1) >= 0 && (z + 1) < z_size_) && 
				((x + 1) >= 0 && (x + 1) < x_size_ && (z + zee) >= 0 && (z + zee) < z_size_)) {
					
					
					
					triangle->points.push_back(points[x * z_size_ + z]);
					triangle->points.push_back(points[x * z_size_ + z+1 ]);
					triangle->points.push_back(points[(x + 1) * z_size_ + (z + zee)]);
					oneD = true;
					
				}
				
				if((x >= 0 && x < x_size_ && (z + 1) >= 0 && (z + 1) < z_size_) &&
				( (x - 1) >= 0 && (x - 1) < x_size_ && (z + zee) >= 0 && (z + zee) < z_size_)) {
					triangle2->points.push_back(points[x * z_size_ + z]);
					triangle2->points.push_back(points[(x - 1) * z_size_ + (z + zee)]);
					triangle2->points.push_back(points[(x) * z_size_ + (z + 1)]);
					twoD = true;
				}
			
					if(flip){
						
						if(twoD ){
							triangles_.insert(triangle2);
						}
						if(oneD){
							triangles_.insert(triangle);
						}
						
						
					} else{
						
						if(oneD ){
							triangles_.insert(triangle);
						}
						if(twoD){
							triangles_.insert(triangle2);
						}
					}
		}
	}
	
	std::map<Point*, std::map<Point*, Spring*>> spring_map_;
	for(Triangle* triangle : triangles_) {
		// for(int idx = 0; idx < 3; idx++) {
			Point* p1 = triangle->points[0];
			Point* p2 = triangle->points[1];
			Point* p3 = triangle->points[2];
			if(!containsSpring(p1, p2)) {
				Spring* s = new Spring(p1, p2, struct_k_);
				
				springs_.insert(s);
				p1->springs_.insert(s);
				p2->springs_.insert(s);
				this->spring_map_[p1][p2] = s;
				this->spring_map_[p2][p1] = s;
				s->triangles_.push_back(triangle);
			}
			else {
				this->spring_map_[p1][p2]->triangles_.push_back(triangle);
			}
			if(!containsSpring(p2, p3)) {
				Spring* s = new Spring(p2, p3, struct_k_);
				
				springs_.insert(s);
				p2->springs_.insert(s);
				p3->springs_.insert(s);
				this->spring_map_[p2][p3] = s;
				this->spring_map_[p3][p2] = s;
				s->triangles_.push_back(triangle);
			}
			else {
				this->spring_map_[p2][p3]->triangles_.push_back(triangle);
			}
			if(!containsSpring(p3, p1)) {
				Spring* s = new Spring(p3, p1, struct_k_);
				
				springs_.insert(s);
				p3->springs_.insert(s);
				p1->springs_.insert(s);
				this->spring_map_[p3][p1] = s;
				this->spring_map_[p1][p3] = s;
				s->triangles_.push_back(triangle);
			}
			else {
				this->spring_map_[p3][p1]->triangles_.push_back(triangle);
			}
		// }
	}
	

	// add bending springs
	for(Spring* spring : springs_) {
		Point* p1 = spring->p2_;
		Point* p2 = spring->p1_;
		if(spring->p1_->grid_x_ < spring->p2_->grid_x_) {
			p1 = spring->p1_;
			p2 = spring->p2_;
		}
		

		// create bending springs
		int bendX1 = -1; 
		int bendZ1 = -1;
		int bendX2 = -1;
		int bendZ2 = -1;
		int val = -1;
		int shift = 0;
		// bool hift = false;
		if(p1->grid_x_ % 2 == 0) {
			val = -val;
		}
			if(p1->grid_x_ == p2->grid_x_) {
				shift = 1;
				bendZ1 = std::min(p1->grid_z_, p2->grid_z_);
				bendZ2 = std::min(p1->grid_z_, p2->grid_z_);
			}
			else if(p1->grid_z_ == p2->grid_z_) {
				bendZ1 = p1->grid_z_ == p2->grid_z_ ? p1->grid_z_ + val : p1->grid_z_ - val;
				bendZ2 = p1->grid_z_ == p2->grid_z_? p2->grid_z_ - val : p2->grid_z_ + val;
				
			}
			bendX1 = p2->grid_x_ - shift; 
			bendX2 = p1->grid_x_ + shift; 
			
		
		

		 
		 
		if((bendX1 >= 0 && bendX1 < x_size_ && bendZ1 >= 0 && bendZ1 < z_size_) && 
		(bendX2 >= 0 && bendX2 < x_size_ && bendZ2 >= 0 && bendZ2 < z_size_)) {
			
			BendSpring* bend_spring = new BendSpring(points[bendX1 * z_size_ + bendZ1], 
										points[bendX2 * z_size_ + bendZ2],
										bend_sheer_k_);
			spring->bend_spring_ = bend_spring;
		}

	}

	

	// update cache vertices
	refreshCache();

}


Cloth::~Cloth() {

}

void Cloth::refreshCache() {
	// vertices and uv_coords
	vertices.clear();
	cloth_uv_coords.clear();
	vertex_normals.clear();
	for(Point* p : points) {
				float x = 0.0f;
				float y = 0.0f;
				float z = 0.0f;
				for(glm::vec3& face_normal : p->face_normals_) {
					x += face_normal[0];
					y += face_normal[1];
					z += face_normal[2];
			}
			float ratio = ((float)p->face_normals_.size());
			p->vertex_normal_ = glm::vec3(x/ratio, y/ratio, z/ratio ) ;
	}

	// spring linemesh
	struct_spring_vertices.clear();
	for(Spring* s : springs_) {
		struct_spring_vertices.push_back(s->p1_->position_);
		struct_spring_vertices.push_back(s->p2_->position_);
	}
	for(Triangle* triangle : triangles_) {
		for(Point* p : triangle->points) {
			
			vertices.push_back(p->position_);
			cloth_uv_coords.push_back(p->uv_coords_);
			
			vertex_normals.push_back(p->vertex_normal_);
		}
	}

	

}

void Cloth::tear(Spring* s) {
	// printf("%f this springs one points pos is \n", s->p1_->position_[0]);
	

	Triangle *t1 = nullptr, *t2 = nullptr;	
	if(s->triangles_.size() > 1) {
		t1 = s->triangles_[0];
		t2 = s->triangles_[1];
	}	
	else if(s->triangles_.size() == 1) {
		t1 = s->triangles_[0];
	}
	Point *p1 = s->p1_;
	Point *p2 = s->p2_;	
	

	glm::vec3 pp1_curr_pos = p1->position_ + ((p1->position_ + p2->position_) / 2.0f - p1->position_) * 0.20f;
	Point* pp1 = new Point(p1->init_position_, pp1_curr_pos, p1->mass_ , p1->uv_coords_, p1->grid_x_, p1->grid_z_);
	glm::vec3 pp2_curr_pos = p2->position_ + ((p1->position_ + p2->position_) / 2.0f - p2->position_) * 0.20f;
	Point* pp2 = new Point(p2->init_position_, pp2_curr_pos, p2->mass_ , p2->uv_coords_, p2->grid_x_, p2->grid_z_);
	points.push_back(pp1);
	points.push_back(pp2);
	Spring* tempSpring1 = new Spring(p1, pp1, struct_k_);
	this->spring_map_[p1][pp1] = tempSpring1;
	this->spring_map_[pp1][p1] = tempSpring1;
	p1->springs_.insert(tempSpring1);
	pp1->springs_.insert(tempSpring1);

	springs_.insert(tempSpring1);
	
	Spring* tempSpring2 = new Spring(p2, pp2, struct_k_);
	this->spring_map_[p2][pp2] = tempSpring2;
	this->spring_map_[pp2][p2] = tempSpring2;
	p2->springs_.insert(tempSpring2);
	pp2->springs_.insert(tempSpring2);

	springs_.insert(tempSpring2);

	for(int x = 0; x < 2; x++){

		if(t1) {
			Point* nb_p1 = nullptr;
			for(Point* p : t1->points) {
				if(p != s->p1_ && p != s->p2_) {
				nb_p1=  p;
				}
			}
			// if(nb_p1 == nullptr){
			// 	break;
			// 	// printf("PROCESSEDIASDFOIANDFPIENF\n");
			// }
			Triangle* temp1 = new Triangle(nb_p1, p1, pp1);
			Triangle* temp2 = new Triangle(nb_p1, pp2, p2);

			

			tempSpring1->triangles_.push_back(temp1);
			tempSpring2->triangles_.push_back(temp2);

			Spring* tempSpring11 = new Spring(pp1, nb_p1, struct_k_);
			this->spring_map_[pp1][nb_p1] = tempSpring11;
			this->spring_map_[nb_p1][pp1] = tempSpring11;
			pp1->springs_.insert(tempSpring11);
			nb_p1->springs_.insert(tempSpring11);

			springs_.insert(tempSpring11);
	
			Spring* tempSpring12 =  new Spring(pp2, nb_p1, struct_k_);
			this->spring_map_[pp2][ nb_p1] = tempSpring12 ;
			this->spring_map_[ nb_p1][pp2] = tempSpring12 ;
			pp2->springs_.insert(tempSpring12 );
			nb_p1->springs_.insert(tempSpring12 );

			springs_.insert(tempSpring12 );
			tempSpring11->triangles_.push_back(temp1);
			tempSpring12->triangles_.push_back(temp2);
			
			
			for(int i = 0; i < this->spring_map_[p1][nb_p1]->triangles_.size(); i++) {
				if(this->spring_map_[p1][nb_p1]->triangles_[i] == t1) {
					this->spring_map_[p1][nb_p1]->triangles_[i] = temp1;
					break;
				}
			}
		
			
			for(int i = 0; i < this->spring_map_[p2][nb_p1]->triangles_.size(); i++) {
				if(this->spring_map_[p2][nb_p1]->triangles_[i] == t1) {
					this->spring_map_[p2][nb_p1]->triangles_[i] = temp2;
					break;
					}
				}
			
			triangles_.insert(temp1);
			triangles_.insert(temp2);
			for(Spring* nb_p1_s : nb_p1->springs_) {
				if(nb_p1_s->bend_spring_ != nullptr) {
					delete nb_p1_s->bend_spring_;
					nb_p1_s->bend_spring_ = nullptr;
				}

			}
			triangles_.erase(t1);
			// printf("PROCESSEDIASDdsadafsdfsddsgdfgdfgdfgdfgfgFOIANDFPIENF2\n");
		}
		// printf("PROCESSEDIAsdfsdfsdfsjdnfksjdnfksdjfnksdjfnjsdkfjsdSDdsadafsdfsddsgdfgdfgdfgdfgfgFOIANDFPIENF3\n");
		t1 = t2;
	}
	delete t1;
	s->fork = glm::vec3(0.0f, 0.0f,0.0f);
	s->p1_->force_ = glm::vec3(0.0f, 0.0f, 0.0f);
	s->p2_->force_ = glm::vec3(0.0f, 0.0f, 0.0f);
	s->p1_->springs_.erase(s);
	s->p2_->springs_.erase(s);
	springs_.erase(s);

	this->spring_map_[s->p1_][s->p2_] = nullptr;
	this->spring_map_[s->p2_][s->p1_] = nullptr;
	// delete s;
	
}






void Cloth::animate(float delta_t) {

	
	for(auto itr = points.begin(); itr != points.end(); itr++) {
		std::map<int, std::unordered_set<Point*>> point_groups;
		groupNeighbors(*itr, point_groups);
		if(point_groups.size() > 1){
			duplicate(*itr, point_groups);
		}
		
	}

	for(Point* point : points) {
		point->force_ =  glm::vec3(0.0f, - 1.0 * point->mass_ * G, 0.0f);
	}


	// update forces
	for(Spring* struct_s : this->springs_) {
		struct_s->calcForce();
		
		struct_s->fork[0] = glm::clamp(struct_s->fork[0], -300.0f, 300.0f);
		struct_s->fork[1] = glm::clamp(struct_s->fork[1], -300.0f, 300.0f);
		struct_s->fork[2] = glm::clamp(struct_s->fork[2], -300.0f, 300.0f);
		struct_s->p1_->force_ += struct_s->fork;
		struct_s->p2_->force_ += -struct_s->fork;
		if(std::fabs(struct_s->p1_->force_[0]) >=300 || std::fabs(struct_s->p1_->force_[1]) >=300 || std::fabs(struct_s->p1_->force_[2]) >=300){

		}
		if(struct_s->force_quantity_ >= std::abs(100.0f)) {
			// struct_s->force_quantity_ = 5.0f;
			// printf("typical forces are %f\n", struct_s->force_quantity_);
			// std::cout << "spring force quantity zero" << std::endl;
		}

		if(struct_s->bend_spring_) {
			struct_s->bend_spring_->calcForce();
			// struct_s->bend_spring_->applyForce();
			struct_s->bend_spring_->fork[0] = glm::clamp(struct_s->bend_spring_->fork[0], -300.0f, 300.0f);
			struct_s->bend_spring_->fork[1] = glm::clamp(struct_s->bend_spring_->fork[1], -300.0f, 300.0f);
			struct_s->bend_spring_->fork[2] = glm::clamp(struct_s->bend_spring_->fork[2], -300.0f, 300.0f);
			struct_s->bend_spring_->p1_->force_ += struct_s->bend_spring_->fork;
			struct_s->bend_spring_->p2_->force_ += -struct_s->bend_spring_->fork;
			// printf("the power in this spring is %f %f %f\n", struct_s->bend_spring_->fork[0], struct_s->bend_spring_->fork[1], struct_s->bend_spring_->fork[2]);
		}
	}


	// update Point velocity and positions
	for(Point* point : points) {
		
		if(!point->fixed_) {

			glm::vec3 acceleration = (point->force_ + (-0.20f * point->velocity_) ) / point->mass_;
			point->velocity_ += acceleration * delta_t  ;
			point->position_[0] += point->velocity_[0] * delta_t;
			point->position_[1] += point->velocity_[1] * delta_t;
			point->position_[2] += point->velocity_[2] * delta_t;
			

		}
	}

	std::queue<Point*> start_points;
	
	for(Point* p : points) {
		if(p->fixed_) {
			start_points.emplace(p);
		}
		if(!(*p->springs_.begin())->constrained_) {
			start_points = std::queue<Point*>();
			start_points.emplace(p);
		}
	}

	for(Point* p : points) {
		if(p->position_.y < kFloorY){
			p->position_.y = kFloorY + kFloorEps;
			p->fixed_ = true;	
		}
	}
	
	
	for(Triangle* t : triangles_) {
		glm::vec3 firstDiff = t->points[2]->position_ - t->points[1]->position_;
		glm::vec3 secDiff = t->points[1]->position_ - t->points[0]->position_;
		glm::vec3 unNormalNormal = glm::cross(firstDiff, secDiff);
		t->face_normal_ = glm::normalize(unNormalNormal);
		t->points[0]->face_normals_.clear();
		t->points[1]->face_normals_.clear();
		t->points[2]->face_normals_.clear();
		t->points[0]->face_normals_.push_back(t->face_normal_);
		t->points[1]->face_normals_.push_back(t->face_normal_);
		t->points[2]->face_normals_.push_back(t->face_normal_);
		
	}
	

	// setCurrentPoint();
	picked_spring_ = nullptr;
	setCurrentSpring();
	
	if(picked_spring_ ) {
		if(to_tear ){//&& !picked_spring_->is_secondary_) {
			tear(picked_spring_);
		}
	}

	// for(Spring* struct_s : this->springs_) {
	// 	if(glm::length(struct_s->p1_->position_ -  struct_s->p2_->position_) > 5.0f){
	// 		tear(struct_s);
	// 	}
	// }
	refreshCache();
	time_ += delta_t;
}




void Cloth::groupNeighbors(Point* p, std::map<int, std::unordered_set<Point*>>& groups) {
	std::vector<int> uf;
	std::vector<Point*> neighbors;
	for(Spring* s : p->springs_) {
		if(s->p1_ == p) {
			neighbors.push_back(s->p2_);
		}
		else if(s->p2_ == p) {
			neighbors.push_back(s->p1_);
		}
		
	}
	std::vector<Spring*> sprinbors;
	uf.resize(neighbors.size());
	for(int i = 0; i < uf.size(); i++) {
		uf[i] = i;
	}
	for(int i = 0; i < neighbors.size(); i++) {
		Point *p1 = neighbors[i];
		for(int j = i + 1; j < neighbors.size(); j++) {
			Point *p2 = neighbors[j];
			if(containsSpring(p1, p2)) {
				int temp = j;
				while(temp != uf[temp]) {
					temp = uf[temp];
				}
				int temp2 = i;
				while(temp2 != uf[temp2]) {
					temp2 = uf[temp2];
				}
				uf[temp] = temp2;
				sprinbors.push_back(spring_map_[p1][p2]);
			}
		}
	}

	// for(int i = 0; i < uf.size(); i++) {
	// 	if(uf[i] == i) {
	// 		groups[i] = std::unordered_set<Point*>();
	// 	}
	// }	

	for(int i = 0; i < uf.size(); i++) {
		if(containsSpring(p, neighbors[i])) {
			int temp = i;
			while(temp != uf[temp]) {
				temp = uf[temp];
			}
			int group_num = temp;
			// if(groups[group_num] == nullptr){
			// 	groups[group_num] = std::unordered_set<Point*>();
			// }
			
			groups[group_num].insert(neighbors[i]);
		}
		
		
	}
	if(groups.size() > 1) {
		for(Spring* nb_s : sprinbors) {
			if(nb_s->bend_spring_ != nullptr) {
				delete nb_s->bend_spring_;
				nb_s->bend_spring_ = nullptr;
			}
			
		}
	}
}



void Cloth::duplicate(Point* p, std::map<int, std::unordered_set<Point*>>& groups) {
	
	std::vector<Point*> new_points;
	int group_count = groups.size(); 
	for(auto const& group : groups) {
		const std::unordered_set<Point*>& group_points = group.second;
		if(group_count == 1) {
			
			break;	
		}
		group_count--;
		
		
		Point* p_copy = new Point(p->init_position_, p->position_, p->mass_, p->uv_coords_,  p->grid_x_, p->grid_z_);
		p_copy->force_ = p->force_;
		p_copy->velocity_ =p->velocity_;
		p_copy->fixed_ = p->fixed_;
		p_copy->old_position_ = p->old_position_;
		new_points.push_back(p_copy);
		for(Point* neigbor : group_points) {
			Spring* s = this->spring_map_[p][neigbor];
			p->springs_.erase(s);
			this->spring_map_[p][neigbor] = nullptr;
			this->spring_map_[neigbor][p] = nullptr;

		
			if(s->p1_ == p) {
				s->p1_ = p_copy;
			}
			else if(s->p2_ == p) {
				s->p2_ = p_copy;
			}
			p_copy->springs_.insert(s);
			this->spring_map_[p_copy][neigbor] = s;
			this->spring_map_[neigbor][p_copy] = s;
			// printf("galfsdf2\n");

			for(Triangle* t : s->triangles_) {	
				
					if(t->points[0] == p) {
						t->points[0] = p_copy;
					}
					if(t->points[1] == p) {
						t->points[1] = p_copy;
					}
					if(t->points[2] == p) {
						t->points[2] = p_copy;
					}
				
			}

		}
		
	}
	for(Point* one : new_points) {
		points.push_back(one);
	}

	// printf("galfsdf3\n");
}




void Cloth::setCurrentSpring() {
	// picked_spring_ = nullptr;
	float min_distance = std::numeric_limits<float>::max();
	for(Spring* s : springs_) {	
		float curr_distance = line_segment_distance_copy(pick_ray_start, pick_ray_end, s->p1_->position_, s->p2_->position_);
		if(curr_distance < SPRING_CYLINDER_RADIUS) {
			min_distance = std::min(curr_distance, min_distance);
			if(s != nullptr){
				picked_spring_ = s;
			}
		}
	}
}





bool Cloth::containsSpring(Point* p1, Point* p2) {
	if((spring_map_[p1][p2] != nullptr) || (spring_map_[p2][p1] != nullptr)){
		return true;
	}
	return false;
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
