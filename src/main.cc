#include <GL/glew.h>

#include "cloth_geometry.h"
#include "procedure_geometry.h"
#include "render_pass.h"
#include "tictoc.h"
#include "config.h"
#include "gui.h"
#include "texture_to_render.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/io.hpp>
#include <debuggl.h>



int window_width = 1280;
int window_height = 720;
int main_view_width = 960;
int main_view_height = 720;
int preview_width = window_width - main_view_width - 20;// 320
int preview_height = preview_width / 4 * 3; // 320 / 4 * 3 = 240
int preview_bar_width = preview_width;
int preview_bar_height = main_view_height;

const char* cmd = "ffmpeg -r 30 -f rawvideo -pix_fmt rgba -s 960x720 -i - "
                  "-threads 0 -preset fast -y -pix_fmt yuv420p -crf 21 -vf vflip output.mp4";

FILE* file_open;
bool if_file_open = false;
int* v_buffer = new int[main_view_width * main_view_height];
const std::string window_title = "Animation";

const char* vertex_shader =
#include "shaders/default.vert"
;

const char* blending_shader =
#include "shaders/blending.vert"
;

const char* geometry_shader =
#include "shaders/default.geom"
;

const char* fragment_shader =
#include "shaders/default.frag"
;

const char* floor_fragment_shader =
#include "shaders/floor.frag"
;

const char* bone_vertex_shader =
#include "shaders/bone.vert"
;

const char* bone_fragment_shader =
#include "shaders/bone.frag"
;

// FIXME: Add more shaders here.
const char* cylinder_vertex_shader = 
#include "shaders/cylinder.vert"
;

const char* cylinder_fragment_shader = 
#include "shaders/cylinder.frag"
;

const char* preview_vertex_shader = 
#include "shaders/preview.vert"
;

const char* preview_fragment_shader = 
#include "shaders/preview.frag"
;

const char* scroll_vertex_shader = 
#include "shaders/scroll.vert"
;

const char* scroll_fragment_shader  = 
#include "shaders/scroll.frag"
;

const char* cloth_vertex_shader =
#include "shaders/cloth.vert"
;

const char* cloth_fragment_shader =
#include "shaders/cloth.frag"
;



void ErrorCallback(int error, const char* description) {
	std::cerr << "GLFW Error: " << description << "\n";
}

GLFWwindow* init_glefw()
{
	if (!glfwInit())
		exit(EXIT_FAILURE);
	glfwSetErrorCallback(ErrorCallback);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	
	glfwWindowHint(GLFW_SAMPLES, 4);
	auto ret = glfwCreateWindow(window_width, window_height, window_title.data(), nullptr, nullptr);
	CHECK_SUCCESS(ret != nullptr);
	glfwMakeContextCurrent(ret);
	glewExperimental = GL_TRUE;
	CHECK_SUCCESS(glewInit() == GLEW_OK);
	glGetError();  // clear GLEW's error for it
	glfwSwapInterval(1);
	const GLubyte* renderer = glGetString(GL_RENDERER);  // get renderer string
	const GLubyte* version = glGetString(GL_VERSION);    // version as a string
	std::cout << "Renderer: " << renderer << "\n";
	std::cout << "OpenGL version supported:" << version << "\n";

	return ret;
}

int main(int argc, char* argv[])
{
	// if (argc < 2) {
	// 	std::cerr << "Input model file is missing" << std::endl;
	// 	std::cerr << "Usage: " << argv[0] << " <PMD file>" << std::endl;
	// 	return -1;
	// }

	
	GLFWwindow *window = init_glefw();
	GUI gui(window, main_view_width, main_view_height, preview_height);
	int scroll_bar_width = 20;
	std::vector<glm::vec4> floor_vertices;
	std::vector<glm::uvec3> floor_faces;
	create_floor(floor_vertices, floor_faces);
	int cloth_x_size = 21;
	int cloth_z_size = 21;
	Cloth cloth(cloth_x_size, cloth_z_size);
	MassSpringSystem ms_system(cloth_x_size, cloth_z_size);
	TicTocTimer *timer = new TicTocTimer;
	*timer = tic();
	std::vector<glm::vec4> prev_vertices;
	std::vector<glm::uvec3> prev_faces;
	std::vector<glm::vec2> prev_coords;
	create_prev(prev_vertices, prev_faces, prev_coords);
	LineMesh cylinder_mesh;
	LineMesh axes_mesh;

	std::vector<glm::vec4> scroll_vertices;
	std::vector<glm::uvec3> scroll_faces;
	std::vector<glm::vec2> scroll_coords;
	create_quad(scroll_vertices, scroll_faces, scroll_coords);

		// preview uniforms
	glm::mat4 orthomat(1.0);
	float frame_shift;
	int sampler;
	int show_border = 1;

	

	// FIXME: we already created meshes for cylinders. Use them to render
	//        the cylinder and axes if required by the assignment.
	// create_cylinder_mesh(cylinder_mesh);
	// create_axes_mesh(axes_mesh);

	Mesh mesh;
	// mesh.loadPmd(argv[1]);
	// if(argc >= 3){
		
	// 	mesh.loadAnimationFrom(argv[2]);
	// 	mesh.preLoaded = true;
	// }
	// std::cout << "Loaded object  with  " << mesh.vertices.size()
	// 	<< " vertices and " << mesh.faces.size() << " faces.\n";

	glm::vec4 mesh_center = glm::vec4(0.0f);
	for (size_t i = 0; i < mesh.vertices.size(); ++i) {
		mesh_center += mesh.vertices[i];
	}
	mesh_center /= mesh.vertices.size();
	// TextureToRender
	std::vector<TextureToRender*>& textures = mesh.textures;
	/*
	 * GUI object needs the mesh object for bone manipulation.
	 */
	gui.assignMesh(&mesh);

	glm::vec4 light_position = glm::vec4(0.0f, 100.0f, 0.0f, 1.0f);
	MatrixPointers mats; // Define MatrixPointers here for lambda to capture

	/*
	 * In the following we are going to define several lambda functions as
	 * the data source of GLSL uniforms
	 *
	 * Introduction about lambda functions:
	 *      http://en.cppreference.com/w/cpp/language/lambda
	 *      http://www.stroustrup.com/C++11FAQ.html#lambda
	 *
	 * Note: lambda expressions cannot be converted to std::function directly
	 *       Hence we need to declare the data function explicitly.
	 *
	 * CAVEAT: DO NOT RETURN const T&, which compiles but causes
	 *         segfaults.
	 *
	 * Do not worry about the efficient issue, copy elision in C++ 17 will
	 * minimize the performance impact.
	 *
	 * More details about copy elision:
	 *      https://en.cppreference.com/w/cpp/language/copy_elision
	 */

	// FIXME: add more lambdas for data_source if you want to use RenderPass.
	//        Otherwise, do whatever you like here
	std::function<const glm::mat4*()> model_data = [&mats]() {
		return mats.model;
	};
	std::function<glm::mat4()> view_data = [&mats]() { return *mats.view; };
	std::function<glm::mat4()> proj_data = [&mats]() { return *mats.projection; };
	std::function<glm::mat4()> identity_mat = [](){ return glm::mat4(1.0f); };
	std::function<glm::vec3()> cam_data = [&gui](){ return gui.getCamera(); };
	std::function<glm::vec4()> lp_data = [&light_position]() { return light_position; };
	float radius = kCylinderRadius;
	std::function<float()> cyl_data = [&radius]() { return radius ;};
	std::function<bool()> show_border_data = [&show_border] () {return show_border;};
	std::function<glm::mat4()> orthomat_data = [&orthomat]() { return orthomat; };
	frame_shift = gui.frame_shift;
	std::function<float()> frame_shift_data = [&gui]() { return gui.frame_shift ;};
	std::function<int()> sampler_data = [&sampler]() { return sampler;};
	std::function<int()> goof_data = [&sampler]() { return 0;};
	glm::mat4 identity_model_mat(1.0);
	std::function<glm::mat4()> identity_model_data = [&identity_model_mat]() {return identity_model_mat;};
	int total_preview_num = mesh.textures.size();
	
	std::function<int()> total_preview_num_data =  [&mesh]()  {  return mesh.textures.size();};
	auto std_model = std::make_shared<ShaderUniform<const glm::mat4*>>("model", model_data);
	
	
	auto floor_model = make_uniform("model", identity_mat);
	auto std_view = make_uniform("view", view_data);
	auto std_camera = make_uniform("camera_position", cam_data);
	auto std_proj = make_uniform("projection", proj_data);
	auto std_light = make_uniform("light_position", lp_data);
	auto cylinder_radius = make_uniform("cylinder_radius", cyl_data);
	auto show_borderu = make_uniform("show_border", show_border_data);
	auto sampleru = make_texture("sampler", goof_data, 0, sampler_data);
	auto orthomatu = make_uniform("orthomat", orthomat_data);
	auto frame_shiftu = make_uniform("frame_shift", frame_shift_data);
	auto preview_uniform = make_uniform("total_preview_num", total_preview_num_data);
	auto identity_model = make_uniform("model", identity_model_data );
	glm::mat4 bone_transform_matrix;
	
	std::function<glm::mat4()> bone_transform_data = [&mesh, &bone_transform_matrix, &gui]() {
		int current_bone = gui.getCurrentBone();
		if(current_bone != -1) {
			// bone_transform_matrix = mesh.skeleton.doBoneTransform(current_bone);
			return mesh.getBoneTransformMatrix(current_bone);	
		}
		return glm::mat4(1.0f);
		};
	auto bone_transform = make_uniform("bone_transform", bone_transform_data);
	std::function<float()> alpha_data = [&gui]() {
		static const float transparet = 0.5; // Alpha constant goes here
		static const float non_transparet = 1.0;
		if (gui.isTransparent())
			return transparet;
		else
			return non_transparet;
	};
	auto object_alpha = make_uniform("alpha", alpha_data);

	std::function<std::vector<glm::vec3>()> trans_data = [&mesh](){ return mesh.getCurrentQ()->transData(); };
	std::function<std::vector<glm::fquat>()> rot_data = [&mesh](){ return mesh.getCurrentQ()->rotData(); };
	auto joint_trans = make_uniform("joint_trans", trans_data);
	auto joint_rot = make_uniform("joint_rot", rot_data);
	// FIXME: define more ShaderUniforms for RenderPass if you want to use it.
	//        Otherwise, do whatever you like here

	// Floor render pass
	RenderDataInput floor_pass_input;
	floor_pass_input.assign(0, "vertex_position", floor_vertices.data(), floor_vertices.size(), 4, GL_FLOAT);
	floor_pass_input.assignIndex(floor_faces.data(), floor_faces.size(), 3);
	RenderPass floor_pass(-1,
			floor_pass_input,
			{ vertex_shader, geometry_shader, floor_fragment_shader},
			{ floor_model, std_view, std_proj, std_light },
			{ "fragment_color" }
			);

	// PMD Model render pass
	// FIXME: initialize the input data at Mesh::loadPmd
	std::vector<glm::vec2>& uv_coordinates = mesh.uv_coordinates;
	RenderDataInput object_pass_input;
	object_pass_input.assign(0, "jid0", mesh.joint0.data(), mesh.joint0.size(), 1, GL_INT);
	object_pass_input.assign(1, "jid1", mesh.joint1.data(), mesh.joint1.size(), 1, GL_INT);
	object_pass_input.assign(2, "w0", mesh.weight_for_joint0.data(), mesh.weight_for_joint0.size(), 1, GL_FLOAT);
	object_pass_input.assign(3, "vector_from_joint0", mesh.vector_from_joint0.data(), mesh.vector_from_joint0.size(), 3, GL_FLOAT);
	object_pass_input.assign(4, "vector_from_joint1", mesh.vector_from_joint1.data(), mesh.vector_from_joint1.size(), 3, GL_FLOAT);
	object_pass_input.assign(5, "normal", mesh.vertex_normals.data(), mesh.vertex_normals.size(), 4, GL_FLOAT);
	object_pass_input.assign(6, "uv", uv_coordinates.data(), uv_coordinates.size(), 2, GL_FLOAT);
	// TIPS: You won't need vertex position in your solution.
	//       This only serves the stub shader.
	object_pass_input.assign(7, "vert", mesh.vertices.data(), mesh.vertices.size(), 4, GL_FLOAT);
	object_pass_input.assignIndex(mesh.faces.data(), mesh.faces.size(), 3);
	object_pass_input.useMaterials(mesh.materials);
	RenderPass object_pass(-1,
			object_pass_input,
			{
			  blending_shader,
			  geometry_shader,
			  fragment_shader
			},
			{ std_model, std_view, std_proj,
			  std_light,
			  std_camera, object_alpha,
			  joint_trans, joint_rot
			},
			{ "fragment_color" }
			);

	// Setup the render pass for drawing bones
	// FIXME: You won't see the bones until Skeleton::joints were properly
	//        initialized
	std::vector<int> bone_vertex_id;
	std::vector<glm::uvec2> bone_indices;
	for (int i = 0; i < (int)mesh.skeleton.joints.size(); i++) {
		bone_vertex_id.emplace_back(i);
	}

	RenderDataInput cylinder_pass_input;
	cylinder_pass_input.assign(0, "vertex_position", cylinder_mesh.vertices.data(), cylinder_mesh.vertices.size(), 4, GL_FLOAT);
	cylinder_pass_input.assignIndex(cylinder_mesh.indices.data(), cylinder_mesh.indices.size(), 2);
	RenderPass cylinder_pass(-1, cylinder_pass_input,
			{ cylinder_vertex_shader, nullptr, cylinder_fragment_shader },
			{ std_model, std_view, std_proj, bone_transform, cylinder_radius },
			{ "fragment_color" }
	);

	for (const auto& joint: mesh.skeleton.joints) {
		if (joint.parent_index < 0)
			continue;
		bone_indices.emplace_back(joint.joint_index, joint.parent_index);
	}
	RenderDataInput bone_pass_input;
	bone_pass_input.assign(0, "jid", bone_vertex_id.data(), bone_vertex_id.size(), 1, GL_UNSIGNED_INT);
	bone_pass_input.assignIndex(bone_indices.data(), bone_indices.size(), 2);
	RenderPass bone_pass(-1, bone_pass_input,
			{ bone_vertex_shader, nullptr, bone_fragment_shader},
			{ std_model, std_view, std_proj, joint_trans },
			{ "fragment_color" }
			);

	// FIXME: Create the RenderPass objects for bones here.
	//        Otherwise do whatever you like.

	
	RenderDataInput tri_cloth_pass_input;
	tri_cloth_pass_input.assign(0, "vertex_position", cloth.vertices.data(), cloth.vertices.size(), 3, GL_FLOAT);
	tri_cloth_pass_input.assign(1, "uv", cloth.cloth_uv_coords.data(), cloth.cloth_uv_coords.size(), 2, GL_FLOAT);

	RenderPass tri_cloth_pass(-1,
			tri_cloth_pass_input,
			{ cloth_vertex_shader, nullptr, cloth_fragment_shader },
			{ std_model, std_view, std_proj, std_light },
			{ "fragment_color" }
			);

	RenderDataInput preview_pass_input;
	preview_pass_input.assign(0, "vertex_position", prev_vertices.data(), prev_vertices.size(), 4, GL_FLOAT);
	preview_pass_input.assign(1, "tex_coord_in", prev_coords.data(), prev_coords.size(), 2, GL_FLOAT);
	preview_pass_input.assignIndex(prev_faces.data(), prev_faces.size(), 3);
	RenderPass preview_pass(-1, preview_pass_input,
			{preview_vertex_shader, nullptr, preview_fragment_shader},
			{orthomatu, frame_shiftu, sampleru, show_borderu},
			{"fragment_color"}
			);


	RenderDataInput scroll_pass_input;
	scroll_pass_input.assign(0, "vertex_position", scroll_vertices.data(), scroll_vertices.size(), 4, GL_FLOAT);
	scroll_pass_input.assign(1, "tex_coord_in", scroll_coords.data(), scroll_coords.size(), 2, GL_FLOAT);



	scroll_pass_input.assignIndex(scroll_faces.data(), scroll_faces.size(), 3);


		RenderDataInput depth_pass_input2;
	
	depth_pass_input2.assign(0, "vertex_position", mesh.vertices.data(), mesh.vertices.size(), 4, GL_FLOAT);
	depth_pass_input2.assignIndex(mesh.faces.data(), mesh.faces.size(), 3);
	// RenderPass depth_pass2(-1, depth_pass_input2,
	// 		{ depth_vertex_shader, nullptr, depth_fragment_shader},
	// 		{ std_model, std_view, std_proj, std_light},
	// 		{ "fragment_color"  }
	// 		);

	RenderPass scroll_pass(-1, scroll_pass_input,
			{scroll_vertex_shader, nullptr, scroll_fragment_shader},
			{frame_shiftu, preview_uniform},
			{"fragment_color"}
			);


	
	RenderDataInput shad_pass_input;
	shad_pass_input.assign(0, "vertex_position", floor_vertices.data(), floor_vertices.size(), 4, GL_FLOAT);
	// shad_pass_input.assign(1, "tex_coord_in", shad_coords.data(), shad_coords.size(), 2, GL_FLOAT);
	// // shad_pass_input.assign(1, "tex_coord_in", shad_coords.data(), shad_coords.size(), 2, GL_FLOAT);
	// shad_pass_input.assignIndex(shad_faces.data(), shad_faces.size(), 3);
	// RenderPass shad_pass(-1, shad_pass_input,
	// 		{shad_vertex_shader, nullptr, shad_fragment_shader},
	// 		{std_model, std_view, std_proj, sampleru},
	// 		{"color"}
	// 		);

	RenderDataInput cloth_pass_input;
	cloth_pass_input.assign(0, "vertex_position", ms_system.node_positions.data(), ms_system.node_positions.size(), 3, GL_FLOAT);
	cloth_pass_input.assignIndex(ms_system.line_indices.data(), ms_system.line_indices.size(), 2);

	RenderPass cloth_pass(-1,
			cloth_pass_input,
			{ cloth_vertex_shader, nullptr, cloth_fragment_shader },
			{ std_model, std_view, std_proj, std_light },
			{ "fragment_color" }
			);

	bool draw_floor = true;
	bool draw_cloth = false;
	bool draw_tri_cloth = true;
	float aspect = 0.0f;
	std::cout << "center = " << mesh.getCenter() << "\n";

	// bool draw_floor = true;
	bool draw_skeleton = true;
	bool draw_object = true;
	bool draw_cylinder = true;
	toc(timer);
	while (!glfwWindowShouldClose(window)) {
		// Setup some basic window stuff.
		if(gui.upSample){
		glfwWindowHint(GLFW_SAMPLES, 64);
	} else{
		glfwWindowHint(GLFW_SAMPLES, 4);
	}
	
		glfwGetFramebufferSize(window, &window_width, &window_height);
		glViewport(0, 0, main_view_width, main_view_height);
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_MULTISAMPLE);
		glEnable(GL_BLEND);
		glEnable(GL_CULL_FACE);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDepthFunc(GL_LESS);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glCullFace(GL_BACK);

		gui.updateMatrices();
		mats = gui.getMatrixPointers();
#if 0
		std::cerr << model_data() << '\n';
		std::cerr << "call from outside: " << std_model->data_source() << "\n";
		std_model->bind(0);
#endif

		if (gui.isPlaying()) {
			std::stringstream title;
			float cur_time = gui.getCurrentPlayTime();
			// printf("cur time is %f\n", cur_time);
			title << window_title << " Playing: "
			      << std::setprecision(2)
			      << std::setfill('0') << std::setw(6)
			      << cur_time << " sec";
			// printf("%f \n", cur_time);
			// if(1.5 - cur_time <= 0.01 && 1.5 - cur_time > ){
			// 	printf("Why\n");
			// }
			glfwSetWindowTitle(window, title.str().data());
			mesh.updateAnimation(cur_time);
			if((int)cur_time > mesh.keyframes.size()){
				gui.play_=false;
				gui.timer.stop();
			}
		} else if (gui.isPoseDirty()) {
			mesh.updateAnimation();
			gui.clearPose();
		}

		int current_bone = gui.getCurrentBone();

		// Draw bones first.
		if (draw_skeleton && gui.isTransparent()) {
			bone_pass.setup();
			// Draw our lines.
			// FIXME: you need setup skeleton.joints properly in
			//        order to see the bones.
			CHECK_GL_ERROR(glDrawElements(GL_LINES,
			                              bone_indices.size() * 2,
			                              GL_UNSIGNED_INT, 0));
		}
		draw_cylinder = (current_bone != -1 && gui.isTransparent());
		// printf("current bone is %d", current_bone);
		
		// Then draw floor.
		if (draw_floor) {
			floor_pass.setup();
			// Draw our triangles.
			CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
			                              floor_faces.size() * 3,
			                              GL_UNSIGNED_INT, 0));
		}
		if (gui.toResetSystem()) {
			ms_system.resetSystem();
			gui.clearResetFlag();
		}
		if (gui.toRandomDisturb()) {
			ms_system.randomDisturb();
			gui.clearDisturbFlag();
		}
		float delta_t = (float) toc(timer) * gui.getTimeSpeed();
		cloth.animate(delta_t);
		// Draw the model
// 		if (draw_object) {
// 			object_pass.setup();
// 			int mid = 0;
// 			while (object_pass.renderWithMaterial(mid))
// 				mid++;
// #if 0
// 			// For debugging also
// 			if (mid == 0) // Fallback
// 				CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES, mesh.faces.size() * 3, GL_UNSIGNED_INT, 0));
// #endif
// 		}

				if(draw_cylinder) {
			// printf("%d boners\n", current_bone);
			cylinder_pass.setup();
			CHECK_GL_ERROR(glDrawElements(GL_LINES,
		                              cylinder_mesh.indices.size() * 2,
		                              GL_UNSIGNED_INT, 0));
		}
			glViewport(window_width - scroll_bar_width, 0, window_width, window_height);
			scroll_pass.setup();
			CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
			                              scroll_faces.size() * 3,
			                              GL_UNSIGNED_INT, 0));
			glViewport(0, 0, main_view_width, main_view_height);
		



		if(gui.showPrev || mesh.refreshFrame != -1) {
			TextureToRender* texture = new TextureToRender();
			texture->create(main_view_width, main_view_height);
			texture->bind();

			floor_pass.setup();
			CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
			                              floor_faces.size() * 3,
			                              GL_UNSIGNED_INT, 0));

			cloth_pass.updateVBO(0, ms_system.node_positions.data(), ms_system.node_positions.size());
			cloth_pass.setup();
			// Draw our triangles.
			CHECK_GL_ERROR(glDrawElements(GL_LINES,
			                              ms_system.line_indices.size() * 2,
			                              GL_UNSIGNED_INT, 0));


			
			if(mesh.refreshFrame != -1){
				TextureToRender* prov = textures[mesh.refreshFrame];
				textures[mesh.refreshFrame] = texture;
				delete prov;
			} else {
				textures.push_back(texture);
			}
			
			// printf("texture size is %d\n", textures.size());
			texture->unbind();
			gui.showPrev = false;
			mesh.refreshFrame = -1;
		}
		if (draw_cloth) {
			cloth_pass.updateVBO(0, ms_system.node_positions.data(), ms_system.node_positions.size());
			cloth_pass.setup();
			// Draw our triangles.
			CHECK_GL_ERROR(glDrawElements(GL_LINES,
			                              ms_system.line_indices.size() * 2,
			                              GL_UNSIGNED_INT, 0));
		}

				if (draw_tri_cloth) {
			glDisable(GL_CULL_FACE);
			tri_cloth_pass.updateVBO(0, cloth.vertices.data(), cloth.vertices.size());
			tri_cloth_pass.updateVBO(1, cloth.cloth_uv_coords.data(), cloth.cloth_uv_coords.size());
			tri_cloth_pass.setup();

			CHECK_GL_ERROR(glDrawArrays(GL_TRIANGLES,
										0,
		                              	cloth.vertices.size()));
		}
		for(int i = 0; i < textures.size(); i++) {
			// printf("reached here \n");
			glViewport(main_view_width, main_view_height - (i + 1) * preview_height + gui.frame_shift, preview_width, preview_height);
			sampler = textures[i]->getTexture();
			if(i== gui.current_frame){
				show_border = gui.borderWall;
			} else {
				show_border = false;
			}
			

			preview_pass.setup();
			glDepthFunc(GL_LESS);
			CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
			                              prev_faces.size() * 3,
			                              GL_UNSIGNED_INT, 0));
		}	

		if(mesh.preLoaded){
			
			// gui.showPrev = false;
			for(int i = 0; i < mesh.keyframes.size(); i++) {
			// printf("reached here \n");
			mesh.updateAnimation(1.0 * i);
			TextureToRender* texture = new TextureToRender();
			texture->create(main_view_width, main_view_height);
			texture->bind();

			floor_pass.setup();
			CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
			                              floor_faces.size() * 3,
			                              GL_UNSIGNED_INT, 0));


			object_pass.setup();
			int mid = 0;
			while (object_pass.renderWithMaterial(mid))
				mid++;

			textures.push_back(texture);
			// printf("texture size is %d\n", textures.size());
			texture->unbind();
			mesh.preLoaded = false;
			mesh.setFirstFrame();
		}	
		// printf("and sponsors like you, thank you \n");
		
		}

		if (gui.video) {
			if (!if_file_open) {
				file_open = popen(cmd, "w");
				if_file_open = true;
			}
			

			glReadPixels(0, 0, main_view_width, main_view_height, GL_RGBA, GL_UNSIGNED_BYTE, v_buffer);

			fwrite(v_buffer, main_view_height * main_view_width * sizeof(int), 1, file_open);

			if (gui.getCurrentPlayTime() > mesh.keyframes.size()-1.0) {
				pclose(file_open);
				gui.video = false;
				if_file_open = false;
			}
}

		// Poll and swap.
		glfwPollEvents();
		glfwSwapBuffers(window);
	}
	for(int i = 0; i < textures.size(); i++) {
		delete(textures[i]);
}
	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}


