R"zzz(#version 330 core





in vec3 vertex_position;
in vec2 uv;
in vec3 normal;
uniform vec4 light_position;
uniform vec3 camera_position;
out vec4 vs_light_direction;
out vec2 vs_uv;
out vec4 vs_camera_direction;
out vec3 vs_normal;

/*
in vec3 vertex_position;
in vec2 uv;
uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;
out vec2 uv_frag;
*/

void main() {
	/*
	mat4 mvp = projection * view * model;
	gl_Position = mvp * vec4(vertex_position, 1.0);
	uv_frag = vec2(uv[0] - 0.6, uv[1]);
	*/

	
	gl_Position = vec4(vertex_position, 1.0);
	vs_light_direction = light_position - gl_Position;
	vs_camera_direction = vec4(camera_position, 1.0) - gl_Position;
	vs_uv = uv;
	vs_normal = normal;

}

)zzz"