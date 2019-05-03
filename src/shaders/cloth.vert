R"zzz(#version 330 core

in vec3 vertex_position;
in vec2 uv;
uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;
out vec2 uv_frag;


void main() {
	mat4 mvp = projection * view * model;
	gl_Position = mvp * vec4(vertex_position, 1.0);
	uv_frag = uv;
}

)zzz"