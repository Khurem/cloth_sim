R"zzz(#version 330 core
uniform mat4 bone_transform; // transform the cylinder to the correct configuration
const float kPi = 3.1415926535897932384626433832795;
uniform mat4 projection;
uniform float cylinder_radius;
uniform mat4 model;
uniform mat4 view;
in vec4 vertex_position;

// FIXME: Implement your vertex shader for cylinders
// Note: you need call sin/cos to transform the input mesh to a cylinder


void main() {
	float theta = vertex_position.x * 2 * kPi;
	vec4 position = vec4(cylinder_radius * sin(theta), vertex_position.y, cylinder_radius * cos(theta), 1);
	mat4 world = projection * view * model;
	gl_Position = world * bone_transform *  vertex_position;
	gl_Position = world * bone_transform *  position;
}
)zzz"
