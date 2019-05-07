R"zzz(
#version 330 core

/*
in vec2 uv_frag;
uniform sampler2D sampler;
out vec4 fragment_color;
*/


in vec4 face_normal;
in vec4 light_direction;
in vec4 camera_direction;
in vec2 uv_coords;

in vec3 vertex_normal;
in vec4 world_position;

uniform sampler2D sampler;
out vec4 fragment_color;


void main() {
	/*
	fragment_color = vec4(1.0, 1.0, 0.0, 1.0);
	if(uv_frag[0] <= 0.01 || uv_frag[0] >= 0.99 || uv_frag[1] <= 0.01 || uv_frag[1] >= 0.99) {
		fragment_color = vec4(1.0, 0.0, 0.0, 1.0);
	}
	*/
	
	/*
	vec3 texcolor = texture(sampler, uv_frag).xyz;
	fragment_color = vec4(texcolor.rgb, 1.0);
	*/

	
	vec3 texcolor = texture(sampler, uv_coords).xyz;
	// float dot_nl = dot(normalize(light_direction), normalize(face_normal));
	float dot_nl = abs(dot(normalize(light_direction), normalize(vec4(vertex_normal, 0.0))));

	dot_nl = clamp(dot_nl + 0.4f, 0.0, 1.0);
	vec3 color = clamp(dot_nl * texcolor, 0.0, 1.0);
		// fragment_color = vec4(1.0, 1.0, 0.0, 1.0);

	float vx = round(uv_coords.x * (10 - 1));
    float vy = round(uv_coords.y * (10 - 1));
	fragment_color= vec4(vx / (10 - 1), vy / (10- 1), 0.2, 1.0);
	
}
)zzz"