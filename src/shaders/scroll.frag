R"zzz(#version 330 core
out vec4 fragment_color;
in vec2 tex_coord;

uniform float frame_shift;
uniform int total_preview_num; 

void main() {
	if(total_preview_num <= 3) {
		fragment_color = vec4(0.0, 0.0, 0.0, 1.0);
	}
	else {
		float scroll = (1.0 * frame_shift) / (total_preview_num * 240);
		float cube = 3.0 * 1.0 / total_preview_num;
		float dy = 1.0 - tex_coord.y;
		if(dy > scroll && dy < scroll + cube) {
			fragment_color = vec4(1.0, 1.0, 1.0, 1.0);
		} else {
			fragment_color = vec4(0.0, 0.0, 0.0, 1.0);
		}

	}

	

}
)zzz"