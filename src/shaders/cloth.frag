R"zzz(
#version 330 core
in vec2 uv_frag;
out vec4 fragment_color;

void main() {
	fragment_color = vec4(1.0, 1.0, 0.0, 1.0);
		if(uv_frag[0] <= 0.01 || uv_frag[0] >= 0.99 || uv_frag[1] <= 0.01 || uv_frag[1] >= 0.99) {
		fragment_color = vec4(1.0, 0.0, 0.0, 1.0);
	}
}
)zzz"