#include "half_inc.glsl"
#include "scene_forward_clustered_inc.glsl"

layout(set = 2, binding = 5, std430) restrict readonly buffer ImpostorCards {
	uint data[];
}
impostor_cards;

layout(location = 0) out vec3 vertex_interp;
layout(location = 1) out vec3 normal_interp;
layout(location = 2) out vec4 color_interp;
layout(location = 3) out vec2 uv_interp;
layout(location = 10) out flat uint instance_index_interp;
layout(location = 15) out flat uint impostor_slice;
layout(location = 16) out flat float impostor_dither;
layout(location = 17) out flat float dissolve_interp;

invariant gl_Position;

const vec2 BILLBOARD[4] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(-1.0, 1.0), vec2(1.0, 1.0));

const uint HEADER_WORDS = 8u;

void main() {
	uint capacity = draw_call.uv_offset;
	uint positions_base = HEADER_WORDS;
	uint normals_base = positions_base + capacity * 16u;
	uint data_base = normals_base + capacity * 16u;

	uint vertex_id = uint(gl_VertexIndex);
	uint impostor_id = vertex_id / 4u;

	uint at = positions_base + vertex_id * 4u;
	vec3 pos3d = vec3(uintBitsToFloat(impostor_cards.data[at]), uintBitsToFloat(impostor_cards.data[at + 1u]), uintBitsToFloat(impostor_cards.data[at + 2u]));
	uint nat = normals_base + vertex_id * 4u;
	vec3 face = vec3(uintBitsToFloat(impostor_cards.data[nat]), uintBitsToFloat(impostor_cards.data[nat + 1u]), uintBitsToFloat(impostor_cards.data[nat + 2u]));
	uvec2 data = uvec2(impostor_cards.data[data_base + impostor_id * 2u], impostor_cards.data[data_base + impostor_id * 2u + 1u]);

	mat4 read_view_matrix = transpose(mat4(scene_data_block.data.view_matrix[0],
			scene_data_block.data.view_matrix[1],
			scene_data_block.data.view_matrix[2],
			vec4(0.0, 0.0, 0.0, 1.0)));

	vertex_interp = (read_view_matrix * vec4(pos3d, 1.0)).xyz;
	normal_interp = normalize(mat3(read_view_matrix) * face);
	color_interp = unpackUnorm4x8(data.y);
	uv_interp = BILLBOARD[vertex_id % 4u] * vec2(-0.5, 0.5) + 0.5;
	instance_index_interp = 0u;
	impostor_slice = data.x & 0xFFFFFFu;
	impostor_dither = float((data.x >> 24u) & 0xFFu) / 255.0;
	dissolve_interp = 0.0;

	gl_Position = scene_data_block.data.projection_matrix * vec4(vertex_interp, 1.0);
}
