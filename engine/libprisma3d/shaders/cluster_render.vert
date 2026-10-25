#version 450

layout(location = 0) in vec3 vertex_attrib;

layout(location = 0) out float depth_interp;
layout(location = 1) out flat uint element_index;

layout(set = 0, binding = 0, std140) uniform Params {
	uint base_index;
	uint pad0;
	uint pad1;
	uint pad2;
}
params;

layout(set = 0, binding = 1, std140) uniform State {
	mat4 projection;

	float inv_z_far;
	uint screen_to_clusters_shift; // shift to obtain coordinates in block indices
	uint cluster_screen_width; //
	uint cluster_data_size; // how much data for a single cluster takes

	uint cluster_depth_offset;
	uint pad0;
	uint pad1;
	uint pad2;
}
state;

struct RenderElement {
	uint type; //0-4
	bool touches_near;
	bool touches_far;
	uint original_index;
	mat3x4 transform_inv;
	vec3 scale;
	uint pad;
};

layout(set = 2, binding = 0, std430) buffer restrict readonly RenderElements {
	RenderElement data[];
}
render_elements;

void main() {
	element_index = params.base_index + gl_InstanceIndex;

	vec3 vertex = vertex_attrib;
	vertex *= render_elements.data[element_index].scale;

	vertex = vec4(vertex, 1.0) * render_elements.data[element_index].transform_inv;
	depth_interp = -vertex.z;

	gl_Position = state.projection * vec4(vertex, 1.0);
}
