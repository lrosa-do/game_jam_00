// Copied from the reference implementation of SMAA (see third_party/smaa/NOTICE.md); only the
// resource declarations changed: the push constant is a uniform block, the samplers are on set 1.


layout(location = 0) out vec2 tex_coord;
layout(location = 1) out vec2 pix_coord;
layout(location = 2) out vec4 offset[3];

layout(set = 0, binding = 0, std140) uniform Params {
	vec2 inv_size;
	ivec2 size;

	vec4 subsample_indices;
}
params;

#define SMAA_MAX_SEARCH_STEPS 32

void main() {
	vec2 vertex_base;
	if (gl_VertexIndex == 0) {
		vertex_base = vec2(-1.0, -1.0);
	} else if (gl_VertexIndex == 1) {
		vertex_base = vec2(-1.0, 3.0);
	} else {
		vertex_base = vec2(3.0, -1.0);
	}
	gl_Position = vec4(vertex_base, 0.0, 1.0);
	tex_coord = clamp(vertex_base, vec2(0.0, 0.0), vec2(1.0, 1.0)) * 2.0; // saturate(x) * 2.0
	pix_coord = tex_coord * params.size.xy;

	offset[0] = fma(params.inv_size.xyxy, vec4(-0.25, -0.125, 1.25, -0.125), tex_coord.xyxy);
	offset[1] = fma(params.inv_size.xyxy, vec4(-0.125, -0.25, -0.125, 1.25), tex_coord.xyxy);
	offset[2] = fma(params.inv_size.xxyy,
			vec4(-2.0, 2.0, -2.0, 2.0) * SMAA_MAX_SEARCH_STEPS,
			vec4(offset[0].xz, offset[1].yw));
}

