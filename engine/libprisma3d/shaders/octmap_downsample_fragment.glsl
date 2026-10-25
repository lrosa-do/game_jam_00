// Copyright 2016 Activision Publishing, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software
// is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "oct_inc.glsl"

layout(set = 0, binding = 0, std140) uniform OctmapParams {
	float border_size;
	uint size;
	uint source_lod;
	uint source_layer;
}
params;

#ifdef OCTMAP_SOURCE_ARRAY
layout(set = 1, binding = 0) uniform sampler2DArray source_octmap;
#define SAMPLE_SOURCE(uv, lod) textureLod(source_octmap, vec3(uv, float(params.source_layer)), lod)
#else
layout(set = 1, binding = 0) uniform sampler2D source_octmap;
#define SAMPLE_SOURCE(uv, lod) textureLod(source_octmap, uv, lod)
#endif

layout(location = 0) in vec2 uv_interp;
layout(location = 0) out vec4 frag_color;

float calcWeight(float u, float v) {
	float val = u * u + v * v + 1.0;
	return val * sqrt(val);
}

void main() {
#ifdef USE_HIGH_QUALITY
	float inv_size = 1.0 / float(params.size);
	float u0 = (uv_interp.x * 2.0f) + (1.0f - 0.75f) * inv_size - 1.0f;
	float u1 = (uv_interp.x * 2.0f) + (1.0f + 0.75f) * inv_size - 1.0f;
	float v0 = (uv_interp.y * 2.0f) + (1.0f - 0.75f) * inv_size - 1.0f;
	float v1 = (uv_interp.y * 2.0f) + (1.0f + 0.75f) * inv_size - 1.0f;
	float weights[4];
	weights[0] = calcWeight(u0, v0);
	weights[1] = calcWeight(u1, v0);
	weights[2] = calcWeight(u0, v1);
	weights[3] = calcWeight(u1, v1);

	const float wsum = 0.5 / (weights[0] + weights[1] + weights[2] + weights[3]);
	for (int i = 0; i < 4; i++) {
		weights[i] = weights[i] * wsum + 0.125;
	}

	frag_color = SAMPLE_SOURCE(vec2(u0, v0) * 0.5f + 0.5f, float(params.source_lod)) * weights[0];
	frag_color += SAMPLE_SOURCE(vec2(u1, v0) * 0.5f + 0.5f, float(params.source_lod)) * weights[1];
	frag_color += SAMPLE_SOURCE(vec2(u0, v1) * 0.5f + 0.5f, float(params.source_lod)) * weights[2];
	frag_color += SAMPLE_SOURCE(vec2(u1, v1) * 0.5f + 0.5f, float(params.source_lod)) * weights[3];
#else
	frag_color = SAMPLE_SOURCE(uv_interp, float(params.source_lod));
#endif
}
