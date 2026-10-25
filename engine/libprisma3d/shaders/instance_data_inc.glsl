struct InstanceData {
	mat3x4 transform;
	vec4 compressed_aabb_position_pad; // Only .xyz is used. .w is padding.
	vec3 compressed_aabb_size_pad; // Only .xyz is used. .w is padding.
	uint material_feedback_index; // Index into the material feedback buffer.
	vec4 uv_scale;
	uint flags;
	uint instance_uniforms_ofs; //base offset in global buffer for instance variables
	uint gi_offset; //GI information when using lightmapping (VCT or lightmap index)
	uint layer_mask;
	mat3x4 prev_transform;
	vec4 lightmap_uv_scale;
#ifdef USE_DOUBLE_PRECISION
	vec4 model_precision;
	vec4 prev_model_precision;
#endif
};

