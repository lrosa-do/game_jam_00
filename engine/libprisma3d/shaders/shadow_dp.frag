#version 450

layout(location = 9) in float dp_clip;

void main()
{
	if (dp_clip > 0.0) {
		discard;
	}
}
