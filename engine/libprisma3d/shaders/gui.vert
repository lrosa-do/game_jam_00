#version 450

layout(location = 0) in vec2 aPosition;
layout(location = 1) in vec2 aUv;
layout(location = 2) in vec4 aColor;

layout(set = 0, binding = 0, std140) uniform Frame
{
    vec2 uInvDisplay;
    vec2 uPad;
};

layout(location = 0) out vec2 vUv;
layout(location = 1) out vec4 vColor;

void main()
{
    vUv = aUv;
    vColor = aColor;
    gl_Position = vec4(aPosition.x * uInvDisplay.x * 2.0 - 1.0,
            1.0 - aPosition.y * uInvDisplay.y * 2.0, 0.0, 1.0);
}
