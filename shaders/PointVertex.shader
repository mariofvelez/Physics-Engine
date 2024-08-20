#version 420 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;

uniform vec3 position;
uniform mat4 view;
uniform mat4 projection;

uniform vec3 scale;

void main()
{
	vec3 pos = aPos * scale + position;
	gl_Position = projection * view * vec4(pos, 1.0);
}