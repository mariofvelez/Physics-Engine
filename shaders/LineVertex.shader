#version 420 core
layout(location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform vec3 scale;

void main()
{
	vec3 pos = aPos * scale;
	gl_Position = projection * view * model * vec4(pos, 1.0);
	gl_Position.z -= 0.001f;
}