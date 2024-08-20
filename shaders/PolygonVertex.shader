#version 420 core

uniform vec3 pos[8];

uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position = projection * view * vec4(pos[gl_VertexID], 1.0);
}