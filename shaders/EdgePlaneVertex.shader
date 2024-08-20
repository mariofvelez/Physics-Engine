#version 420 core

uniform vec3 pos[4];
uniform float opacity[4];

out float Opacity;

uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position = projection * view * vec4(pos[gl_VertexID], 1.0);
	Opacity = opacity[gl_VertexID];
}