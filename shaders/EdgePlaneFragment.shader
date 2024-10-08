#version 420 core

in float Opacity;

out vec4 FragColor;

uniform vec3 color;

void main()
{
	FragColor = vec4(color, Opacity);
}