#version 120
// Vertex shader for triplanar texture projection.
// Outputs world-space position and normal so the fragment shader can sample
// a 2D texture using XY/XZ/YZ planar projections — no UV map required.

uniform mat4 worldMatrix;
uniform mat4 worldViewProjMatrix;

varying vec3 vWorldPos;
varying vec3 vWorldNormal;

void main()
{
    vec4 wp = worldMatrix * gl_Vertex;
    vWorldPos = wp.xyz;
    vWorldNormal = mat3(worldMatrix) * gl_Normal;
    gl_Position = worldViewProjMatrix * gl_Vertex;
}
