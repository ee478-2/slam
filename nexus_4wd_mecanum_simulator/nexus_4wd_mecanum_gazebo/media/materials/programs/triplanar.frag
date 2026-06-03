#version 120
// Fragment shader for triplanar projection.
// Samples the diffuse texture along three world-aligned planes (YZ, XZ, XY)
// and blends by absolute world-normal so each face shows the texture without
// stretching. Output is unlit diffuse color; ambient is applied via gazebo
// scene lighting since gl_FragColor is the final color.

uniform sampler2D diffuseTex;
uniform float scale;          // texture frequency in 1/world-units

varying vec3 vWorldPos;
varying vec3 vWorldNormal;

void main()
{
    vec3 n = abs(normalize(vWorldNormal));
    float total = n.x + n.y + n.z + 1e-6;
    n /= total;

    vec2 uvX = vWorldPos.yz * scale;   // YZ plane → faces with X-aligned normal
    vec2 uvY = vWorldPos.xz * scale;   // XZ plane → faces with Y-aligned normal
    vec2 uvZ = vWorldPos.xy * scale;   // XY plane → faces with Z-aligned normal

    vec4 cX = texture2D(diffuseTex, uvX);
    vec4 cY = texture2D(diffuseTex, uvY);
    vec4 cZ = texture2D(diffuseTex, uvZ);

    gl_FragColor = cX * n.x + cY * n.y + cZ * n.z;
}
