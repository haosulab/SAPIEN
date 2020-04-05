#version 410

layout(location=0) in vec3 vpos;
layout(location=1) in vec3 vnormal;
layout(location=2) in vec2 vtexcoord;
layout(location=3) in vec3 vtangent;
layout(location=4) in vec3 vbitangent;

uniform mat4 lightSpaceMatrix;
uniform mat4 gbufferModelMatrix;

void main()
{
  gl_Position = lightSpaceMatrix * gbufferModelMatrix * vec4(vpos, 1.0);
}  
