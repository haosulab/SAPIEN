// https://tuxedolabs.blogspot.com/2018/05/bokeh-depth-of-field-in-single-pass.html
#version 450

layout(set = 0, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
  mat4 prevViewMatrix;
  mat4 prevViewMatrixInverse;
  float width;
  float height;
} cameraBuffer;

layout(set = 1, binding = 0) uniform sampler2D samplerColor; //Image to be processed
layout(set = 1, binding = 1) uniform sampler2D samplerDepthLinear;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColorDof;

const float GOLDEN_ANGLE = 2.39996323;
const float MAX_BLUR_SIZE = 20.0;
const float RAD_SCALE = 2.; // Smaller = nicer blur, larger = faster

float getBlurSize(float depth, float focusPoint, float focusScale)
{
 float coc = clamp((1.0 / focusPoint - 1.0 / depth)*focusScale, -1.0, 1.0);
 return abs(coc) * MAX_BLUR_SIZE;
}

vec3 depthOfField(vec2 pixelSize, vec2 texCoord, float focusPoint, float focusScale)
{
 float centerDepth = texture(samplerDepthLinear, texCoord).x;
 float centerSize = getBlurSize(centerDepth, focusPoint, focusScale);
 vec3 color = texture(samplerColor, texCoord).rgb;
 float tot = 1.0;

 float radius = RAD_SCALE;
 for (float ang = 0.0; radius<MAX_BLUR_SIZE; ang += GOLDEN_ANGLE)
 {
  vec2 tc = texCoord + vec2(cos(ang), sin(ang)) * pixelSize * radius;

  vec3 sampleColor = texture(samplerColor, tc).rgb;
  float sampleDepth = texture(samplerDepthLinear, tc).x;
  float sampleSize = getBlurSize(sampleDepth, focusPoint, focusScale);
  if (sampleDepth > centerDepth)
   sampleSize = clamp(sampleSize, 0.0, centerSize*2.0);

  float m = smoothstep(radius-0.5, radius+0.5, sampleSize);
  color += mix(color/tot, sampleColor, m);
  tot += 1.0;
  radius += RAD_SCALE/radius;
 }
 return color /= tot;
}

void main() {
  vec2 pixelSize = vec2(1./cameraBuffer.width, 1./cameraBuffer.height);
  vec3 color = depthOfField(pixelSize, inUV, 5, 0.5);
  outColorDof = vec4(color, 1.);
}
