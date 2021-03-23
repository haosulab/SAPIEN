#version 450

layout(set = 0, binding = 0) uniform sampler2D samplerAlbedo;
layout(set = 0, binding = 1) uniform sampler2D samplerSpecular;
layout(set = 0, binding = 2) uniform sampler2D samplerNormal;
layout(set = 0, binding = 3) uniform sampler2D samplerGbufferDepth;
layout(set = 0, binding = 4) uniform sampler2D samplerLighting;

layout(set = 1, binding = 0) uniform CameraBuffer {
  mat4 viewMatrix;
  mat4 projectionMatrix;
  mat4 viewMatrixInverse;
  mat4 projectionMatrixInverse;
  mat4 prevViewMatrix;
  mat4 prevViewMatrixInverse;
  float width;
  float height;
} cameraBuffer;

layout (location = 0) in vec2 inUV;
layout (location = 0) out vec4 outLightingSSR;


// Consts should help improve performance
const float rayStep = 0.01;
const float minRayStep = 0.1;
const float maxSteps = 60;
const float searchDist = 5;
const float searchDistInv = 0.2;
const int numBinarySearchSteps = 10;
const float maxThickness = 2e-3;

const float reflectionSpecularFalloffExponent = 3;

const vec3 Scale = vec3(.8, .8, .8);
const float K = 19.19;
vec3 hash(vec3 a)
{
  a = fract(a * Scale);
  a += dot(a, a.yxz + K);
  return fract((a.xxy + a.yxx)*a.zyx);
}

vec3 BinarySearch(vec3 dir, inout vec3 hitCoord, out float dDepth, out float thickness)
{
  float depth;
  vec3 searchCoord = hitCoord;

  for(int i = 0; i < numBinarySearchSteps; i++)
  {
    vec4 projectedCoord = cameraBuffer.projectionMatrix * vec4(searchCoord, 1.0);
    projectedCoord.xy /= projectedCoord.w;
    projectedCoord.xy = projectedCoord.xy * 0.5 + 0.5;

    depth = texture(samplerGbufferDepth, projectedCoord.xy).x;
    vec4 p = cameraBuffer.projectionMatrixInverse * (vec4(0, 0, depth, 1));
    depth = p.z / p.w;

    dDepth = searchCoord.z - depth;

    if(dDepth > 0.0) {
      searchCoord += dir;
      thickness = dDepth;
    } else {
      hitCoord = searchCoord;
    }

    dir *= 0.5;
    searchCoord -= dir;
  }

  vec4 projectedCoord = cameraBuffer.projectionMatrix * vec4(hitCoord, 1.0);
  projectedCoord.xy /= projectedCoord.w;
  projectedCoord.xy = projectedCoord.xy * 0.5 + 0.5;

  return vec3(projectedCoord.xy, depth);
}


vec4 RayCast(vec3 dir, inout vec3 hitCoord, out float dDepth)
{
  dir *= rayStep;

  float depth;
  float thickness = 0.;

  for(int i = 0; i < maxSteps; i++)
  {
    hitCoord += dir;

    vec4 projectedCoord = cameraBuffer.projectionMatrix * vec4(hitCoord, 1.0);
    projectedCoord.xy /= projectedCoord.w;
    projectedCoord.xy = projectedCoord.xy * 0.5 + 0.5;

    depth = texture(samplerGbufferDepth, projectedCoord.xy).x;
    vec4 p = cameraBuffer.projectionMatrixInverse * (vec4(0, 0, depth, 1));
    depth = p.z / p.w;

    dDepth = hitCoord.z - depth;

    if(dDepth < 0.0) {
      vec4 result = vec4(BinarySearch(dir, hitCoord, dDepth, thickness), 1.0);
      if (thickness > maxThickness) {
        return vec4(0.0, 0.0, 0.0, 0.0);
      }
      return result;
    }
    thickness = dDepth;
  }

  return vec4(0.0, 0.0, 0.0, 0.0);
}


void main()
{
  // Samples
  // float specular = texture(samplerSpecular, inUV).x;

  vec3 albedo = texture(samplerAlbedo, inUV).xyz;
  vec3 frm = texture(samplerSpecular, inUV).xyz;
  float specular = frm.x;
  float roughness = frm.y;
  float metallic = frm.z;
  vec3 fresnel = specular * (1 - metallic) + albedo * metallic;

  if(fresnel.x < 0.1 && fresnel.y < 0.1 && fresnel.z < 0.1)
  {
    outLightingSSR = vec4(0.);
    return;
  }

  vec3 viewNormal = texture(samplerNormal, inUV).xyz * 2 - 1;

  float depth = texture(samplerGbufferDepth, inUV).x;

  vec4 csPosition = cameraBuffer.projectionMatrixInverse * (vec4(inUV * 2 - 1, depth, 1));
  vec3 viewPos = csPosition.xyz / csPosition.w;

  // vec3 viewPos = texture(samplerPosition, inUV).xyz;

  // Reflection vector
  vec3 reflected = normalize(reflect(normalize(viewPos), normalize(viewNormal)));

  // Ray cast
  vec3 hitPos = viewPos + 1e-3 * viewNormal;
  float dDepth;

  // // Jitter based on specular value
  // vec3 jitter = mix(vec3(0.0), vec3(hash(vec3(inUV * 2 - 1, depth))), specular);

  vec4 coords = RayCast(reflected * max(minRayStep, -viewPos.z), hitPos, dDepth);

  // Implementing blur near edges
  vec2 dCoords = smoothstep(0.2, 0.6, abs(vec2(0.5, 0.5) - coords.xy));


  // Discarding samples where the reflected ray goes off screen
  float screenEdgefactor = clamp(1.0 - (dCoords.x + dCoords.y), 0.0, 1.0);

  vec3 alpha = pow(fresnel, vec3(reflectionSpecularFalloffExponent)) *
               screenEdgefactor * clamp(-reflected.z, 0.0, 1.0) *
               clamp((searchDist - length(viewPos - hitPos)) * searchDistInv, 0.0, 1.0);

  // Get color
  outLightingSSR = vec4(texture(samplerLighting, coords.xy).rgb, alpha);
}
