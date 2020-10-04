#version 450 

layout(set = 0, binding = 5) uniform usampler2D segmentationSampler;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

vec4 colors[60] = {  
  vec4(0.8,  0.4,  0.4 , 1 ),
  vec4(0.8,  0.41, 0.24, 1 ), 
  vec4(0.8,  0.75, 0.32, 1 ), 
  vec4(0.6,  0.8,  0.4 , 1 ),  
  vec4(0.35, 0.8,  0.24, 1 ), 
  vec4(0.32, 0.8,  0.51, 1 ), 
  vec4(0.4,  0.8,  0.8 , 1 ),  
  vec4(0.24, 0.63, 0.8 , 1 ),  
  vec4(0.32, 0.37, 0.8 , 1 ),  
  vec4(0.6,  0.4,  0.8 , 1 ),  
  vec4(0.69, 0.24, 0.8 , 1 ),  
  vec4(0.8,  0.32, 0.61, 1 ), 
  vec4(0.8,  0.32, 0.32, 1 ),
  vec4(0.8,  0.64, 0.4 , 1 ), 
  vec4(0.8,  0.74, 0.24, 1 ),
  vec4(0.56, 0.8,  0.32, 1 ),
  vec4(0.4,  0.8,  0.44, 1 ),
  vec4(0.24, 0.8,  0.46, 1 ),
  vec4(0.32, 0.8,  0.8 , 1 ), 
  vec4(0.4,  0.56, 0.8 , 1 ), 
  vec4(0.24, 0.3,  0.8 , 1 ), 
  vec4(0.56, 0.32, 0.8 , 1 ), 
  vec4(0.8,  0.4,  0.76, 1 ),
  vec4(0.8,  0.24, 0.58, 1 ),
  vec4(0.8,  0.24, 0.24, 1 ),
  vec4(0.8,  0.61, 0.32, 1 ),
  vec4(0.72, 0.8,  0.4 , 1 ), 
  vec4(0.52, 0.8,  0.24, 1 ),
  vec4(0.32, 0.8,  0.37, 1 ),
  vec4(0.4,  0.8,  0.68, 1 ),
  vec4(0.24, 0.8,  0.8 , 1 ), 
  vec4(0.32, 0.51, 0.8 , 1 ), 
  vec4(0.48, 0.4,  0.8 , 1 ), 
  vec4(0.52, 0.24, 0.8 , 1 ), 
  vec4(0.8,  0.32, 0.75, 1 ),
  vec4(0.8,  0.4,  0.52, 1 ),
  vec4(0.8,  0.52, 0.4 , 1 ), 
  vec4(0.8,  0.58, 0.24, 1 ),
  vec4(0.7,  0.8,  0.32, 1 ),
  vec4(0.48, 0.8,  0.4 , 1 ), 
  vec4(0.24, 0.8,  0.3 , 1 ), 
  vec4(0.32, 0.8,  0.66, 1 ),
  vec4(0.4,  0.68, 0.8 , 1 ), 
  vec4(0.24, 0.46, 0.8 , 1 ), 
  vec4(0.42, 0.32, 0.8 , 1 ), 
  vec4(0.72, 0.4,  0.8 , 1 ), 
  vec4(0.8,  0.24, 0.74, 1 ),
  vec4(0.8,  0.32, 0.46, 1 ),
  vec4(0.8,  0.46, 0.32, 1 ),
  vec4(0.8,  0.76, 0.4 , 1 ),
  vec4(0.69, 0.8,  0.24, 1 ),
  vec4(0.42, 0.8,  0.32, 1 ),
  vec4(0.4,  0.8,  0.56, 1 ),
  vec4(0.24, 0.8,  0.63, 1 ),
  vec4(0.32, 0.66, 0.8 , 1 ),
  vec4(0.4,  0.44, 0.8 , 1 ),
  vec4(0.35, 0.24, 0.8 , 1 ),
  vec4(0.7,  0.32, 0.8 , 1 ),
  vec4(0.8,  0.4,  0.64, 1 ),
  vec4(0.8,  0.24, 0.41, 1 )
};


void main() {
  uint id = texture(segmentationSampler, inUV).y;
  if (id > 0) {
    outColor = colors[(id - 1) % 60];
  } else {
    outColor = vec4(0,0,0,1);
  }
}