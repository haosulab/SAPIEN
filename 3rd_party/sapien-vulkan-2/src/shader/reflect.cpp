/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "reflect.h"
#include <stdexcept>

namespace svulkan2 {

DataType get_data_type(spirv_cross::SPIRType const &type) {
  uint32_t shape = type.vecsize * type.columns;
  switch (type.basetype) {
  case spirv_cross::SPIRType::Struct:
    return DataType::STRUCT();
  case spirv_cross::SPIRType::SByte:
    return {shape, TypeKind::eInt, 1};
  case spirv_cross::SPIRType::Short:
    return {shape, TypeKind::eInt, 2};
  case spirv_cross::SPIRType::Int:
    return {shape, TypeKind::eInt, 4};
  case spirv_cross::SPIRType::Int64:
    return {shape, TypeKind::eInt, 8};

  case spirv_cross::SPIRType::UByte:
    return {shape, TypeKind::eUint, 1};
  case spirv_cross::SPIRType::UShort:
    return {shape, TypeKind::eUint, 2};
  case spirv_cross::SPIRType::UInt:
    return {shape, TypeKind::eUint, 4};
  case spirv_cross::SPIRType::UInt64:
    return {shape, TypeKind::eUint, 8};

  case spirv_cross::SPIRType::Half:
    return {shape, TypeKind::eFloat, 2};
  case spirv_cross::SPIRType::Float:
    return {shape, TypeKind::eFloat, 4};
  case spirv_cross::SPIRType::Double:
    return {shape, TypeKind::eFloat, 8};
  default:
    throw std::runtime_error("unsupported type");
  }
}

// bool type_is_int(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Int && type.vecsize == 1 && type.columns == 1;
// }
// bool type_is_uint(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::UInt && type.vecsize == 1 && type.columns == 1;
// }
// bool type_is_float(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Float && type.vecsize == 1 && type.columns ==
//   1;
// }

// bool type_is_int2(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Int && type.vecsize == 2 && type.columns == 1;
// }
// bool type_is_uint2(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::UInt && type.vecsize == 2 && type.columns == 1;
// }
// bool type_is_float2(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Float && type.vecsize == 2 && type.columns ==
//   1;
// }

// bool type_is_int3(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Int && type.vecsize == 3 && type.columns == 1;
// }
// bool type_is_uint3(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::UInt && type.vecsize == 3 && type.columns == 1;
// }
// bool type_is_float3(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Float && type.vecsize == 3 && type.columns ==
//   1;
// }

// bool type_is_int4(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Int && type.vecsize == 4 && type.columns == 1;
// }
// bool type_is_uint4(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::UInt && type.vecsize == 4 && type.columns == 1;
// }
// bool type_is_float4(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Float && type.vecsize == 4 && type.columns ==
//   1;
// }

// bool type_is_int44(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Int && type.vecsize == 4 && type.columns == 4;
// }
// bool type_is_uint44(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::UInt && type.vecsize == 4 && type.columns == 4;
// }
// bool type_is_float44(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Float && type.vecsize == 4 && type.columns ==
//   4;
// }

// bool type_is_struct(spirv_cross::SPIRType const &type) {
//   return type.basetype == spirv_cross::SPIRType::Struct;
// }

// DataType get_data_type(spirv_cross::SPIRType const &type) {
//   if (type_is_int(type)) {
//     return DataType::INT();
//   }
//   if (type_is_int2(type)) {
//     return DataType::INT2();
//   }
//   if (type_is_int3(type)) {
//     return DataType::INT3();
//   }
//   if (type_is_int4(type)) {
//     return DataType::INT4();
//   }
//   if (type_is_int44(type)) {
//     return DataType::INT44();
//   }

//   if (type_is_float(type)) {
//     return DataType::FLOAT();
//   }
//   if (type_is_float2(type)) {
//     return DataType::FLOAT2();
//   }
//   if (type_is_float3(type)) {
//     return DataType::FLOAT3();
//   }
//   if (type_is_float4(type)) {
//     return DataType::FLOAT4();
//   }
//   if (type_is_float44(type)) {
//     return DataType::FLOAT44();
//   }

//   if (type_is_uint(type)) {
//     return DataType::UINT();
//   }
//   if (type_is_uint2(type)) {
//     return DataType::UINT2();
//   }
//   if (type_is_uint3(type)) {
//     return DataType::UINT3();
//   }
//   if (type_is_uint4(type)) {
//     return DataType::UINT4();
//   }
//   if (type_is_uint44(type)) {
//     return DataType::UINT44();
//   }

//   if (type_is_struct(type)) {
//     return DataType::STRUCT();
//   }

//   return DataType::UNKNOWN();
// }

spirv_cross::Resource *find_uniform_by_decoration(spirv_cross::Compiler &compiler,
                                                  spirv_cross::ShaderResources &resource,
                                                  uint32_t binding_number, uint32_t set_number) {

  for (auto &r : resource.uniform_buffers) {
    if (compiler.get_decoration(r.id, spv::Decoration::DecorationBinding) == binding_number &&
        compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet) == set_number) {
      return &r;
    }
  }
  return nullptr;
}

spirv_cross::Resource *find_sampler_by_decoration(spirv_cross::Compiler &compiler,
                                                  spirv_cross::ShaderResources &resource,
                                                  uint32_t binding_number, uint32_t set_number) {

  for (auto &r : resource.sampled_images) {
    if (compiler.get_decoration(r.id, spv::Decoration::DecorationBinding) == binding_number &&
        compiler.get_decoration(r.id, spv::Decoration::DecorationDescriptorSet) == set_number) {
      return &r;
    }
  }
  return nullptr;
}

vk::Format get_image_format(spv::ImageFormat format) {
  switch (format) {
  case spv::ImageFormat::ImageFormatUnknown:
    return vk::Format::eUndefined;

  case spv::ImageFormat::ImageFormatRgba32f:
    return vk::Format::eR32G32B32A32Sfloat;
  case spv::ImageFormat::ImageFormatR32f:
    return vk::Format::eR32Sfloat;

  case spv::ImageFormat::ImageFormatRgba16f:
    return vk::Format::eR16G16B16A16Sfloat;
  case spv::ImageFormat::ImageFormatR16f:
    return vk::Format::eR16Sfloat;

  case spv::ImageFormat::ImageFormatRgba32i:
    return vk::Format::eR32G32B32A32Sint;
  case spv::ImageFormat::ImageFormatRgba32ui:
    return vk::Format::eR32G32B32A32Uint;

  case spv::ImageFormat::ImageFormatRgba8:
    return vk::Format::eR8G8B8A8Unorm;
  case spv::ImageFormat::ImageFormatR8:
    return vk::Format::eR8Unorm;

  default:
    throw std::runtime_error("Unsupported spirv format.");
  }
}

} // namespace svulkan2