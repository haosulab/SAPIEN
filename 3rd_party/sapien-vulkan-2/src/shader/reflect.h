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
#pragma once

#include "svulkan2/common/layout.h"
#include <spirv_cross.hpp>

namespace svulkan2 {

// DataType SPIRTypeToType(spirv_cross::SPIRType const &type);

// bool type_is_int(spirv_cross::SPIRType const &type);
// bool type_is_uint(spirv_cross::SPIRType const &type);
// bool type_is_float(spirv_cross::SPIRType const &type);

// bool type_is_int2(spirv_cross::SPIRType const &type);
// bool type_is_uint2(spirv_cross::SPIRType const &type);
// bool type_is_float2(spirv_cross::SPIRType const &type);

// bool type_is_int3(spirv_cross::SPIRType const &type);
// bool type_is_uint3(spirv_cross::SPIRType const &type);
// bool type_is_float3(spirv_cross::SPIRType const &type);

// bool type_is_int4(spirv_cross::SPIRType const &type);
// bool type_is_uint4(spirv_cross::SPIRType const &type);
// bool type_is_float4(spirv_cross::SPIRType const &type);

// bool type_is_int44(spirv_cross::SPIRType const &type);
// bool type_is_uint44(spirv_cross::SPIRType const &type);
// bool type_is_float44(spirv_cross::SPIRType const &type);

// bool type_is_struct(spirv_cross::SPIRType const &type);

DataType get_data_type(spirv_cross::SPIRType const &type);

spirv_cross::Resource *
find_uniform_by_decoration(spirv_cross::Compiler &compiler,
                           spirv_cross::ShaderResources &resource,
                           uint32_t binding_number, uint32_t set_number);

spirv_cross::Resource *
find_sampler_by_decoration(spirv_cross::Compiler &compiler,
                           spirv_cross::ShaderResources &resource,
                           uint32_t binding_number, uint32_t set_number);

vk::Format get_image_format(spv::ImageFormat format);
} // namespace svulkan2