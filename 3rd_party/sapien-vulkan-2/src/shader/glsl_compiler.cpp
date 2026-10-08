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
#include "svulkan2/shader/glsl_compiler.h"
#include "../common/logger.h"
#include "svulkan2/common/fs.h"
#include "svulkan2/common/vk.h"
#include <SPIRV/GlslangToSpv.h>
#include <glslang/Public/ShaderLang.h>
#include <mutex>
#include <regex>
#include <unordered_map>

namespace svulkan2 {

static std::mutex mutex;
static std::unordered_map<std::string, std::vector<std::uint32_t>> shaderCodeCache;

std::vector<std::uint32_t> const &
GLSLCompiler::compileGlslFileCached(vk::ShaderStageFlagBits stage, fs::path const &filepath) {
  std::string path = fs::canonical(filepath).string();
  std::lock_guard<std::mutex> guard(mutex);
  if (shaderCodeCache.contains(path)) {
    return shaderCodeCache[path];
  }
  auto [code, debugInfo] = GLSLCompiler::loadGlslCodeWithDebugInfo(filepath);
  return shaderCodeCache[path] = GLSLCompiler::compileToSpirv(stage, code, debugInfo);
}

std::tuple<std::string, std::vector<std::tuple<std::string, int>>>
GLSLCompiler::loadGlslCodeWithDebugInfo(fs::path const &filepath) {
  std::vector<char> charCode = readFile(filepath);
  std::string code{charCode.begin(), charCode.end()};
  std::istringstream iss(code);
  std::string result;

  std::vector<std::tuple<std::string, int>> lineInfo;
  int lineNum = 1;

  for (std::string line; std::getline(iss, line); ++lineNum) {
    // left trim for erase
    std::string line2 = line;
    line2.erase(line2.begin(), std::find_if(line2.begin(), line2.end(),
                                            [](unsigned char ch) { return !std::isspace(ch); }));

    if (line2.starts_with("#include") && std::isspace(line[8])) {
      line2 = line2.substr(8);

      // lr trim
      line2.erase(line2.begin(), std::find_if(line2.begin(), line2.end(),
                                              [](unsigned char ch) { return !std::isspace(ch); }));
      line2.erase(std::find_if(line2.rbegin(), line2.rend(),
                               [](unsigned char ch) { return !std::isspace(ch); })
                      .base(),
                  line2.end());
      if (line2.size() >= 2 && line2[0] == '"' && line2[line2.size() - 1] == '"') {
        std::string filename = line2.substr(1, line2.size() - 2);
        auto includePath = filepath.parent_path() / filename;
        auto [includeCode, includeInfo] = loadGlslCodeWithDebugInfo(includePath);

        lineInfo.insert(lineInfo.end(), includeInfo.begin(), includeInfo.end());
        result += includeCode;
      } else {
        throw std::runtime_error("invalid include: " + line + " in " + filepath.string());
      }
    } else {
      lineInfo.push_back({filepath.string(), lineNum});
      result += line + "\n";
    }
  }
  return {result, lineInfo};
}

// std::string GLSLCompiler::loadGlslCode(fs::path const &filepath) {
//   std::vector<char> charCode = readFile(filepath);
//   std::string code{charCode.begin(), charCode.end()};
//   std::istringstream iss(code);
//   std::string result;

//   for (std::string line; std::getline(iss, line);) {
//     // left trim
//     line.erase(line.begin(),
//                std::find_if(line.begin(), line.end(), [](unsigned char ch) {
//                  return !std::isspace(ch);
//                }));

//     if (line.starts_with("#include") && std::isspace(line[8])) {
//       line = line.substr(8);

//       // lr trim
//       line.erase(line.begin(),
//                  std::find_if(line.begin(), line.end(), [](unsigned char ch) {
//                    return !std::isspace(ch);
//                  }));
//       line.erase(
//           std::find_if(line.rbegin(), line.rend(),
//                        [](unsigned char ch) { return !std::isspace(ch); })
//               .base(),
//           line.end());
//       if (line.size() >= 2 && line[0] == '"' && line[line.size() - 1] == '"') {
//         std::string filename = line.substr(1, line.size() - 2);
//         auto includePath = filepath.parent_path() / filename;
//         result += loadGlslCode(includePath) + "\n";

//       } else {
//         throw std::runtime_error("invalid include: " + line + " in " +
//                                  filepath.string());
//       }
//     } else {
//       result += line + "\n";
//     }
//   }
//   return result;
// }

static TBuiltInResource GetDefaultTBuiltInResource() {
  TBuiltInResource Resources;

  Resources.maxLights = 32;
  Resources.maxClipPlanes = 6;
  Resources.maxTextureUnits = 32;
  Resources.maxTextureCoords = 32;
  Resources.maxVertexAttribs = 64;
  Resources.maxVertexUniformComponents = 4096;
  Resources.maxVaryingFloats = 64;
  Resources.maxVertexTextureImageUnits = 32;
  Resources.maxCombinedTextureImageUnits = 80;
  Resources.maxTextureImageUnits = 32;
  Resources.maxFragmentUniformComponents = 4096;
  Resources.maxDrawBuffers = 32;
  Resources.maxVertexUniformVectors = 128;
  Resources.maxVaryingVectors = 8;
  Resources.maxFragmentUniformVectors = 16;
  Resources.maxVertexOutputVectors = 16;
  Resources.maxFragmentInputVectors = 15;
  Resources.minProgramTexelOffset = -8;
  Resources.maxProgramTexelOffset = 7;
  Resources.maxClipDistances = 8;
  Resources.maxComputeWorkGroupCountX = 65535;
  Resources.maxComputeWorkGroupCountY = 65535;
  Resources.maxComputeWorkGroupCountZ = 65535;
  Resources.maxComputeWorkGroupSizeX = 1024;
  Resources.maxComputeWorkGroupSizeY = 1024;
  Resources.maxComputeWorkGroupSizeZ = 64;
  Resources.maxComputeUniformComponents = 1024;
  Resources.maxComputeTextureImageUnits = 16;
  Resources.maxComputeImageUniforms = 8;
  Resources.maxComputeAtomicCounters = 8;
  Resources.maxComputeAtomicCounterBuffers = 1;
  Resources.maxVaryingComponents = 60;
  Resources.maxVertexOutputComponents = 64;
  Resources.maxGeometryInputComponents = 64;
  Resources.maxGeometryOutputComponents = 128;
  Resources.maxFragmentInputComponents = 128;
  Resources.maxImageUnits = 8;
  Resources.maxCombinedImageUnitsAndFragmentOutputs = 8;
  Resources.maxCombinedShaderOutputResources = 8;
  Resources.maxImageSamples = 0;
  Resources.maxVertexImageUniforms = 0;
  Resources.maxTessControlImageUniforms = 0;
  Resources.maxTessEvaluationImageUniforms = 0;
  Resources.maxGeometryImageUniforms = 0;
  Resources.maxFragmentImageUniforms = 8;
  Resources.maxCombinedImageUniforms = 8;
  Resources.maxGeometryTextureImageUnits = 16;
  Resources.maxGeometryOutputVertices = 256;
  Resources.maxGeometryTotalOutputComponents = 1024;
  Resources.maxGeometryUniformComponents = 1024;
  Resources.maxGeometryVaryingComponents = 64;
  Resources.maxTessControlInputComponents = 128;
  Resources.maxTessControlOutputComponents = 128;
  Resources.maxTessControlTextureImageUnits = 16;
  Resources.maxTessControlUniformComponents = 1024;
  Resources.maxTessControlTotalOutputComponents = 4096;
  Resources.maxTessEvaluationInputComponents = 128;
  Resources.maxTessEvaluationOutputComponents = 128;
  Resources.maxTessEvaluationTextureImageUnits = 16;
  Resources.maxTessEvaluationUniformComponents = 1024;
  Resources.maxTessPatchComponents = 120;
  Resources.maxPatchVertices = 32;
  Resources.maxTessGenLevel = 64;
  Resources.maxViewports = 16;
  Resources.maxVertexAtomicCounters = 0;
  Resources.maxTessControlAtomicCounters = 0;
  Resources.maxTessEvaluationAtomicCounters = 0;
  Resources.maxGeometryAtomicCounters = 0;
  Resources.maxFragmentAtomicCounters = 8;
  Resources.maxCombinedAtomicCounters = 8;
  Resources.maxAtomicCounterBindings = 1;
  Resources.maxVertexAtomicCounterBuffers = 0;
  Resources.maxTessControlAtomicCounterBuffers = 0;
  Resources.maxTessEvaluationAtomicCounterBuffers = 0;
  Resources.maxGeometryAtomicCounterBuffers = 0;
  Resources.maxFragmentAtomicCounterBuffers = 1;
  Resources.maxCombinedAtomicCounterBuffers = 1;
  Resources.maxAtomicCounterBufferSize = 16384;
  Resources.maxTransformFeedbackBuffers = 4;
  Resources.maxTransformFeedbackInterleavedComponents = 64;
  Resources.maxCullDistances = 8;
  Resources.maxCombinedClipAndCullDistances = 8;
  Resources.maxSamples = 4;
  Resources.maxMeshOutputVerticesNV = 256;
  Resources.maxMeshOutputPrimitivesNV = 512;
  Resources.maxMeshWorkGroupSizeX_NV = 32;
  Resources.maxMeshWorkGroupSizeY_NV = 1;
  Resources.maxMeshWorkGroupSizeZ_NV = 1;
  Resources.maxTaskWorkGroupSizeX_NV = 32;
  Resources.maxTaskWorkGroupSizeY_NV = 1;
  Resources.maxTaskWorkGroupSizeZ_NV = 1;
  Resources.maxMeshViewCountNV = 4;

  Resources.limits.nonInductiveForLoops = 1;
  Resources.limits.whileLoops = 1;
  Resources.limits.doWhileLoops = 1;
  Resources.limits.generalUniformIndexing = 1;
  Resources.limits.generalAttributeMatrixVectorIndexing = 1;
  Resources.limits.generalVaryingIndexing = 1;
  Resources.limits.generalSamplerIndexing = 1;
  Resources.limits.generalVariableIndexing = 1;
  Resources.limits.generalConstantMatrixVectorIndexing = 1;

  return Resources;
}

static EShLanguage GetEShLanguage(vk::ShaderStageFlagBits stage) {
  switch (stage) {
  case vk::ShaderStageFlagBits::eVertex:
    return EShLanguage::EShLangVertex;
  case vk::ShaderStageFlagBits::eFragment:
    return EShLanguage::EShLangFragment;
  case vk::ShaderStageFlagBits::eGeometry:
    return EShLanguage::EShLangGeometry;
  case vk::ShaderStageFlagBits::eRaygenKHR:
    return EShLanguage::EShLangRayGen;
  case vk::ShaderStageFlagBits::eMissKHR:
    return EShLanguage::EShLangMiss;
  case vk::ShaderStageFlagBits::eClosestHitKHR:
    return EShLanguage::EShLangClosestHit;
  case vk::ShaderStageFlagBits::eAnyHitKHR:
    return EShLanguage::EShLangAnyHit;
  case vk::ShaderStageFlagBits::eIntersectionKHR:
    return EShLanguage::EShLangIntersect;
  case vk::ShaderStageFlagBits::eCompute:
    return EShLanguage::EShLangCompute;
  default:
    throw std::invalid_argument("GetEShLanguage: invalid shader stage.");
  }
}

static int gInitCount = 0;
void GLSLCompiler::InitializeProcess() {
  if (gInitCount == 0) {
    glslang::InitializeProcess();
  }
  gInitCount++;
}

void GLSLCompiler::FinalizeProcess() {
  gInitCount--;
  if (gInitCount == 0) {
    glslang::FinalizeProcess();
  }
}

std::vector<std::uint32_t>
GLSLCompiler::compileToSpirv(vk::ShaderStageFlagBits shaderStage, std::string const &glslCode,
                             std::vector<std::tuple<std::string, int>> const &debugInfo) {

  EShLanguage language = GetEShLanguage(shaderStage);
  glslang::TShader shader(language);
  const char *codes[1] = {glslCode.c_str()};

  glslang::EShTargetLanguageVersion spvVersion = glslang::EShTargetSpv_1_1;
  if (shaderStage == vk::ShaderStageFlagBits::eRaygenKHR ||
      shaderStage == vk::ShaderStageFlagBits::eMissKHR ||
      shaderStage == vk::ShaderStageFlagBits::eClosestHitKHR ||
      shaderStage == vk::ShaderStageFlagBits::eAnyHitKHR ||
      shaderStage == vk::ShaderStageFlagBits::eIntersectionKHR) {
    spvVersion = glslang::EShTargetSpv_1_4;
  }

  shader.setStrings(codes, 1);
  shader.setEnvInput(glslang::EShSourceGlsl, GetEShLanguage(shaderStage), glslang::EShClientVulkan,
                     110);
  shader.setEnvClient(glslang::EShClientVulkan,
                      glslang::EShTargetClientVersion::EShTargetVulkan_1_1);
  shader.setEnvTarget(glslang::EShTargetSpv, spvVersion);

  TBuiltInResource resource = GetDefaultTBuiltInResource();
  EShMessages messages =
      static_cast<EShMessages>(EShMsgDefault | EShMsgVulkanRules | EShMsgSpvRules);
  bool result = shader.parse(&resource, 110, false, messages);

  {
    std::string log = shader.getInfoLog();
    if (log.length()) {
      // log::error(log);

      std::stringstream ss(log);
      std::string line;
      while (std::getline(ss, line, '\n')) {
        std::regex pattern("^ERROR: ([0-9]+):([0-9]+): '([^']*)' : (.*)$");
        std::smatch sm;
        if (std::regex_match(line, sm, pattern) && sm.size() == 5) {
          uint32_t l = std::stoi(sm[2]) - 1;
          std::string var = sm[3];
          std::string reason = sm[4];
          if (debugInfo.size() > l) {
            auto [file, lineNumber] = debugInfo[l];
            logger::error("ERROR: {}:{}: '{}' :  {}", file, lineNumber, var, reason);
          }
        } else if (line.length()) {
          logger::error(line);
        }
      }
    }
  }

  if (!result) {
    throw std::runtime_error("copmileToSpirv: failed to compile shader");
  }

  glslang::TProgram program;
  program.addShader(&shader);
  result = program.link(messages);
  {
    std::string log = program.getInfoLog();
    if (log.length()) {
      logger::getLogger()->warn(log);
    }
    log = program.getInfoDebugLog();
    if (log.length()) {
      logger::getLogger()->warn(log);
    }
  }
  if (!result) {
    throw std::runtime_error("copmileToSpirv: failed to link shader");
  }

  glslang::TIntermediate *intermediate = program.getIntermediate(language);
  if (!intermediate) {
    throw std::runtime_error("copmileToSpirv: failed to get intermediate");
  }
  spv::SpvBuildLogger logger;

  std::vector<std::uint32_t> spirv;
  glslang::GlslangToSpv(*intermediate, spirv, &logger);

  {
    std::string log = logger.getAllMessages();
    if (log.length()) {
      logger::getLogger()->warn(log);
    }
  }

  return spirv;
}

} // namespace svulkan2