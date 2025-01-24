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
#include "sapien/sapien_renderer/material.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"

namespace sapien {
namespace sapien_renderer {

SapienRenderMaterial::SapienRenderMaterial()
    : SapienRenderMaterial({0.f, 0.f, 0.f, 1.f}, {1.f, 1.f, 1.f, 1.f}, 0.f, 1.f, 0.f, 0.f, 1.45f,
                           0.f) {}

SapienRenderMaterial::SapienRenderMaterial(std::array<float, 4> emission,
                                           std::array<float, 4> baseColor, float fresnel,
                                           float roughness, float metallic, float transmission,
                                           float ior, float transmissionRoughness) {
  mEngine = SapienRenderEngine::Get();
  mMaterial = std::make_shared<svulkan2::resource::SVMetallicMaterial>(
      glm::vec4{emission[0], emission[1], emission[2], emission[3]},
      glm::vec4{baseColor[0], baseColor[1], baseColor[2], baseColor[3]}, fresnel, roughness,
      metallic, transmission, ior, transmissionRoughness);
}

SapienRenderMaterial::SapienRenderMaterial(
    std::shared_ptr<svulkan2::resource::SVMetallicMaterial> material) {
  if (!material) {
    throw std::runtime_error("invalid material");
  }
  mEngine = SapienRenderEngine::Get();
  mMaterial = material;
}

void SapienRenderMaterial::setEmission(std::array<float, 4> color) {
  mMaterial->setEmission({color[0], color[1], color[2], color[3]});
}
std::array<float, 4> SapienRenderMaterial::getEmission() const {
  auto color = mMaterial->getEmission();
  return {color.r, color.g, color.b, color.a};
}

void SapienRenderMaterial::setBaseColor(std::array<float, 4> color) {
  mMaterial->setBaseColor({color[0], color[1], color[2], color[3]});
}
std::array<float, 4> SapienRenderMaterial::getBaseColor() const {
  auto color = mMaterial->getBaseColor();
  return {color.r, color.g, color.b, color.a};
}

void SapienRenderMaterial::setRoughness(float roughness) { mMaterial->setRoughness(roughness); }
float SapienRenderMaterial::getRoughness() const { return mMaterial->getRoughness(); }
void SapienRenderMaterial::setSpecular(float specular) { mMaterial->setFresnel(specular); }
float SapienRenderMaterial::getSpecular() const { return mMaterial->getFresnel(); }
void SapienRenderMaterial::setMetallic(float metallic) { mMaterial->setMetallic(metallic); }
float SapienRenderMaterial::getMetallic() const { return mMaterial->getMetallic(); }
void SapienRenderMaterial::setIOR(float ior) { mMaterial->setIor(ior); }
float SapienRenderMaterial::getIOR() const { return mMaterial->getIor(); }
void SapienRenderMaterial::setTransmission(float transmission) {
  mMaterial->setTransmission(transmission);
}
float SapienRenderMaterial::getTransmission() const { return mMaterial->getTransmission(); }
void SapienRenderMaterial::setTransmissionRoughness(float roughness) {
  mMaterial->setTransmissionRoughness(roughness);
}
float SapienRenderMaterial::getTransmissionRoughness() const {
  return mMaterial->getTransmissionRoughness();
}

void SapienRenderMaterial::setEmissionTexture(std::shared_ptr<SapienRenderTexture2D> texture) {
  mMaterial->setEmissionTexture(texture ? texture->getTexture() : nullptr);
}

std::shared_ptr<SapienRenderTexture2D> SapienRenderMaterial::getEmissionTexture() const {
  auto tex = mMaterial->getEmissionTexture();
  return tex ? std::make_shared<SapienRenderTexture2D>(tex) : nullptr;
}

void SapienRenderMaterial::setBaseColorTexture(std::shared_ptr<SapienRenderTexture2D> texture) {
  mMaterial->setBaseColorTexture(texture ? texture->getTexture() : nullptr);
}
std::shared_ptr<SapienRenderTexture2D> SapienRenderMaterial::getBaseColorTexture() const {
  auto tex = mMaterial->getBaseColorTexture();
  return tex ? std::make_shared<SapienRenderTexture2D>(tex) : nullptr;
}

void SapienRenderMaterial::setNormalTexture(std::shared_ptr<SapienRenderTexture2D> texture) {
  mMaterial->setNormalTexture(texture ? texture->getTexture() : nullptr);
}
std::shared_ptr<SapienRenderTexture2D> SapienRenderMaterial::getNormalTexture() const {
  auto tex = mMaterial->getNormalTexture();
  return tex ? std::make_shared<SapienRenderTexture2D>(tex) : nullptr;
}

void SapienRenderMaterial::setRoughnessTexture(std::shared_ptr<SapienRenderTexture2D> texture) {
  mMaterial->setRoughnessTexture(texture ? texture->getTexture() : nullptr);
}
std::shared_ptr<SapienRenderTexture2D> SapienRenderMaterial::getRoughnessTexture() const {
  auto tex = mMaterial->getRoughnessTexture();
  return tex ? std::make_shared<SapienRenderTexture2D>(tex) : nullptr;
}

void SapienRenderMaterial::setMetallicTexture(std::shared_ptr<SapienRenderTexture2D> texture) {
  mMaterial->setMetallicTexture(texture ? texture->getTexture() : nullptr);
}
std::shared_ptr<SapienRenderTexture2D> SapienRenderMaterial::getMetallicTexture() const {
  auto tex = mMaterial->getMetallicTexture();
  return tex ? std::make_shared<SapienRenderTexture2D>(tex) : nullptr;
}

void SapienRenderMaterial::setTransmissionTexture(std::shared_ptr<SapienRenderTexture2D> texture) {
  mMaterial->setTransmissionTexture(texture ? texture->getTexture() : nullptr);
}
std::shared_ptr<SapienRenderTexture2D> SapienRenderMaterial::getTransmissionTexture() const {
  auto tex = mMaterial->getTransmissionTexture();
  return tex ? std::make_shared<SapienRenderTexture2D>(tex) : nullptr;
}

} // namespace sapien_renderer
} // namespace sapien
