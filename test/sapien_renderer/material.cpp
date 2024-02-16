#include "sapien/sapien_renderer/sapien_renderer.h"
#include <filesystem>
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::sapien_renderer;

TEST(SapienRenderTexture, CreateMaterial) {
  auto meshfile =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "test_texture.png";
  auto texture = std::make_shared<SapienRenderTexture2D>(
      meshfile.string(), 3, SapienRenderTexture2D::FilterMode::eNEAREST,
      SapienRenderTexture2D::AddressMode::eMIRROR);

  auto material = std::make_shared<SapienRenderMaterial>(std::array<float, 4>{0.1, 0.2, 0.3, 0.9},
                                                         std::array<float, 4>{0.4, 0.5, 0.6, 0.8},
                                                         0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
  EXPECT_FLOAT_EQ(std::get<0>(material->getEmission()), 0.1);
  EXPECT_FLOAT_EQ(std::get<1>(material->getEmission()), 0.2);
  EXPECT_FLOAT_EQ(std::get<2>(material->getEmission()), 0.3);
  EXPECT_FLOAT_EQ(std::get<3>(material->getEmission()), 0.9);

  EXPECT_FLOAT_EQ(std::get<0>(material->getBaseColor()), 0.4);
  EXPECT_FLOAT_EQ(std::get<1>(material->getBaseColor()), 0.5);
  EXPECT_FLOAT_EQ(std::get<2>(material->getBaseColor()), 0.6);
  EXPECT_FLOAT_EQ(std::get<3>(material->getBaseColor()), 0.8);

  EXPECT_FLOAT_EQ(material->getSpecular(), 0.1);
  EXPECT_FLOAT_EQ(material->getRoughness(), 0.2);
  EXPECT_FLOAT_EQ(material->getMetallic(), 0.3);
  EXPECT_FLOAT_EQ(material->getTransmission(), 0.4);
  EXPECT_FLOAT_EQ(material->getIOR(), 0.5);
  EXPECT_FLOAT_EQ(material->getTransmissionRoughness(), 0.6);

  EXPECT_EQ(material->getEmissionTexture(), nullptr);
  EXPECT_EQ(material->getDiffuseTexture(), nullptr);
  EXPECT_EQ(material->getRoughnessTexture(), nullptr);
  EXPECT_EQ(material->getMetallicTexture(), nullptr);
  EXPECT_EQ(material->getNormalTexture(), nullptr);
  EXPECT_EQ(material->getTransmissionTexture(), nullptr);

  // setters

  material->setEmission({0.2, 0.3, 0.4, 0.8});
  EXPECT_FLOAT_EQ(std::get<0>(material->getEmission()), 0.2);
  EXPECT_FLOAT_EQ(std::get<1>(material->getEmission()), 0.3);
  EXPECT_FLOAT_EQ(std::get<2>(material->getEmission()), 0.4);
  EXPECT_FLOAT_EQ(std::get<3>(material->getEmission()), 0.8);

  material->setBaseColor({0.5, 0.6, 0.7, 0.9});
  EXPECT_FLOAT_EQ(std::get<0>(material->getBaseColor()), 0.5);
  EXPECT_FLOAT_EQ(std::get<1>(material->getBaseColor()), 0.6);
  EXPECT_FLOAT_EQ(std::get<2>(material->getBaseColor()), 0.7);
  EXPECT_FLOAT_EQ(std::get<3>(material->getBaseColor()), 0.9);

  material->setSpecular(0.5);
  EXPECT_FLOAT_EQ(material->getSpecular(), 0.5);
  material->setRoughness(0.6);
  EXPECT_FLOAT_EQ(material->getRoughness(), 0.6);
  material->setMetallic(0.7);
  EXPECT_FLOAT_EQ(material->getMetallic(), 0.7);
  material->setTransmission(0.8);
  EXPECT_FLOAT_EQ(material->getTransmission(), 0.8);
  material->setIOR(0.9);
  EXPECT_FLOAT_EQ(material->getIOR(), 0.9);
  material->setTransmissionRoughness(0.1);
  EXPECT_FLOAT_EQ(material->getTransmissionRoughness(), 0.1);

  material->setEmissionTexture(texture);
  ASSERT_TRUE(material->getEmissionTexture());
  EXPECT_EQ(material->getEmissionTexture()->getTexture(), texture->getTexture());
  material->setEmissionTexture(nullptr);
  EXPECT_EQ(material->getEmissionTexture(), nullptr);

  material->setDiffuseTexture(texture);
  ASSERT_TRUE(material->getDiffuseTexture());
  EXPECT_EQ(material->getDiffuseTexture()->getTexture(), texture->getTexture());
  material->setDiffuseTexture(nullptr);
  EXPECT_EQ(material->getDiffuseTexture(), nullptr);

  material->setRoughnessTexture(texture);
  ASSERT_TRUE(material->getRoughnessTexture());
  EXPECT_EQ(material->getRoughnessTexture()->getTexture(), texture->getTexture());
  material->setRoughnessTexture(nullptr);
  EXPECT_EQ(material->getRoughnessTexture(), nullptr);

  material->setMetallicTexture(texture);
  ASSERT_TRUE(material->getMetallicTexture());
  EXPECT_EQ(material->getMetallicTexture()->getTexture(), texture->getTexture());
  material->setMetallicTexture(nullptr);
  EXPECT_EQ(material->getMetallicTexture(), nullptr);

  material->setNormalTexture(texture);
  ASSERT_TRUE(material->getNormalTexture());
  EXPECT_EQ(material->getNormalTexture()->getTexture(), texture->getTexture());
  material->setNormalTexture(nullptr);
  EXPECT_EQ(material->getNormalTexture(), nullptr);

  material->setTransmissionTexture(texture);
  ASSERT_TRUE(material->getTransmissionTexture());
  EXPECT_EQ(material->getTransmissionTexture()->getTexture(), texture->getTexture());
  material->setTransmissionTexture(nullptr);
  EXPECT_EQ(material->getTransmissionTexture(), nullptr);
}
