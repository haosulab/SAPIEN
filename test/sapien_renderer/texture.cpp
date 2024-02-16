#include "sapien/sapien_renderer/sapien_renderer.h"
#include <filesystem>
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::sapien_renderer;

TEST(SapienRenderTexture, CreateFromFile) {
  auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                  "assets" / "test_texture.png";
  auto texture = std::make_shared<SapienRenderTexture2D>(
      meshfile.string(), 3, SapienRenderTexture2D::FilterMode::eNEAREST,
      SapienRenderTexture2D::AddressMode::eMIRROR);
  ASSERT_TRUE(texture->getTexture());
  EXPECT_EQ(texture->getMipmapLevels(), 3);
  EXPECT_EQ(texture->getWidth(), 64);
  EXPECT_EQ(texture->getHeight(), 32);
  EXPECT_EQ(texture->getChannels(), 4);
  EXPECT_EQ(texture->getFormat(), vk::Format::eR8G8B8A8Unorm);
  EXPECT_EQ(texture->getFilterMode(), SapienRenderTexture2D::FilterMode::eNEAREST);
  EXPECT_EQ(texture->getAddressMode(), SapienRenderTexture2D::AddressMode::eMIRROR);
}
