#pragma once
#include "sapien/array.h"

namespace sapien {
namespace component {

std::string getFormatDescriptor(vk::Format format);
int getFormatChannelSize(vk::Format format);
int getFormatChannels(vk::Format format);
std::string getFormatTypestr(vk::Format format);

struct SapienRenderImageCpu : public CpuArrayHandle {
  vk::Format format;

  SapienRenderImageCpu(int width, int height, vk::Format format, void *ptr);
};

struct SapienRenderImageCuda : public CudaArrayHandle {
  vk::Format format;

  SapienRenderImageCuda(int width, int height, vk::Format format, void *ptr, int cudaId);
};

} // namespace component
} // namespace sapien
