#include "svulkan2/common/fs.h"
#include "svulkan2/shader/glsl_compiler.h"
#include <fstream>

using namespace svulkan2;
namespace fs = std::filesystem;

bool checkVariableName(std::string const &name) {
  if (name.empty()) {
    return false;
  }

  if (!std::isalpha(name[0]) && name[0] != '_') {
    return false;
  }

  for (char c : name) {
    if (!std::isalnum(c) && c != '_') {
      return false;
    }
  }

  return true;
}

std::string hexembed(const unsigned char *data, int size, std::string name) {
  std::ostringstream ss;
  ss << "static const int " << name << "_size = " << size << ";\n";
  ss << "static const unsigned char " << name << "_code[" << name << "_size]"
     << " = {\n";

  for (int i = 0; i < size; ++i) {
    char buf[16]{};
    std::sprintf(buf, "0x%02x%s", data[i], i == size - 1 ? "" : ((i + 1) % 16 == 0 ? ",\n" : ","));
    ss << buf;
  }
  ss << "};\n";

  return ss.str();
}

char *getCmdOption(char **begin, char **end, const std::string &option) {
  char **itr = std::find(begin, end, option);
  if (itr != end && ++itr != end) {
    return *itr;
  }
  return 0;
}

bool cmdOptionExists(char **begin, char **end, const std::string &option) {
  return std::find(begin, end, option) != end;
}

int main(int argc, char *argv[]) {
  std::string outFile = "";
  std::string inFile = "";

  if (auto res = getCmdOption(argv, argv + argc, "-o")) {
    outFile = res;
  } else {
    throw std::runtime_error("no output file specified");
  }
  if (auto res = getCmdOption(argv, argv + argc, "-i")) {
    inFile = res;
  } else {
    throw std::runtime_error("no input file specified");
  }

  GLSLCompiler::InitializeProcess();

  vk::ShaderStageFlagBits stage;

  auto inPath = fs::path(inFile);

  // rasterization
  if (inPath.extension() == ".vert") {
    stage = vk::ShaderStageFlagBits::eVertex;
  } else if (inPath.extension() == ".frag") {
    stage = vk::ShaderStageFlagBits::eFragment;
  } else if (inPath.extension() == ".geom") {
    stage = vk::ShaderStageFlagBits::eGeometry;
  }

  // ray tracing
  else if (inPath.extension() == ".rchit") {
    stage = vk::ShaderStageFlagBits::eClosestHitKHR;
  } else if (inPath.extension() == ".rahit") {
    stage = vk::ShaderStageFlagBits::eAnyHitKHR;
  } else if (inPath.extension() == ".rgen") {
    stage = vk::ShaderStageFlagBits::eRaygenKHR;
  } else if (inPath.extension() == ".rmiss") {
    stage = vk::ShaderStageFlagBits::eMissKHR;
  }

  // compute
  else if (inPath.extension() == ".comp") {
    stage = vk::ShaderStageFlagBits::eCompute;
  }

  // ???
  else {
    throw std::runtime_error("invalid input: the file must use of the following extensions: "
                             ".vert, .frag, .geom, .rchit, .rahit, .rgen, .rmiss, .comp");
  }

  auto name = inPath.stem().string();

  auto code = readFile(inPath);
  auto spv = GLSLCompiler::compileToSpirv(stage, {code.begin(), code.end()});

  if (outFile.ends_with(".spv")) {
    std::ofstream f(outFile, std::ios::binary | std::ios::out);
    f.write(reinterpret_cast<const char *>(spv.data()), spv.size() * sizeof(uint32_t));
  } else if (outFile.ends_with(".h") || outFile.ends_with(".hpp")) {
    if (!checkVariableName(name)) {
      throw std::runtime_error(
          "invalid input: the stem of the file name should be a valid c variable filename");
    }
    std::ofstream f(outFile, std::ios::binary | std::ios::out);
    auto result = hexembed(reinterpret_cast<const unsigned char *>(spv.data()),
                           spv.size() * sizeof(uint32_t), name);
    f.write(result.data(), result.length());
  } else {
    throw std::runtime_error("invalid output file: the file must end with one of: .spv, .h, .hpp");
  }

  GLSLCompiler::FinalizeProcess();
}
