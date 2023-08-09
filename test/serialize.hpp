#pragma once
#include <cereal/archives/binary.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/vector.hpp>

template <typename T> static T saveload(T const &data) {
  std::ostringstream os;
  {
    cereal::BinaryOutputArchive archive(os);
    archive(data);
    archive.serializeDeferments();
  }

  std::istringstream is(os.str());
  {
    cereal::BinaryInputArchive archive(is);
    T data;
    archive(data);
    return data;
  }
}
