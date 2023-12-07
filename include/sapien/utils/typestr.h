#pragma once
#include <string>

namespace sapien {

inline int typestrBytes(std::string const &type) {
  if (type.at(0) == '<' || type.at(0) == '>' || type.at(0) == '|') {
    return std::stoi(type.substr(2));
  }
  return std::stoi(type.substr(1));
}

inline char typestrCode(std::string const &type) {
  if (type.at(0) == '<' || type.at(0) == '>' || type.at(0) == '|') {
    return type.at(1);
  }
  return type.at(0);
}

} // namespace sapien
