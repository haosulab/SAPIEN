#include <string>
#include <cstring>

namespace sapien {
inline char *newNameFromString(std::string const &name) {
  char *cname = new char[name.length() + 1];
  strcpy(cname, name.c_str());
  return cname;
}
} // namespace sapien
