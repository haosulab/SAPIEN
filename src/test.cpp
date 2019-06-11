#include <fstream>
#include <sstream>
#include <iostream>

int main() {
  std::ifstream f("/home/fx/mobility_mesh/resources/46437-4/objs/original-1.obj");
  std::string line;

  std::string t;
  float a, b, c;
  while (std::getline(f, line)) {
    if (line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    iss >> t;
    if (t == "v") {
      iss >> a >> b >> c;
      std::cout << t << " " << a << " " << b << " " << c << std::endl;
    }
  }
}
