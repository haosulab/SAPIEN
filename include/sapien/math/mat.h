#include <Eigen/Dense>

namespace sapien {

// TODO: implement our own matrix class?
using Mat4 = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;
using Mat3 = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;
using Mat34 = Eigen::Matrix<float, 3, 4, Eigen::RowMajor>;

} // namespace sapien
