#include <igl/planarize_quad_mesh.h>
#include <igl/readOFF.h>

#include <Eigen/Core>
#include <iostream>

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

RowMatrixXd planarize_quads(RowMatrixXd V, RowMatrixXi F, int maxiter = 100, double threshold = 0.005) {
  RowMatrixXd Vplanar;

  igl::planarize_quad_mesh(V, F, maxiter, threshold, Vplanar);

  return Vplanar;
}

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/tubemesh.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  Eigen::MatrixXd Vplanar = planarize_quads(V, F);
  std::cout << "Vplanar: " << Vplanar << std::endl;

  std::cout << "Loaded mesh with " << V.rows() << " vertices." << std::endl;
  return 0;
}