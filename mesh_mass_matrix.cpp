#include <igl/massmatrix.h>
#include <igl/readOFF.h>

#include <Eigen/Core>
#include <iostream>

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

Eigen::VectorXd trimesh_massmatrix(RowMatrixXd V, RowMatrixXi F) {
  Eigen::SparseMatrix<double> M;
  igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_VORONOI, M);

  // std::cout << M << std::endl;

  Eigen::VectorXd mass = M.diagonal();

  return mass;
}

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/elephant.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  Eigen::VectorXd mass = trimesh_massmatrix(V, F);

  std::cout << "Loaded mesh with " << V.rows() << " vertices." << std::endl;
  std::cout << "Mass matrix: " << mass << std::endl;
  return 0;
}