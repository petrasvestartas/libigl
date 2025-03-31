
#include <igl/gaussian_curvature.h>
#include <igl/principal_curvature.h>
#include <igl/readOFF.h>

#include <iostream>

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

Eigen::VectorXd trimesh_gaussian_curvature(RowMatrixXd V, RowMatrixXi F) {
  Eigen::VectorXd C;
  igl::gaussian_curvature(V, F, C);

  return C;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
trimesh_principal_curvature(RowMatrixXd V, RowMatrixXi F) {
  if (F.cols() != 3) {
    std::cerr << "Error: Principal curvature requires triangular faces."
              << std::endl;
    return std::make_tuple(Eigen::MatrixXd(), Eigen::MatrixXd(),
                           Eigen::VectorXd(), Eigen::VectorXd());
  }

  Eigen::MatrixXd PD1;
  Eigen::MatrixXd PD2;
  Eigen::VectorXd PV1;
  Eigen::VectorXd PV2;

  std::vector<int> bad_vertices;

  igl::principal_curvature(V, F, PD1, PD2, PV1, PV2, bad_vertices);

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
      result = std::make_tuple(PD1, PD2, PV1, PV2);

  return result;
}

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/elephant.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  std::cout << "Vertices (V): " << V.rows() << " x " << V.cols() << std::endl;
  std::cout << "Faces (F): " << F.rows() << " x " << F.cols() << std::endl;
  Eigen::VectorXd C = trimesh_gaussian_curvature(V, F);
  std::cout << "Gaussian curvature: " << C << std::endl;

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
      result = trimesh_principal_curvature(V, F);
  std::cout << "Principal curvatures: " << std::endl;
  std::cout << "PD1: " << std::endl;
  std::cout << std::get<0>(result) << std::endl;
  std::cout << "PD2: " << std::endl;
  std::cout << std::get<1>(result) << std::endl;
  std::cout << "PV1: " << std::endl;
  std::cout << std::get<2>(result) << std::endl;
  std::cout << "PV2: " << std::endl;
  std::cout << std::get<3>(result) << std::endl;

  std::cout << "Loaded mesh with " << V.rows() << " vertices." << std::endl;
  return 0;
}
