#include <igl/avg_edge_length.h>
#include <igl/exact_geodesic.h>
#include <igl/heat_geodesics.h>
#include <igl/readOFF.h>

#include <iostream>

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

Eigen::VectorXd trimesh_geodistance_exact(RowMatrixXd V, RowMatrixXi F,
                                          int vid) {
  Eigen::VectorXd D;
  Eigen::VectorXi VS, FS, VT, FT;

  VS.resize(1);
  VS << vid;

  VT.setLinSpaced(V.rows(), 0, V.rows() - 1);

  igl::exact_geodesic(V, F, VS, FS, VT, FT, D);

  return D;
}

Eigen::VectorXd trimesh_geodistance_heat(RowMatrixXd V, RowMatrixXi F,
                                         int vid) {
  Eigen::VectorXi gamma;
  gamma.resize(1);
  gamma << vid;

  igl::HeatGeodesicsData<double> data;
  double t = std::pow(igl::avg_edge_length(V, F), 2);
  igl::heat_geodesics_precompute(V, F, t, data);

  Eigen::VectorXd D = Eigen::VectorXd::Zero(data.Grad.cols());
  D(vid) = 1;

  igl::heat_geodesics_solve(data, gamma, D);

  return D;
}

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/camelhead.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  // Must be triangle mesh
  if (F.cols() != 3) {
    std::cerr << "Error: Mesh must be triangle mesh." << std::endl;
    return -1;
  }

  Eigen::VectorXd D1 = trimesh_geodistance_exact(V, F, 0);
  Eigen::VectorXd D2 = trimesh_geodistance_heat(V, F, 0);

  std::cout << "Exact geodesic distance: " << D1 << std::endl;
  std::cout << "Heat geodesic distance: " << D2 << std::endl;

  std::cout << "Loaded mesh with " << V.rows() << " vertices." << std::endl;
  return 0;
}