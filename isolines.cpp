#include <igl/isolines.h>
#include <igl/readOFF.h>

#include <Eigen/Core>
#include <iostream>

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

std::tuple<RowMatrixXd, RowMatrixXi, RowMatrixXi> trimesh_isolines(
    RowMatrixXd V,         // by dim list of mesh vertex positions
    RowMatrixXi F,         // by 3 list of mesh triangle indices into some V
    Eigen::VectorXd vals)  // List of values for isolines
{
  RowMatrixXd iV;  // iV by dim list of isoline vertex positions
  RowMatrixXi iE;  // iE by 2 list of edge indices into iV
  Eigen::VectorXi
      I;  // ieE by 1 list of indices into vals indicating which value

  igl::isolines(V, F, V.col(1).eval(), vals, iV, iE, I);

  return std::make_tuple(iV, iE, I);
}

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/elephant.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }
  RowMatrixXd iV;
  RowMatrixXi iE;
  Eigen::VectorXi I;
  Eigen::VectorXd vals(3);
  vals << -0.5, 0, 0.5;
  igl::isolines(V, F, V.col(1).eval(), vals, iV, iE, I);
  std::cout << "iV: " << iV << std::endl;
  std::cout << "iE: " << iE << std::endl;
  std::cout << "I: " << I << std::endl;
  return 0;
}