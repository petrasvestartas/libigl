#include <igl/readOFF.h>

#include <Eigen/Core>
#include <iostream>

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/elephant.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }
  std::cout << "Loaded mesh with " << V.rows() << " vertices." << std::endl;
  return 0;
}