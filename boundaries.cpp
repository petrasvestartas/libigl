
#include <igl/boundary_loop.h>
#include <igl/readOFF.h>

#include <Eigen/StdVector>
#include <iostream>

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/beetle.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  std::vector<std::vector<int>> L;
  igl::boundary_loop(F, L);

  std::cout << "Number of boundary loops: " << L.size() << std::endl;
  for (const auto &loop : L) {
    std::cout << "Loop: ";
    for (int v : loop) {
      std::cout << v << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "Loaded mesh with " << V.rows() << " vertices." << std::endl;
  return 0;
}
