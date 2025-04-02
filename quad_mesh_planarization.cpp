#include <igl/planarize_quad_mesh.h>
#include <igl/readOFF.h>
#include <igl/writeOBJ.h>

#include <Eigen/Core>
#include <iostream>

int main() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  
  // Load mesh
  if (!igl::readOFF("../data/tubemesh.off", V, F)) {  
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  std::cout << "Loaded mesh with " << V.rows() << " vertices and " << F.rows() << " faces." << std::endl;

  // Planarize quad mesh
  Eigen::MatrixXd Vplanar = V;
  const double threshold = 0.005;
  const int maxiter = 100;
  igl::planarize_quad_mesh(V, F, maxiter, threshold, Vplanar);

  std::cout << "Planarization complete." << std::endl;

  // Save result
  igl::writeOBJ("../planarized.obj", Vplanar, F);  
  std::cout << "Saved result to planarized.obj" << std::endl;

  return 0;
}