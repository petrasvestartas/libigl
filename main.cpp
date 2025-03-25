#include <igl/readOFF.h>
#include <iostream>
#include <Eigen/Core>

int main() {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOFF("example.off", V, F);
    std::cout << "Loaded mesh with " << V.rows() << " vertices." << std::endl;
    return 0;
}
