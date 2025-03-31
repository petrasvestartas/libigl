#include <igl/Hit.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/readOFF.h>

#include <iostream>

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

using HitList = std::vector<igl::Hit<double>>;

HitList intersection_ray_mesh(Eigen::Vector3d point, Eigen::Vector3d direction,
                              RowMatrixXd V, RowMatrixXi F) {
  HitList hits;
  std::vector<igl::Hit<double>> igl_hits;

  bool result = igl::ray_mesh_intersect(point, direction, V, F, igl_hits);

  if (result) {
    for (const auto &hit : igl_hits) {
      hits.push_back(hit);  // Directly copying the hit
    }
  }

  return hits;
}

std::vector<HitList> intersection_rays_mesh(RowMatrixXd points,
                                            RowMatrixXd directions,
                                            RowMatrixXd V, RowMatrixXi F) {
  std::vector<HitList> hits_per_ray;

  int r = points.rows();

  for (int i = 0; i < r; i++) {
    std::vector<igl::Hit<double>> igl_hits;
    bool result = igl::ray_mesh_intersect(points.row(i), directions.row(i), V,
                                          F, igl_hits);

    HitList hits;
    if (result) {
      hits = igl_hits;  // Directly assign the result
    }
    hits_per_ray.push_back(hits);
  }

  return hits_per_ray;
}

int single_ray() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Load a mesh from an OFF file
  if (!igl::readOFF("../data/elephant.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  // Define a single ray
  Eigen::Vector3d origin =
      V.colwise().mean();  // Use mesh centroid as starting point
  origin[2] += 1.0;        // Move slightly above the mesh

  Eigen::Vector3d direction(0, 0, -1);  // Shoot ray downward in -Z direction

  // Call the intersection function
  HitList hits = intersection_ray_mesh(origin, direction, V, F);

  // Print results
  if (!hits.empty()) {
    std::cout << "Ray intersects the mesh at:" << std::endl;
    for (const auto &hit : hits) {
      // Retrieve the vertices of the intersected triangle
      Eigen::Vector3d A = V.row(F(hit.id, 0));
      Eigen::Vector3d B = V.row(F(hit.id, 1));
      Eigen::Vector3d C = V.row(F(hit.id, 2));

      // Compute the exact intersection point using barycentric coordinates
      Eigen::Vector3d intersection =
          (1 - hit.u - hit.v) * A + hit.u * B + hit.v * C;

      std::cout << "  - Face ID: " << hit.id
                << ", Intersection Point: " << intersection.transpose()
                << ", Distance (t): " << hit.t << std::endl;
    }
  } else {
    std::cout << "No intersection detected." << std::endl;
  }

  return 0;
}

int multiple_rays() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (!igl::readOFF("../data/elephant.off", V, F)) {
    std::cerr << "Error loading mesh from file." << std::endl;
    return -1;
  }

  // Define the base point (centroid of the mesh)
  Eigen::RowVector3d base = V.colwise().mean();
  base[2] = 0;  // Set z-coordinate to 0

  // Generate rays
  double r = 1.0;
  RowMatrixXd points;
  RowMatrixXd directions;
  int num_rays = 20;
  points.resize(num_rays * num_rays, 3);
  directions.resize(num_rays * num_rays, 3);

  int index = 0;
  for (int i = 0; i < num_rays; ++i) {
    double theta =
        static_cast<double>(i) / num_rays * M_PI;  // Angle in the range [0, pi]
    for (int j = 0; j < num_rays; ++j) {
      double phi = static_cast<double>(j) / num_rays * 2 *
                   M_PI;  // Angle in the range [0, 2*pi]
      double x = r * sin(theta) * cos(phi) + base[0];
      double y = r * sin(theta) * sin(phi) + base[1];
      double z = r * cos(theta);

      // Create direction vector
      Eigen::RowVector3d direction(x - base[0], y - base[1], z - base[2]);
      direction.normalize();  // Normalize the direction vector

      points.row(index) = base;           // Store base point
      directions.row(index) = direction;  // Store direction
      index++;
    }
  }

  // Compute ray-mesh intersections
  std::vector<HitList> results =
      intersection_rays_mesh(points, directions, V, F);

  // Print results
  for (size_t i = 0; i < results.size(); ++i) {
    if (!results[i].empty()) {
      std::cout << "Ray " << i << " intersects at:" << std::endl;
      for (const auto &hit : results[i]) {
        // Retrieve triangle vertices
        Eigen::RowVector3d A = V.row(F(hit.id, 0));
        Eigen::RowVector3d B = V.row(F(hit.id, 1));
        Eigen::RowVector3d C = V.row(F(hit.id, 2));

        // Compute intersection point
        Eigen::RowVector3d intersection =
            (1 - hit.u - hit.v) * A + hit.u * B + hit.v * C;

        std::cout << "  - Face ID: " << hit.id
                  << ", Intersection Point: " << intersection.transpose()
                  << ", Distance (t): " << hit.t << std::endl;
      }
    }
  }

  return 0;
}

int main() {
  return single_ray();
  // return multiple_rays();
}
