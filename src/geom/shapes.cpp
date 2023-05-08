#include "shapes.h"

#include "mesh.h"

using namespace Eigen;

float BOUNDS_PADDING = 1.f;

Plane::Plane(Edge e, const Vector3d &norm, array<double, 6> bbox) {
    // Get the minimum and maximum coordinates of the bounding box
    auto [a, b, c, x, y, z] = bbox;
    Vector3d minCoords(a, b, c);
    Vector3d maxCoords(x, y, z);

    // Get diagonal distance between bounding box
    auto dist_diag = dist(minCoords, maxCoords);

    // Get direction vector
    auto n = (e[1] - e[0]).normalized();

    // Scale edge endpoints by that distance, in the appropriate direction
    e[0] -= dist_diag * n;
    e[1] += dist_diag * n;

    // Take those endpoints, and add/subtract the normal in which we wish to create the plane
    p0 = e[0] + dist_diag * norm;
    p1 = e[0] - dist_diag * norm;
    p2 = e[1] - dist_diag * norm;
    p3 = e[1] + dist_diag * norm;

    // ensure that all points make a plane
    double EPSILON = 1e-9;
    Eigen::Matrix<double, 3, 3> mat;
    mat << (p1 - p0), (p2 - p0), (p3 - p0);
    assert(mat.determinant() < EPSILON);
}

Plane::Plane(const quickhull::Plane<double> &p, std::array<double, 6> bbox) {
    // Convert plane types
    Vector3d planeNormal(p.m_N.x, p.m_N.y, p.m_N.z);
    Vector3d planePoint = -p.m_D * planeNormal;

    // Get the minimum and maximum coordinates of the bounding box
    auto [a, b, c, x, y, z] = bbox;
    Vector3d minCoords(a, b, c);
    Vector3d maxCoords(x, y, z);

    // Define the four corner points of the bounding region plane
    p0 = minCoords - BOUNDS_PADDING * Vector3d::Ones();
    p1 = minCoords - BOUNDS_PADDING * Vector3d::UnitX() +
         (maxCoords.x() - minCoords.x() + 2 * BOUNDS_PADDING) * Vector3d::UnitX();
    p2 = minCoords - BOUNDS_PADDING * Vector3d::UnitY() +
         (maxCoords.y() - minCoords.y() + 2 * BOUNDS_PADDING) * Vector3d::UnitY();
    p3 = planePoint + (p0 - planePoint).dot(planeNormal) / planeNormal.dot(p0 - p3) * (p3 - p0);
}
