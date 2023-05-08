#include "shapes.h"

float BOUNDS_PADDING = 1.f;

Plane::Plane(const quickhull::Plane<double>& p, const Mesh& m) {
    // Convert plane types
    Vector3d planeNormal(p.m_N.x, p.m_N.y, p.m_N.z);
    Vector3d planePoint = -p.m_D * planeNormal;

    // Get the minimum and maximum coordinates of the bounding box
    auto [a,b,c,x,y,z] = m.bounding_box();
    Vector3d minCoords(a,b,c);
    Vector3d maxCoords(x,y,z);

    // Define the four corner points of the bounding region plane
    p0 = minCoords - BOUNDS_PADDING * Vector3d::Ones();
    p1 = minCoords - BOUNDS_PADDING * Vector3d::UnitX() + (maxCoords.x() - minCoords.x() + 2 * BOUNDS_PADDING) * Vector3d::UnitX();
    p2 = minCoords - BOUNDS_PADDING * Vector3d::UnitY() + (maxCoords.y() - minCoords.y() + 2 * BOUNDS_PADDING) * Vector3d::UnitY();
    p3 = planePoint + (p0 - planePoint).dot(planeNormal) / planeNormal.dot(p0 - p3) * (p3 - p0);
}
