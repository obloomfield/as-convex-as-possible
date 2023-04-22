#pragma once

#include <array>
#include <vector>

#include "Eigen/Dense"
#include "btConvexHull/btConvexHullComputer.h"
#include "convex-hull/convex-hull.h"
#include "graphics/shape.h"
#include "quickhull/QuickHull.hpp"

using namespace std;
using namespace Eigen;

class Mesh {
 private:
    vector<Vector3f> m_verts;
    vector<Vector3i> m_triangles;

    // for Monte-Carlo Tree Search
    array<double, 6> m_bbox;
    // for Eigenvalue computations
    array<double, 3> m_barycenter;
    // for optional PCA
    array<array<double, 3>, 3> m_rot;

 public:
    Mesh() = default;
    Mesh(vector<Vector3f> verts, vector<Vector3i> tris) : m_verts(verts), m_triangles(tris) {}
    Mesh(Shape m_shape) : m_verts(m_shape.getVertices()), m_triangles(m_shape.getFaces()) {}

    // Computes the volume of a mesh.
    double volume() const;

    // Computes the convex hull of the mesh. Fast, but can be unstable.
    Mesh computeCH() const;
    // Computes the volumetric(?) convex hull of the mesh. More stable, but slower than CH.
    Mesh computeVCH() const;

    std::vector<Mesh> merge(const std::vector<Mesh>& Q);

    //    std::pair<Mesh, Mesh> clip(const Plane& p);
};
