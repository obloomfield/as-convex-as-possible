#pragma once

#include <array>
#include <vector>
#include <unordered_map>

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

    // surface area, calculated during constructor
    unordered_map<int, float> tri_to_area;
    float total_surface_area;



 public:
    Mesh() = default;

    // upon construction:
    // - calculate triangle areas and total surface area
    Mesh(vector<Vector3f> verts, vector<Vector3i> tris) : m_verts(verts), m_triangles(tris) {
        total_surface_area = calc_triangle_areas();
    }
    Mesh(Shape m_shape) : m_verts(m_shape.getVertices()), m_triangles(m_shape.getFaces()) {
        total_surface_area = calc_triangle_areas();
    }

    // Computes the volume of a mesh.
    double volume() const;
    // Computes the convex hull of the mesh. Fast, but can be unstable.
    Mesh computeCH() const;
    // Computes the volumetric(?) convex hull of the mesh. More stable, but slower than CH.
    Mesh computeVCH() const;

    std::vector<Mesh> merge(const std::vector<Mesh>& Q);

    // -------- used for concavity metric -------
    float calc_triangle_areas();
    vector<Vector3f> boundary_sample(int samples_per_unit_area);
    Vector3f random_barycentric_coord(Vector3f& p1, Vector3f& p2, Vector3f& p3);

    float calc_volume();

    //    std::pair<Mesh, Mesh> clip(const Plane& p);
};
