#pragma once

#include <array>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "btConvexHull/btConvexHullComputer.h"
#include "graphics/shape.h"
#include "plane.h"
#include "quickhull/QuickHull.hpp"

using namespace std;
using namespace Eigen;

class Mesh {
 public:
    // Vertices/triangles information.
    vector<Vector3f> m_verts;
    vector<Vector3i> m_triangles;

    // Default constructor.
    Mesh() = default;

    // Constructs a mesh from a list of vertices and triangles. Initializes total surface area.
    Mesh(vector<Vector3f> verts, vector<Vector3i> tris) : m_verts(verts), m_triangles(tris) {
        m_surface_area = compute_tri_areas();
    }

    // Constructs a Mesh from a Shape object. Initializes total surface area.
    Mesh(Shape m_shape) : m_verts(m_shape.getVertices()), m_triangles(m_shape.getFaces()) {
        m_surface_area = compute_tri_areas();
    }

    // Computes the boundary mesh of a mesh. TODO: check if this is actually necessary? also could
    // just port FEM code I think
    Mesh computeBoundary() const;

    // Computes the convex hull of the mesh. Fast, but can be unstable.
    Mesh computeCH() const;
    // Computes the volumetric(?) convex hull of the mesh. More stable, but slower than CH.
    Mesh computeVCH() const;

    std::vector<Mesh> merge(const std::vector<Mesh>& Q);

    // Computes the volume of a mesh.
    double volume() const;

    // === Concavity Metric computations.

    // Samples a point set from the Mesh with the specified resolution (resolution here indicates
    // the number of samples per unit surface area; by default 2000)
    pair<vector<Vector3d>, vector<int>> sample_point_set(int resolution = 2000) const;

    float compute_tri_areas();  // TODO: should probably be private

    vector<Vector3f> boundary_sample(int samples_per_unit_area);
    Vector3f random_barycentric_coord(Vector3f& p1, Vector3f& p2, Vector3f& p3);

    //    std::pair<Mesh, Mesh> clip(const Plane& p);
    std::vector<Mesh> cut_plane(Plane& p);

 private:
    // for Monte-Carlo Tree Search
    array<double, 6> m_bbox;
    // for Eigenvalue computations
    array<double, 3> m_barycenter;
    // for optional PCA
    array<array<double, 3>, 3> m_rot;

    // surface area, calculated on construction.
    unordered_map<int, float> m_tri_areas;
    float m_surface_area;
};
