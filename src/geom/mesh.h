#pragma once

#include <array>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "plane.h"
#include "Eigen/Dense"
#include "btConvexHull/btConvexHullComputer.h"
#include "graphics/shape.h"
#include "quickhull/QuickHull.hpp"

class Plane;

using namespace std;
using namespace Eigen;

class Mesh {
 public:
    // Default constructor.
    Mesh() = default;

    // Constructs a mesh from a list of vertices and triangles. Initializes total surface area.
    Mesh(vector<Vector3f> verts, vector<Vector3i> tris) : m_verts(verts), m_triangles(tris) {
        m_surface_area = compute_tri_areas();
        m_bbox = compute_bounding_box();
    }

    // Constructs a Mesh from a Shape object. Initializes total surface area.
    Mesh(Shape m_shape) : m_verts(m_shape.getVertices()), m_triangles(m_shape.getFaces()) {
        m_surface_area = compute_tri_areas();
        m_bbox = compute_bounding_box();
    }

    // Computes the volume of a mesh.
    double volume() const;
    // Computes the convex hull of the mesh. Fast, but can be unstable.
    Mesh computeCH() const;
    // Computes the volumetric(?) convex hull of the mesh. More stable, but slower than CH.
    Mesh computeVCH() const;

    std::vector<Mesh> merge(const std::vector<Mesh>& Q);

    // === Concavity Metric computations.
    float compute_tri_areas();
    vector<Vector3f> boundary_sample(int samples_per_unit_area);
    Vector3f random_barycentric_coord(Vector3f& p1, Vector3f& p2, Vector3f& p3);

    float calc_volume();

    vector<Mesh> cut_plane(Plane& p);
    vector<Mesh> cut_plane(quickhull::Plane<double>& p);

    array<double, 6> bounding_box() const { return m_bbox; };
    vector<Vector3f> vertices() const { return m_verts; };


 private:
    // Vertices/triangles information.
    vector<Vector3f> m_verts;
    vector<Vector3i> m_triangles;

    // for Monte-Carlo Tree Search
    array<double, 6> m_bbox;
    array<double, 6> compute_bounding_box();

    // for Eigenvalue computations
    array<double, 3> m_barycenter;
    // for optional PCA
    array<array<double, 3>, 3> m_rot;

    // surface area, calculated on construction.
    unordered_map<int, float> m_tri_areas;
    float m_surface_area;
};
