#pragma once

#include <array>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "btConvexHull/btConvexHullComputer.h"
#include "graphics/shape.h"
#include "quickhull/QuickHull.hpp"
#include "shapes.h"
#include "utils/rng.h"

class Plane;
class Triangle;

class Mesh {
 public:
    // Vertices/triangles information.
    std::vector<Eigen::Vector3f> m_verts;
    std::vector<Eigen::Vector3i> m_triangles;

    // Default constructor.
    Mesh() = default;

    // Constructs a mesh from a list of vertices and triangles. Initializes total
    // surface area.
    Mesh(std::vector<Eigen::Vector3f> verts, std::vector<Eigen::Vector3i> tris)
        : m_verts(verts), m_triangles(tris) {
        m_surface_area = compute_tri_areas();
        m_bbox = compute_bounding_box();
    }

    // Constructs a Mesh from a Shape object. Initializes total surface area.
    Mesh(Shape m_shape) : m_verts(m_shape.getVertices()), m_triangles(m_shape.getFaces()) {
        m_surface_area = compute_tri_areas();
        m_bbox = compute_bounding_box();
    }

    // Get a triangle from a provided Vector3i.
    Triangle get_triangle(const Eigen::Vector3i &tri) const {
        return {m_verts[tri[0]], m_verts[tri[1]], m_verts[tri[2]]};
    }

    // Computes the convex hull of the mesh. Fast, but can be unstable.
    Mesh computeCH() const;
    // Computes the volumetric(?) convex hull of the mesh. More stable, but slower
    // than CH.
    Mesh computeVCH() const;

    std::vector<Mesh> merge(const std::vector<Mesh> &Q);

    // Computes the volume of a mesh.
    double volume() const;

    // === Concavity Metric computations.

    // Samples a point set from the Mesh with the specified resolution (resolution
    // here indicates the number of samples per unit surface area; by default
    // 2000)
    pair<vector<Eigen::Vector3d>, std::vector<int>> sample_point_set(int resolution = 2000) const;

    std::vector<Mesh> cut_plane(Plane &p);
    std::vector<Mesh> cut_plane(quickhull::Plane<double> &p);

    std::array<double, 6> bounding_box() const { return m_bbox; };

    // Concavity Metric private members
 private:
    // for Monte-Carlo Tree Search
    std::array<double, 6> m_bbox;
    std::array<double, 6> compute_bounding_box();

    // for Eigenvalue computations
    std::array<double, 3> m_barycenter;
    // for optional PCA
    std::array<std::array<double, 3>, 3> m_rot;

    // surface area, calculated on construction.
    std::vector<float> m_tri_areas;
    std::vector<float> m_cdf;
    float m_surface_area;

    // Concavity Metric private members

    static Eigen::Vector3d random_barycentric_coord(const Eigen::Vector3f &p1,
                                                    const Eigen::Vector3f &p2,
                                                    const Eigen::Vector3f &p3);
    float compute_tri_areas();  // should probably be private
};
