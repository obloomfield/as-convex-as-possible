#pragma once

#include <array>
#include <map>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "QtCore/qfileinfo.h"
#include "btConvexHull/btConvexHullComputer.h"
#include "geom/utils.h"
#include "graphics/meshloader.h"
#include "graphics/shape.h"
#include "mcut/mcut.h"
#include "quickhull/QuickHull.hpp"
#include "shapes.h"
#include "utils/rng.h"

class Plane;
class Triangle;

constexpr double VOL_EPSILON = 5 * 1e-3;
constexpr double AREA_EPSILON = 6 * 1e-4;
constexpr double DOT_THRESHOLD = .996;

class Mesh {
 public:
    // Vertices/triangles information.
    std::vector<Eigen::Vector3d> m_verts;
    std::vector<Eigen::Vector3i> m_triangles;
    // Map of EdgeIndices -> Triangles
    std::map<EdgeIndices, std::array<Eigen::Vector3i, 2>> m_edge_tris;

    // Default constructor.
    Mesh() = default;

    // Constructs a mesh from a list of vertices and triangles. Initializes total
    // surface area and edge map.
    Mesh(std::vector<Eigen::Vector3d> verts, std::vector<Eigen::Vector3i> tris, int scale = 1);

    // Constructs a Mesh from a Shape object. Initializes total surface area.
    Mesh(Shape m_shape) : Mesh(m_shape.getVerticesDouble(), m_shape.getFaces()) {}

    // Scale by a factor
    Mesh scale(int k) {
        auto n = m_verts.size();
        for (int i = 0; i < n; i++) {
            cout << i << endl;
            m_verts[i] *= k;
        }
        return Mesh(m_verts, m_triangles);
    }

    // Get a triangle/edge from a provided Vector3i.
    Triangle get_triangle(const Eigen::Vector3i &tri) const {
        return {m_verts[tri[0]], m_verts[tri[1]], m_verts[tri[2]]};
    }
    Edge get_edge(const EdgeIndices &ei) const { return Edge(m_verts[ei.ai_], m_verts[ei.bi_]); }

    // Get a triangle's edges from a provided Vector3i.
    std::array<Edge, 3> get_triangle_edges(const Eigen::Vector3i &tri) const;
    std::array<EdgeIndices, 3> get_triangle_edge_indices(const Eigen::Vector3i &tri) const;

    // Computes the convex hull of the mesh. Fast, but can be unstable.
    Mesh computeCH() const;
    // Computes the volumetric(?) convex hull of the mesh. More stable, but slower
    // than CH.
    Mesh computeVCH() const;

    // Computes the volume of a mesh.
    double volume() const;

    // === Concavity Metric computations.

    // Samples a point set from the Mesh with the specified resolution (resolution
    // here indicates the number of samples per unit surface area; by default
    // 2000)
    pair<vector<Eigen::Vector3d>, std::vector<int>> sample_point_set(int resolution = 2000) const;

    // Get k cutting planes for the mesh along a concave edge, from one triangle to another.
    std::vector<Plane> get_cutting_planes(const EdgeIndices &ei, int k) const;
    vector<Plane> get_axis_aligned_planes(int k) const;  // axis aligned version for MCTS

    // Cut the mesh with the specified cutting plane.
    std::vector<Mesh> cut_plane(Plane &p) const;
    std::vector<Mesh> cut_plane(quickhull::Plane<double> &p) const;

    std::array<double, 6> bounding_box() const { return m_bbox; };

    // MCTS helpers
    bool is_convex() const;
    bool is_concave() const;
    vector<EdgeIndices> get_concave_edges() const;
    std::vector<Edge> shared_edges(const Eigen::Vector3i &tri1, const Eigen::Vector3i &tri2);
    std::vector<Mesh> merge(const std::vector<Mesh> &Q);

    // For helpful intermediate step visualizations
    static Mesh load_from_file(const std::string &path, int scale = 1);
    void save_to_file(const std::string &path) const;

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
    static Eigen::Vector3d random_barycentric_coord(const Eigen::Vector3d &p1,
                                                    const Eigen::Vector3d &p2,
                                                    const Eigen::Vector3d &p3);

    double angle_between_tris(const Eigen::Vector3i &t1, const Eigen::Vector3i &t2);

    float compute_tri_areas();  // should probably be private
};
