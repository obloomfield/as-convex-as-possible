#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <vector>

#include "quickhull/Structs/Plane.hpp"
// #include "mesh.h"

class Mesh;
class Triangle;
class Edge;

// Forward declare; will be defined in utils.h
double dist(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
double dist_pt2edge(const Eigen::Vector3d &pt, const Edge &e);
double dist_pt2tri(const Eigen::Vector3d &pt, const Triangle &tri);

class Plane {
 private:
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d p3;

 public:
    Plane(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d) {
        p0 = a, p1 = b, p2 = c, p3 = d;
        // ensure that all points make a plane
        double EPSILON = 1e-9;
        Eigen::Matrix<double, 3, 3> mat;
        mat << (p1 - p0), (p2 - p0), (p3 - p0);
        assert(mat.determinant() < EPSILON);
    }

    Plane(Edge e, const Eigen::Vector3d &norm, std::array<double, 6> bbox);
    Plane(const quickhull::Plane<double> &p, std::array<double, 6> bbox);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> bounds() {
        return std::make_tuple(p0, p1, p2, p3);
    }
};

class Triangle {
 public:
    std::array<Eigen::Vector3d, 3> pts_;

    Triangle() = default;
    Triangle(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c)
        : pts_({a, b, c}) {}
    Triangle(const std::array<Eigen::Vector3d, 3> &tri) : pts_(tri) {}

    inline Eigen::Vector3d operator[](int i) const {
        assert(i >= 0 && i < 3);
        return pts_[i];
    }
};

inline bool operator<(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    for (int i = 0; i < 3; i++) {
        if (a[i] < b[i]) return true;
        if (a[i] > b[i]) return false;
    }
    return false;
}

class Edge {
 public:
    Eigen::Vector3d a_, b_;
    //    int ai_ = 0, bi_ = 0;  // If we need them, get the corresponding indices in the Mesh

    Edge() = default;

    // Maintain ordering such that a <= b always.
    Edge(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        if (a < b) {
            a_ = a, b_ = b;
        } else {
            a_ = b, b_ = a;
        }
    }
    Edge(const std::array<Eigen::Vector3d, 2> &e) {
        if (e[0] < e[1]) {
            a_ = e[0], b_ = e[1];
        } else {
            a_ = e[1], b_ = e[0];
        }
    }

    // In this case, we must maintain that each edge is ordered such that ai_ < bi_, for proper
    // ordering.
    //    Edge(const Eigen::Vector3d &a, const Eigen::Vector3d &b, int ai, int bi) {
    //        if (ai < bi) {
    //            a_ = a, b_ = b, ai_ = ai, bi_ = bi;
    //        } else {
    //            a_ = b, b_ = a, ai_ = bi, bi_ = ai;
    //        }
    //    }
    //    Edge(const std::array<Eigen::Vector3d, 2> &e, const std::array<int, 2> &ei) {
    //        if (ei[0] < ei[1]) {
    //            a_ = e[0], b_ = e[1], ai_ = ei[0], bi_ = ei[1];
    //        } else {
    //            a_ = e[1], b_ = e[0], ai_ = ei[1], bi_ = ei[0];
    //        }
    //    }

    inline Eigen::Vector3d midpoint() const { return (a_ + b_) / 2.; }

    // Get distances to points/edges/triangles
    double dist_to(const Eigen::Vector3d &pt) const { return std::min(dist(a_, pt), dist(b_, pt)); }
    double dist_to(const Edge &e) const {
        return std::min(dist_pt2edge(a_, e), dist_pt2edge(b_, e));
    }
    double dist_to(const Triangle &tri) const {
        return std::min(dist_pt2tri(a_, tri), dist_pt2tri(b_, tri));
    }

    // Array accessing. Here, we only need the actual values
    inline Eigen::Vector3d operator[](int i) const {
        assert(i == 0 || i == 1);
        return i ? b_ : a_;
    }

    // For map stuffs
    inline bool operator<(const Edge &other) const {
        if (a_ < other.a_) return true;
        if (other.a_ < a_) return false;
        return b_ < other.b_;
    }

    // Equality operators
    inline bool operator==(const Edge &other) const {
        return a_.isApprox(other.a_) && b_.isApprox(other.b_);
    }

    inline bool operator!=(const Edge &other) const { return !(*this == other); }
};
