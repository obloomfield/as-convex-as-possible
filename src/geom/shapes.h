#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

#include "quickhull/Structs/Plane.hpp"
//#include "mesh.h"

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

    Plane(const quickhull::Plane<double> &p, const Mesh &m);

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

class Edge {
 public:
    Eigen::Vector3d a_, b_;

    Edge() = default;
    Edge(const Eigen::Vector3d &a, const Eigen::Vector3d &b) : a_(a), b_(b) {}
    Edge(const std::array<Eigen::Vector3d, 2> &e) : a_(e[0]), b_(e[1]) {}

    inline Eigen::Vector3d operator[](int i) const {
        assert(i == 0 || i == 1);
        return i ? b_ : a_;
    }

    inline Eigen::Vector3d midpoint() const { return (a_ + b_) / 2.; }

    // Get distances to points/edges/triangles
    double dist_to(const Eigen::Vector3d &pt) const { return dist(this->midpoint(), pt); }
    double dist_to(const Edge &e) const { return dist_pt2edge(this->midpoint(), e); }
    double dist_to(const Triangle &tri) const { return dist_pt2tri(this->midpoint(), tri); }

    // Equality operators
    bool operator==(const Edge& other) const {
        return (a_ == other.a_ && b_ == other.b_) || (a_ == other.b_ && b_ == other.a_);
    }
    bool operator!=(const Edge& other) const {
        return !(*this == other);
    }
};
