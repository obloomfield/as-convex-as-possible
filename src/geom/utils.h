#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <array>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "geom/shapes.h"
#include "mcut/mcut.h"

using namespace std;
using namespace Eigen;

inline bool approx(double x, double y) {
    // https://stackoverflow.com/a/253874/15140014
    return std::abs(x - y) <= (std::max(std::abs(x), std::abs(y)) * DBL_EPSILON);
}

inline vector<Vector3d> vec3f_to_vec3d(const vector<Vector3f>& v) {
    auto nv = v.size();
    vector<Vector3d> res(nv);
    for (auto i = 0; i < nv; i++) res[i] = v[i].cast<double>();
    return res;
}

inline vector<Vector3f> float_to_vec3f(const vector<float>& float_vec) {
    vector<Vector3f> vec3d_vec;
    for (int i = 0; i < float_vec.size(); i += 3) {
        vec3d_vec.emplace_back(float_vec[i], float_vec[i + 1], float_vec[i + 2]);
    }
    return vec3d_vec;
}

inline vector<Vector3i> uint_to_vec3i(const vector<uint32_t>& uint_vec) {
    vector<Vector3i> vec3i_vec;
    for (int i = 0; i < uint_vec.size(); i += 3) {
        vec3i_vec.emplace_back(uint_vec[i], uint_vec[i + 1], uint_vec[i + 2]);
    }
    return vec3i_vec;
}

inline Vector3d get_third_point(const Triangle& t, const Edge& e) {
    if (!t[0].isApprox(e[0]) && !t[0].isApprox(e[1])) return t[0];
    if (!t[1].isApprox(e[0]) && !t[1].isApprox(e[1])) return t[1];
    return t[2];
}

inline bool same_dir(const Vector3d& a, const Vector3d& b) {
    return a.dot(b) > 0;
}

inline Vector3d project_onto_edge(const Edge& e, const Vector3d& pt) {
    // Compute direction vector of the line segment
    auto d_vec = (e[1] - e[0]).normalized();

    // Get vector from an endpoint to the point
    auto v = pt - e[0];

    // Compute dot product, then project pt onto the line
    auto t = v.dot(d_vec);
    auto proj = e[0] + t * d_vec;

    return proj;
}

inline double dist(const Vector3d& a, const Vector3d& b) {
    return sqrt((a - b).dot(a - b));
}

// Computes the distance between a point and a line segment (i.e. an edge).
inline double dist_pt2edge(const Vector3d& pt, const Edge& edge) {
    auto proj = project_onto_edge(edge, pt);
    return dist(pt, proj);
}

// Computes the distance between a point and a triangle.
inline double dist_pt2tri(const Vector3d& pt, const Triangle& tri) {
    // Get triangle normal; this defines a plane
    // 		n.dot(p - p0) = 0
    // 	where n = triangle normal and p0 = anchor point.
    auto r = tri[0], q = tri[1], p = tri[2];
    auto n = (r - p).cross(q - p);

    // Get vector from an anchor point (here, any point on the triangle) to pt
    auto v = pt - tri[0];
    // Then, compute v*n (distance along normal); if in opposite direction, flip n and dist
    auto d = v.dot(n);
    if (d < 0) {
        n *= -1;
    }

    // Subtract triangle normal, scaled by distance, from point
    auto proj = pt - d * n;

    // Check if projected point is within the triangle
    // See https://math.stackexchange.com/a/51328/1178805
    auto AB = tri[1] - tri[0], BC = tri[2] - tri[1], CA = tri[0] - tri[2];
    auto AP = proj - tri[0], BP = proj - tri[1], CP = proj - tri[2];

    // If each cross product is in the same direction as n, we're good
    if (same_dir(AB.cross(AP), n) && same_dir(BC.cross(BP), n) && same_dir(CA.cross(CP), n)) {
        return d;
    } else {
        // If not, compute minimum of distance between point and the 3 edges and 3 verts
        return min(min(min(dist_pt2edge(pt, {tri[0], tri[1]}),   // Compute minimum of distance
                           dist_pt2edge(pt, {tri[1], tri[2]})),  // between pt and the 3 edges
                       dist_pt2edge(pt, {tri[2], tri[0]})),
                   min(min(dist(pt, tri[0]),                     // Compute minimum of distance
                           dist(pt, tri[1])),                    // between pt and the 3 points
                       dist(pt, tri[2])));
    }
}

inline Vector3d edge_pt_norm(const Edge& e, const Vector3d& pt) {
    auto proj = project_onto_edge(e, pt);
    auto n = (pt - proj).normalized();
    return n;
}

inline Vector3d edge_tri_norm(const Edge& e, const Triangle& t) {
    auto non_edge_pt = get_third_point(t, e);
    return edge_pt_norm(e, non_edge_pt);
}

inline double signed_tri_volume(const Vector3d& p1, const Vector3d& p2, const Vector3d& p3) {
    // From here: https://stackoverflow.com/a/1568551
    return p1.dot(p2.cross(p3)) / 6.;
}

inline void print_triangle(const Vector3i& tri) {
    cout << "Triangle: " << (tri[0] + 1) << " " << (tri[1] + 1) << " " << (tri[2] + 1) << endl;
}

// Save verts/faces to an .obj file
inline void writeOBJ(const std::string& path, const float* ccVertices, const int ccVertexCount,
                     const uint32_t* ccFaceIndices, const uint32_t* faceSizes,
                     const uint32_t ccFaceCount) {
    printf("write file: %s\n", path.c_str());

    std::ofstream file(path);

    // write vertices and normals
    for (uint32_t i = 0; i < (uint32_t)ccVertexCount; ++i) {
        double x = ccVertices[(McSize)i * 3 + 0];
        double y = ccVertices[(McSize)i * 3 + 1];
        double z = ccVertices[(McSize)i * 3 + 2];
        file << "v " << x << " " << y << " " << z << std::endl;
    }

    int faceVertexOffsetBase = 0;

    // for each face in CC
    for (uint32_t f = 0; f < ccFaceCount; ++f) {
        int faceSize = faceSizes[f];
        file << "f ";
        // for each vertex in face
        for (int v = 0; (v < faceSize); v++) {
            const int ccVertexIdx = ccFaceIndices[(McSize)faceVertexOffsetBase + v];
            file << (ccVertexIdx + 1) << " ";
        }
        file << std::endl;

        faceVertexOffsetBase += faceSize;
    }
}

#endif  // UTILS_H
