#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <array>
#include <fstream>
#include <string>

#include "mcut/mcut.h"

using namespace std;
using namespace Eigen;

inline float dist(const Vector3d& a, const Vector3d& b) {
    // TODO: is this not just the distance lol
    return sqrt(a.dot(b));
}

// Computes the distance between a point and a line segment (i.e. an edge).
double dist_pt2seg(const Vector3d& p, const array<Vector3d, 2>& seg) {
    // TODO: figure out how to do this

    return 0.0;
}

// Computes the distance between a point and a triangle.
double dist_pt2tri(const Vector3d& p, const array<Vector3d, 3>& tri) {
    // First, compute the triangle's corresponding plane
    // TODO: implement https://math.stackexchange.com/q/1034568/1178805

    // Then, project point onto plane
    // TODO: implement https://stackoverflow.com/q/9605556/15140014

    // See if projected point is on triangle
    // TODO: implement https://math.stackexchange.com/a/51328/1178805

    // If so, return distance between projected and real point

    // If not, compute minimum of distance between point and the 3 edges and 3 verts

    return 0.;
}

// Save verts/faces to an .obj file
void writeOBJ(const std::string& path, const float* ccVertices, const int ccVertexCount,
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
