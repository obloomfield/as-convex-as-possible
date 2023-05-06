#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <string>

#include "mcut/mcut.h"

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
