#pragma once

#include "Eigen/Dense"
#include "convex-hull/convex-hull.h"
#include <vector>

class Mesh {
private:

public:
    // TODO: populate instance variables

    std::vector<Eigen::Vector3f> m_verts;
    std::vector<Eigen::Vector3i> m_triangles;

    Mesh();

    void init(std::vector<Eigen::Vector3f> vertices,  std::vector<Eigen::Vector3i> triangles);

    Mesh VCH() const;
};
