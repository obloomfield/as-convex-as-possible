#pragma once

#include <vector>

#include "Eigen/Dense"

struct Edge {};

class ConvexHull {
 public:
    void calculate(std::vector<Eigen::Vector3f> verts);

    std::vector<Eigen::Vector3f>& getVertices() { return vertices; }
    std::vector<Edge>& getEdges() { return edges; }
    std::vector<Eigen::Vector3i>& getFaces() { return faces; }

 private:
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Edge> edges;
    std::vector<Eigen::Vector3i> faces;
};
