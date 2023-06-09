#pragma once

#include <Eigen/SparseCholesky>
#include <cmath>
#include <unordered_map>

#include "Eigen/StdList"
#include "Eigen/StdVector"
#include "geom/mesh.h"
#include "graphics/shape.h"
#include "cost/mcts.h"

class Shader;

constexpr float COST_THRESHOLD = 100.;  // TODO: assign a better value

class ACAP {
 public:
    void init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax);

    void draw(Shader *shader, GLenum mode) { m_shape.draw(shader, mode); }

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold) {
        return m_shape.getClosestVertex(start, ray, threshold);
    }

    SelectMode select(Shader *shader, int vertex) { return m_shape.select(shader, vertex); }

    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode) {
        return m_shape.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool getAnchorPos(int lastSelected, Eigen::Vector3f &pos, Eigen::Vector3f ray,
                      Eigen::Vector3f start) {
        return m_shape.getAnchorPos(lastSelected, pos, ray, start);
    }

    void ACD(const std::string& mesh_path, const std::string& out_path);

 private:
    Shape m_shape;

    std::vector<Mesh> merge(Mesh &mesh, std::vector<Mesh> &convex_meshes);
};
