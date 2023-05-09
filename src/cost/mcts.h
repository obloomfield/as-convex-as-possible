#pragma once

#include "geom/mesh.h"

constexpr int MAX_NUM_PIECES = 9;
constexpr int MAX_NUM_EDGES = 5;
constexpr int NUM_CUTTING_PLANES = 5;

class MCTS {
 public:
    static map<double, Mesh> greedy_search(const Mesh& cur_mesh);

    static vector<Edge> get_concave_edges(const Mesh& mesh);

    static std::pair<Mesh, Mesh> get_best_cut(const vector<Edge>& concave_edges, Mesh& m);

 private:
};
