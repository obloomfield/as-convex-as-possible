#pragma once

#include <algorithm>
#include <set>
#include <unordered_set>

#include "geom/mesh.h"
#include "geom/shapes.h"

constexpr int MAX_NUM_PIECES = 9;
constexpr int MAX_NUM_EDGES = 5;
constexpr int NUM_CUTTING_PLANES = 5;

// d constant in DefaultPolicy and TreePolicy
constexpr int MAX_DEPTH = 10;
constexpr int ITERATIONS = 100;

// TODO: how to initialize axis-aligned cutting planes


typedef map<double, Mesh*> ComponentsQueue;

/*
 * Each child node corresponds to a cutting action from its parent
 */
struct TreeNode {
    ComponentsQueue C; // state of the components and their concavity at this stage in the tree search
    Plane* prev_cut_plane = nullptr; // current cut plane for this node
    std::unordered_set<TreeNode*> child_cuts; // all future/descendant states from this node. contains at most 3m nodes, (where m is num cutting planes)

    double UCB_score = 0.; // Upper Confidence Bound score
    int visit_count = 0;
    int depth;
};

class MCTS {
 public:
    static ComponentsQueue MCTS_search(const Mesh& cur_mesh);
    static map<double, Mesh> greedy_search(const Mesh& cur_mesh);

 private:

    // MCTS
    static std::pair<std::unordered_set<Plane>, TreeNode*> tree_policy(TreeNode* v, int depth);

    // greedy
    static vector<Edge> get_concave_edges_greedy(const Mesh& mesh);
    static std::pair<Mesh, Mesh> get_best_cut_greedy(const vector<Edge>& concave_edges, Mesh& m);

};
