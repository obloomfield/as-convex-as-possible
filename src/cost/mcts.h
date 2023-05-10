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
constexpr int MAX_DEPTH = 4;
constexpr int ITERATIONS = 500;

// TODO: how to initialize axis-aligned cutting planes

typedef map<double, const Mesh*> ComponentsQueue;

/*
 * Each child node corresponds to a cutting action from its parent
 */
struct TreeNode {
    ComponentsQueue C; // state of the components and their concavity at this stage in the tree search
    Plane prev_cut_plane; // current cut plane for this node
    std::vector<TreeNode*> child_cuts; // all future/descendant states from this node. contains at most 3m nodes, (where m is num cutting planes)
    TreeNode* parent = nullptr; // parent pointer

    int depth;
    int visit_count = 0;
    int next_cand_choice = 0;
    int children_count = 0;

    double UCB_score = 0.; // Upper Confidence Bound score ? this might need to be a function that we recalculate

    double q;

    const Mesh* c_star = nullptr;
    std::vector<Plane> candidate_planes; // candidate planes for c_star


    // FUNCTIONS

    // constructor for root, no plane
    TreeNode(ComponentsQueue& new_C, int d, std::vector<Plane>& init_cand_planes, std::default_random_engine& rng) {
        depth = d;
        C = new_C;
        auto it = C.rbegin();
        q = it->first;
        c_star = it->second; // get argmax Concavity(c_i) in C

        // for root node, can just pass in
        candidate_planes = init_cand_planes;

        // shuffle it for randomness
        std::shuffle(std::begin(candidate_planes), std::end(candidate_planes), rng);
    }

    // for intermediate nodes, if passed in a plane and parent
    TreeNode(ComponentsQueue& new_C, int d, Plane& p, std::vector<Plane>& cand_planes, TreeNode* parent, std::default_random_engine& rng) {
        depth = d;
        C = new_C;
        auto it = C.rbegin();
        q = it->first;
        c_star = it->second; // get argmax Concavity(c_i) in C

        prev_cut_plane = p;
        // TODO: initialize candidate_planes. can pass in the same one each time and reshuffle
        candidate_planes = std::vector<Plane>(cand_planes);

        // shuffle it for randomness
        std::shuffle(std::begin(candidate_planes), std::end(candidate_planes), rng);
    }

    // sample "random" next candidate plane
    Plane sample_next_candidate() {
        return candidate_planes[next_cand_choice++];
    }

    // best child of the current tree node according to the UCB formula
    TreeNode* get_best_child() {
        // error case, if child cuts is empty
        if (child_cuts.size() == 0) {
            return nullptr;
        }
        return child_cuts[0]; // TODO: implement this
    }

    // check if we have already expanded all candidates
    inline bool has_expanded_all() {
        return children_count == candidate_planes.size();
    }

    // TODO: destroy this node's children, and then destroy itself.
    void free_node() {


    }

};

class MCTS {
 public:
    static ComponentsQueue MCTS_search(const Mesh& cur_mesh);
    static map<double, Mesh> greedy_search(const Mesh& cur_mesh);

 private:
    // MCTS
    static std::pair<std::vector<Plane>, TreeNode*> tree_policy(TreeNode* v, int max_depth);
    static std::pair<std::vector<Plane>, double> default_policy(TreeNode* v, int max_depth);
    static void backup(TreeNode* v, double _q);

    // greedy
    static vector<Edge> get_concave_edges_greedy(const Mesh& mesh);
    static std::pair<Mesh, Mesh> get_best_cut_greedy(const vector<Edge>& concave_edges, Mesh& m);

};
