#pragma once

#include <algorithm>
#include <set>
#include <unordered_set>

#include "geom/mesh.h"
#include "geom/shapes.h"

constexpr int MAX_NUM_PIECES = 80;
constexpr int MAX_NUM_EDGES = 5;
constexpr int NUM_CUTTING_PLANES = 5;

// d constant in DefaultPolicy and TreePolicy
constexpr int MAX_DEPTH = 4;
constexpr int ITERATIONS = 20;

constexpr bool SAVE_ITER = true;

static const std::string OUT_EDGE_FILE = "out/bunny_concave_edges.txt";

typedef map<double, const Mesh*> ComponentsQueue;

/*
 * Each child node corresponds to a cutting action from its parent
 */
struct TreeNode {
    ComponentsQueue
        C;  // state of the components and their concavity at this stage in the tree search
    Plane prev_cut_plane;               // current cut plane for this node
    std::vector<TreeNode*> child_cuts;  // all future/descendant states from this node. contains at
                                        // most 3m nodes, (where m is num cutting planes)
    TreeNode* parent = nullptr;         // parent pointer

    int depth;
    int visit_count = 0;
    int next_cand_choice = 0;
    int children_count = 0;

    double c;  // exploration parameter of c_star, is concavity(shape) / depth of tree
    double q;  // value function
    double concavity_max;

    const Mesh* c_star = nullptr;
    std::vector<Plane> candidate_planes;  // candidate planes for c_star

    Mesh* cut_l = nullptr;
    Mesh* cut_r = nullptr;

    // FUNCTIONS

    // constructor for root, no plane
    TreeNode(ComponentsQueue& new_C, int d, std::vector<Plane>& init_cand_planes,
             std::default_random_engine& rng) {
        depth = d;
        C = new_C;
        auto it = C.rbegin();
        q = it->first;        // initialize q to be concavity score, but will change later
        concavity_max = it->first; // keep concavity
        c = q / static_cast<double>(MAX_DEPTH);
        c_star = it->second;  // get argmax Concavity(c_i) in C

        // for root node, can just pass in
        candidate_planes = init_cand_planes;

        // shuffle it for randomness
        std::shuffle(std::begin(candidate_planes), std::end(candidate_planes), rng);
    }

    // for intermediate nodes, if passed in a plane and parent
    TreeNode(ComponentsQueue& new_C, int d, Plane& p, std::vector<Plane>& cand_planes,
             TreeNode* _parent, std::default_random_engine& rng) {
        depth = d;
        C = new_C;
        auto it = C.rbegin();
        q = it->first;        // initialize q to be concavity score, but will change later
        concavity_max = it->first; // keep concavity
        c = q / static_cast<double>(MAX_DEPTH);
        c_star = it->second;  // get argmax Concavity(c_i) in C
        parent = _parent; // set parent

        prev_cut_plane = p;
        // TODO: initialize candidate_planes. can pass in the same one each time and reshuffle
        candidate_planes = std::vector<Plane>(cand_planes);

        // shuffle it for randomness
        std::shuffle(std::begin(candidate_planes), std::end(candidate_planes), rng);
    }

    // sample "random" next candidate plane
    Plane sample_next_candidate() { return candidate_planes[next_cand_choice++]; }

    // Upper Confidence Bound score.
    double UCB_score() {
        // account for root node (this really shouldn't happen tho?)
        double p_vis = 1.;
        if (parent) {
            p_vis = static_cast<double>(parent->visit_count);
        }

        return q + (c * std::sqrt((2. * std::log(p_vis) / static_cast<double>(visit_count))));
    }

    // best child of the current tree node according to the UCB formula
    TreeNode* get_best_child() {
        // error case, if child cuts is empty
        if (child_cuts.size() == 0) {
            return nullptr;
        }

        // find best child based on highest UCB score
        double max_score = -std::numeric_limits<double>::infinity();
        TreeNode* best_child = nullptr;
        for (TreeNode* c : child_cuts) {
            double c_score = c->UCB_score();
            // replacement
            if (c_score > max_score) {
                max_score = c_score;
                best_child = c;
            }
        }
        return best_child;
    }

    // check if we have already expanded all candidates
    inline bool has_expanded_all() { return children_count == candidate_planes.size(); }

    // sets the new components resulting from the plane cut prev_cut_plane
    void set_newly_cut_pieces(Mesh* m0, Mesh* m1) {
        cut_l = m0;
        cut_r = m1;
    }

    // TODO: destroy this node's children recursively
    void free_children() {
        for (TreeNode* ch : child_cuts) {
            ch->free_children();
            delete ch;
        }
    }
};

class MCTS {
 public:
    static std::pair<Mesh*, Mesh*> MCTS_search(Mesh& cur_mesh);
    static map<double, Mesh> greedy_search(const Mesh& cur_mesh);

 private:
    // MCTS
    static std::pair<TreeNode*, double> tree_policy(TreeNode* v, int max_depth);
    static double default_policy(TreeNode* v, int max_depth);
    static void backup(TreeNode* v, double _q);

    // greedy
    static std::vector<Edge> get_concave_edges_greedy(const Mesh& mesh);
    static pair<map<double, Mesh>, vector<Plane>> get_best_cut_greedy(
        const vector<EdgeIndices>& concave_edges, Mesh& m);
};
