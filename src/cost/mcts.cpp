#include "mcts.h"

#include "cost/concavity.h"

// ============ MCTS =============

// TODO: change concavity to R_v

static std::default_random_engine rng_eng = std::default_random_engine {};

std::pair<Mesh*, Mesh*> MCTS::MCTS_search(const Mesh& cur_mesh)  {

    // Create root node v0 with input mesh
    double cost = ConcavityMetric::concavity(cur_mesh); // TODO: might need to switch to the R_v concavity
    ComponentsQueue root_C;
    root_C[cost] = new Mesh(cur_mesh); // TODO: NEED SHARED POINTERS HERE!!!!!!

    // initial candidates from mesh
    std::vector<Plane> candidate_planes = cur_mesh.get_axis_aligned_planes(NUM_CUTTING_PLANES);

    // create v_0 root tree node
    TreeNode* root = new TreeNode(root_C, 0, candidate_planes, rng_eng);

    // run search for ITERATIONS
    for (int t = 0; t < ITERATIONS; ++t) {

        // TreePolicy
        auto [v_l, q_l] = tree_policy(root, MAX_DEPTH);

        // DefaultPolicy
        double q_d = default_policy(v_l, MAX_DEPTH);

        // Quality
        double quality = (q_l + q_d) / static_cast<double>(MAX_DEPTH); // average between all the highest concavities down the path

        // Backup
        backup(v_l, quality);
    }

    // for all children of v_0, choose the TreeNode with the highest Q score
    // grab handle on the Mesh resulting from the plane cut (before its destroyed later)
    double max_q = -std::numeric_limits<double>::infinity();
    Mesh* cut_l = nullptr;
    Mesh* cut_r = nullptr;
    for (auto& tn : root->child_cuts) {
        // find better child
        if (tn->q > max_q) {
            // replace
            max_q = tn->q;
            cut_l = tn->cut_l;
            cut_r = tn->cut_r;
        }
    }

    // TODO: destroy the tree

    // assertion that meshes exist
    if (!cut_l || !cut_r)
        throw std::runtime_error("PANIC: no meshes after cut");

    // return the meshes after cut
    return {cut_l, cut_r};

}


std::pair<TreeNode*, double> MCTS::tree_policy(TreeNode* v, int max_depth) {
    // selected cutting planes
//    std::vector<Plane> S;

    TreeNode* curr_v = v;

    // negative concavity for tree policy (convexity)
    double total_concavity = 0.;

    // from the root to the leaf
    while (curr_v->depth < max_depth) {

        // if all cutting planes of c_star are expanded
        if (curr_v->has_expanded_all()) {
            // get next node based on best child of v
            TreeNode* curr_v = curr_v->get_best_child();

            // error case, if children is empty when all has been expanded for some reason
            if (curr_v == nullptr) {
                throw std::runtime_error("PANIC: unexpected children are empty");
            }

            // accumulate negative concavity for the next child
            total_concavity += -curr_v->C.rbegin()->first;

            // add corresponding plane of curr_v to selected cutting planes
//            S.push_back(curr_v->prev_cut_plane);
        } else {
            // randomly select a untried cutting plane P of c_star
            Plane untried_plane = curr_v->sample_next_candidate();

            // cut c_star into c_star_l and c_star_r (potentially more) with P
            std::vector<Mesh> components = curr_v->c_star->cut_plane(untried_plane);

            // ignore cuts that result in more than 2 pieces
            if (components.size() == 2) {

                // add the cutting plane P to selected cutting planes
//                S.push_back(untried_plane);

                // create new node v_prime to curr_v

                // first create new components cost queue
                ComponentsQueue C_prime(curr_v->C);
                // erase c_star from new queue
                C_prime.erase(C_prime.rbegin()->first);

                // compute concavity for new meshes and add to queue
                Mesh* m0 = new Mesh(components[0]);
                Mesh* m1 = new Mesh(components[1]);
                C_prime[ConcavityMetric::concavity(components[0])] = m0; // TODO: SHARED POINTER
                C_prime[ConcavityMetric::concavity(components[1])] = m1; // TODO: SHARED POINTER

                // create new tree node
                TreeNode* v_prime = new TreeNode(C_prime, curr_v->depth + 1, untried_plane, curr_v->candidate_planes, curr_v, rng_eng);
                v_prime->set_newly_cut_pieces(m0, m1);

                // add tree to children of curr_v
                curr_v->child_cuts.push_back(v_prime);

                // accumulate negative concavity for new child
                total_concavity += -C_prime.rbegin()->first;
            }

            // increment children count no matter what
            ++curr_v->children_count;

            // return selected planes and current tree node
            return {curr_v, total_concavity};
        }
    }

    return {curr_v, total_concavity};
}


double MCTS::default_policy(TreeNode* v, int max_depth) {
    // selected cutting planes
//    std::vector<Plane> S;

    // create a copy of v's components queue
    ComponentsQueue C_copy(v->C);

    // negative concavity (convexity)
    double total_concavity = 0.0;

    for (int i = 0; i < max_depth - v->depth; ++i) {
        auto it = C_copy.rbegin();
        const Mesh* c_star = it->second;

        // store the "best" results from cutting
        std::vector<Mesh> best_results;
        Plane best_direction;
        double q_max = -std::numeric_limits<double>::infinity();
        // concavity metrics for the two points
        double c_m0, c_m1;

        // for direction in {𝑥𝑦, 𝑥𝑧, 𝑦𝑧}
        std::vector<Plane> directions = c_star->get_axis_aligned_planes(NUM_CUTTING_PLANES);
        for (Plane& direction : directions) {
            std::vector<Mesh> cut = c_star->cut_plane(direction);

            // only care if cut resulted in 2 pieces
            if (cut.size() == 2) {
                // calculate concavity for the cut pieces
                double curr_c_m0 = ConcavityMetric::concavity(cut[0]);
                double curr_c_m1 = ConcavityMetric::concavity(cut[1]);
                double curr_q = -std::max(curr_c_m0, curr_c_m1);
                // if the negative concavity is greater than the max (most convex)
                if (curr_q > q_max) {
                    // save fields
                    best_results = cut;
                    best_direction = direction;
                    q_max = curr_q;
                    // save concavity scores
                    c_m0 = curr_c_m0;
                    c_m1 = curr_c_m1;
                }
            }
        }

        // at this point, have best cut pieces and plane
        C_copy.erase(it->first); // erase c_star
        // insert new pieces into queue
        // TODO: shared pointers here
        C_copy[c_m0] = new Mesh(best_results[0]);
        C_copy[c_m1] = new Mesh(best_results[1]);
        // add P to the selected set
//        S.push_back(best_direction);

        // accumulate total concavity
        total_concavity += q_max;
    }

    return total_concavity;
}


void MCTS::backup(TreeNode* v, double _q) {
    TreeNode* curr_v = v;

    while (curr_v != nullptr) {
        ++curr_v->visit_count; // increment visit count
        curr_v->q = std::max(curr_v->q, _q); // update value function score
        curr_v = curr_v->parent;
    }
}


// ============ Greedy =============

map<double, Mesh> MCTS::greedy_search(const Mesh& cur_mesh) {
    double cost = ConcavityMetric::concavity(cur_mesh);
    map<double, Mesh> cost_to_mesh;
    cost_to_mesh[cost] = cur_mesh;

    while (cost_to_mesh.size() > 0 && cost_to_mesh.size() < MAX_NUM_PIECES) {
        // Find the Mesh with the worst (i.e. highest) concavity score
        auto it = cost_to_mesh.rbegin();
        Mesh worst_mesh = it->second;

        // Get all concave edges of the worst shape, sorted from furthest to closest distance to CH
        vector<Edge> concave_edges = worst_mesh.get_concave_edges();
        deque<Edge> sorted_concave_edges =
            ConcavityMetric::sort_concave_edges(worst_mesh, concave_edges);

        // trim down number of concave edges
        vector<Edge> selected_concave_edges(MAX_NUM_EDGES);
        for (auto i = 0; i < MAX_NUM_EDGES && !sorted_concave_edges.empty(); i++) {
            selected_concave_edges[i] = sorted_concave_edges.front();
            sorted_concave_edges.pop_front();
        }

        // Get the best cut, remove the old Mesh, then insert the new fragments
        auto [frag1, frag2] = get_best_cut_greedy(selected_concave_edges, worst_mesh);
        cost_to_mesh.erase(it->first);

        double frag1_cost = ConcavityMetric::concavity(frag1);
        double frag2_cost = ConcavityMetric::concavity(frag2);

        cost_to_mesh[frag1_cost] = frag1;
        cost_to_mesh[frag2_cost] = frag2;
    }

    return cost_to_mesh;
}

vector<Edge> MCTS::get_concave_edges_greedy(const Mesh& mesh) {
    vector<Edge> concave_edges;
    for (const auto& [edge, tris] : mesh.m_edge_tris) {
        // Get the triangle points corresponding to each triangle's indices
        auto [tri1_indices, tri2_indices] = tris;
        Triangle tri1 = mesh.get_triangle(tri1_indices), tri2 = mesh.get_triangle(tri2_indices);

        // Get the vertices that the triangles don't share
        Vector3d v1 = get_third_point(tri1, edge), v2 = get_third_point(tri2, edge);

        // Check angle between the two triangles
        Vector3d u = v1 - edge.a_, v = v2 - edge.a_;
        double dotProduct = u.dot(v);

        // is angle less than 180 (is edge concave)? Add to list:
        if (dotProduct >= 0.0) {
            concave_edges.push_back(edge);
        }
    }
    return concave_edges;
}

std::pair<Mesh, Mesh> MCTS::get_best_cut_greedy(const vector<Edge>& concave_edges, Mesh& m) {
    Vector3d u(0.0, 0.0, 0.0);
    std::pair<Mesh, Mesh> best_pair;
    double min_cut_score = 1e17;

    for (const Edge& edge : concave_edges) {
        vector<Plane> cutting_planes = m.get_cutting_planes(edge, NUM_CUTTING_PLANES);
        for (Plane& plane : cutting_planes) {
            vector<Mesh> frags = m.cut_plane(plane);
            if (frags.size() != 2) {
                continue;
            }
            Mesh frag_one = frags[0];
            Mesh frag_two = frags[1];

            // Check if the new cut has a better min cut score than the current best
            double curr_cut_score = ConcavityMetric::concavity(frag_one) +
                                    ConcavityMetric::concavity(frag_two) -
                                    ConcavityMetric::concavity(m);
            if (curr_cut_score < min_cut_score) {
                min_cut_score = curr_cut_score;
                best_pair = {frag_one, frag_two};
            }
        }
    }
    return best_pair;
}
