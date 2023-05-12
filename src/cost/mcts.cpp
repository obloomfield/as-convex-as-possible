#include "mcts.h"

#include "cost/concavity.h"

// ============ MCTS =============

static std::default_random_engine rng_eng = std::default_random_engine {};

std::pair<Mesh*, Mesh*> MCTS::MCTS_search(Mesh& cur_mesh) {
    // Create root node v0 with input mesh
    double cost = ConcavityMetric::R_v(cur_mesh);
    ComponentsQueue root_C;
    root_C[cost] = new Mesh(cur_mesh);         // TODO: NEED SHARED POINTERS HERE!!!!!!

    // initial candidates from mesh
    std::vector<Plane> candidate_planes = cur_mesh.get_axis_aligned_planes(NUM_CUTTING_PLANES);

    // create v_0 root tree node
    TreeNode* root = new TreeNode(root_C, 0, candidate_planes, rng_eng);

    // run search for ITERATIONS
    for (int t = 0; t < ITERATIONS; ++t) {

        std::cout << "iteration: " << t << std::endl;

        // TreePolicy
        auto [v_l, q_l] = tree_policy(root, MAX_DEPTH);

        // DefaultPolicy
        double q_d = default_policy(v_l, MAX_DEPTH);

        // Quality
        double quality =
            (q_l + q_d) /
            static_cast<double>(
                MAX_DEPTH);  // average between all the highest concavities down the path

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
    if (!cut_l || !cut_r) return {nullptr, nullptr};

    // return the meshes after cut
    return {cut_l, cut_r};
}

std::pair<TreeNode*, double> MCTS::tree_policy(TreeNode* v, int max_depth) {
    // selected cutting planes
    //    std::vector<Plane> S;
    std::cout << "tree policy on " << v << std::endl;

    TreeNode* curr_v = v;

    // negative concavity for tree policy (convexity)
    double total_concavity = 0.;

    // from the root to the leaf
    while (curr_v->depth < max_depth) {
        // if all cutting planes of c_star are expanded
        if (curr_v->has_expanded_all()) {


            // get next node based on best child of v
            TreeNode* next_v = curr_v->get_best_child();

            // error case, if children is empty when all has been expanded for some reason
            if (next_v == nullptr) {
                // simply return the old node early (don't throw an error)
                return {curr_v, total_concavity};
            }

            // set next
            curr_v = next_v;

            // accumulate negative concavity for the next child
            total_concavity += -curr_v->concavity_max;

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

                C_prime[ConcavityMetric::R_v(components[0])] = m0; // TODO: SHARED POINTER
                C_prime[ConcavityMetric::R_v(components[1])] = m1; // TODO: SHARED POINTER

                // create new tree node
                TreeNode* v_prime = new TreeNode(C_prime, curr_v->depth + 1, untried_plane,
                                                 curr_v->candidate_planes, curr_v, rng_eng);
                v_prime->set_newly_cut_pieces(m0, m1);

                // add tree to children of curr_v
                curr_v->child_cuts.emplace_back(v_prime);

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
    std::cout << "default policy on " << v << std::endl;

    // create a copy of v's components queue
    ComponentsQueue C_copy(v->C);

    // negative concavity (convexity)
    double total_concavity = 0.0;

    for (int i = 0; i < max_depth - v->depth; ++i) {

        std::cout << "default search on depth " << v->depth + i << std::endl;

        auto it = C_copy.rbegin();
        const Mesh* c_star = it->second;

        // store the "best" results from cutting
        std::vector<Mesh> best_results;
        Plane best_direction;
        double q_max = -std::numeric_limits<double>::infinity();
        // concavity metrics for the two points
        double c_m0, c_m1;

//        std::cout << "depth: " << i << " , cutting in each direction" << std::endl;
        // for direction in {xy, xz, yz}
        std::vector<Plane> directions = c_star->get_axis_aligned_planes(NUM_CUTTING_PLANES);
        for (Plane& direction : directions) {
//            std::cout << "trying to cut in direction" << std::endl;
            std::vector<Mesh> cut = c_star->cut_plane(direction);

            // only care if cut resulted in 2 pieces
            if (cut.size() == 2) {
                // calculate concavity for the cut pieces
                double curr_c_m0 = ConcavityMetric::R_v(cut[0]);
                double curr_c_m1 = ConcavityMetric::R_v(cut[1]);
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

//        std::cout << "default after direction search" << std::endl;
        if (best_results.size() == 0) {
            break;
        }
        // at this point, have best cut pieces and plane
        C_copy.erase(it->first);  // erase c_star

//        std::cout << "after erase" << std::endl;

        // insert new pieces into queue
        // TODO: shared pointers here
        C_copy[c_m0] = new Mesh(best_results[0]);
        C_copy[c_m1] = new Mesh(best_results[1]);

//        std::cout << "after new meshes" << std::endl;

        // add P to the selected set
        //                                S.push_back(best_direction);

        // accumulate total concavity
        total_concavity += q_max;
    }

    return total_concavity;
}

void MCTS::backup(TreeNode* v, double _q) {
    std::cout << "backing up" << std::endl;
    TreeNode* curr_v = v;

    while (curr_v != nullptr) {
        ++curr_v->visit_count;                // increment visit count
        curr_v->q = std::max(curr_v->q, _q);  // update value function score
        curr_v = curr_v->parent;
    }
}

// ============ Greedy =============

map<double, Mesh> MCTS::greedy_search(const Mesh& cur_mesh) {
    double cost = ConcavityMetric::R_v(cur_mesh);
    map<double, Mesh> cost_to_mesh;
    cost_to_mesh[cost] = cur_mesh;

    int i = 0;
    while (cost_to_mesh.size() > 0 && cost_to_mesh.size() < MAX_NUM_PIECES) {
        DEBUG_MSG("On iteration " << i++);
        // Find the Mesh with the worst (i.e. highest) concavity score
        auto it = std::prev(cost_to_mesh.end());
        Mesh worst_mesh = it->second;

        // If basically convex, done
        if (worst_mesh.is_convex()) {
            DEBUG_MSG("Worst mesh is convex!");
            break;
        }

        // Get all concave edges of the worst shape, sorted from furthest to closest distance to CH
        vector<EdgeIndices> concave_edge_indices = worst_mesh.get_concave_edges();
        append_to_file(OUT_EDGE_FILE, concave_edge_indices.size());
        cout << "num concave edges: " << concave_edge_indices.size() << endl;

        // if no concave edges, we're done
        //        if (concave_edge_indices.empty()) {
        //            cout << "worst mesh is convex!\n";
        //            break;
        //        }

        // Sort by furthest distance from CH
        deque<EdgeIndices> sorted_concave_edge_indices =
            ConcavityMetric::sort_concave_edge_indices(worst_mesh, concave_edge_indices);

        // trim down number of concave edges
        vector<EdgeIndices> selected_concave_edge_indices;
        selected_concave_edge_indices.reserve(MAX_NUM_EDGES);
        for (auto i = 0; i < MAX_NUM_EDGES && !sorted_concave_edge_indices.empty(); i++) {
            selected_concave_edge_indices.push_back(sorted_concave_edge_indices.front());
            sorted_concave_edge_indices.pop_front();
        }

        // Get the best cut, remove the old Mesh, then insert the new fragments
        auto [frag_costs, cuts] = get_best_cut_greedy(selected_concave_edge_indices, worst_mesh);
        if (frag_costs.empty()) {
            DEBUG_MSG("Got no fragments; stopping search...");
            break;
        }

        cost_to_mesh.erase(it);
        DEBUG_MSG("Adding " << frag_costs.size() << " fragments...");

        for (auto& [cost, m] : frag_costs) {
            if (cost_to_mesh.contains(cost)) DEBUG_MSG(cost << " already exists in map");
            cost_to_mesh[cost] = m;
        }

        if (SAVE_ITER) {
            int cnt = 0;
            for (auto&& p : cuts) {
                p.save_to_file("out/iter" + to_string(i) + "plane" + to_string(cnt++) + ".obj");
            }
            cnt = 0;
            for (auto& [_, m] : cost_to_mesh) {
                m.save_to_file("out/iter" + to_string(i) + "mesh" + to_string(cnt++) + ".obj");
            }
        }

        DEBUG_MSG("currently have " << cost_to_mesh.size() << " fragments");
    }

    return cost_to_mesh;
}

vector<Edge> MCTS::get_concave_edges_greedy(const Mesh& mesh) {
    vector<Edge> concave_edges;
    for (const auto& [ei, tris] : mesh.m_edge_tris) {
        // Get the triangle points corresponding to each triangle's indices
        auto [tri1_indices, tri2_indices] = tris;
        Triangle tri1 = mesh.get_triangle(tri1_indices), tri2 = mesh.get_triangle(tri2_indices);

        auto edge = mesh.get_edge(ei);
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

pair<map<double, Mesh>, vector<Plane>> MCTS::get_best_cut_greedy(
    const vector<EdgeIndices>& concave_edge_indices, Mesh& m) {
    Vector3d u(0.0, 0.0, 0.0);
    EdgeIndices best_ei;
    map<double, Mesh> best_frags;
    vector<Plane> best_cuts;
    double min_cut_score = 1e17;

    for (const EdgeIndices& ei : concave_edge_indices) {
        vector<Plane> cutting_planes = m.get_cutting_planes(ei, NUM_CUTTING_PLANES);
        for (Plane& plane : cutting_planes) {
            map<double, Mesh> curr_costs;

            DEBUG_MSG("Cutting plane along edge " << ei.ai_ << ", " << ei.bi_ << "...");
            vector<Mesh> frags = m.cut_plane(plane);
            DEBUG_MSG("Done cutting");

            if (frags.empty()) {
                // If failed to cut any components, skip
                DEBUG_MSG("Failed to cut plane along edge " << ei.ai_ << ", " << ei.bi_
                                                            << ". Saving and continuing...");
                if (SAVE_ITER) {
                    for (int i = 0; i < cutting_planes.size(); i++) {
                        cutting_planes[i].save_to_file("out/plane_failed" + to_string(i) + ".obj");
                    }
                    m.save_to_file("out/mesh_failed.obj");
                }
                continue;
                exit(EXIT_FAILURE);
            }

            // If any of the mesh's volumes are sufficiently small, discard cut


            //            DEBUG_MSG("Computing concavity...");
            //            auto t1 = chrono::high_resolution_clock::now();

            double cut_score = -1 * ConcavityMetric::concavity(m);

            for (const auto& frag : frags) {
                double cost = ConcavityMetric::concavity(frag);
                cut_score += cost;
                // Don't continue if worse
                if (cut_score > min_cut_score) break;
                curr_costs[cost] = frag;
            }

            if (cut_score < min_cut_score) {
                best_ei = ei;
                min_cut_score = cut_score;
                best_cuts = cutting_planes;
                best_frags = curr_costs;
            }

            //            auto t2 = chrono::high_resolution_clock::now();
            //            DEBUG_MSG("Total time: " << chrono::duration_cast<chrono::milliseconds>(t2
            //            - t1).count()
            //                                     << "ms");
        }
    }

    //    DEBUG_MSG("Best cut: " << best_ei.ai_ << ", " << best_ei.bi_);
    return {best_frags, best_cuts};
}
