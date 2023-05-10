#include "mcts.h"

#include "cost/concavity.h"

// ============ MCTS =============

ComponentsQueue MCTS::MCTS_search(const Mesh& cur_mesh) {
    // Create root node v0 with input mesh

    TreeNode root = {.depth = 0};

    for (int iter = 0; iter < ITERATIONS; ++iter) {
        // TreePolicy
        auto [root_planes, intermediate_planes] = tree_policy(&root, MAX_DEPTH);
    }

    return {};
}

std::pair<std::unordered_set<Plane>, TreeNode*> MCTS::tree_policy(TreeNode* v, int depth) {
    //    std::unordered_set<Plane> S;
    return {};
}

// ============ Greedy =============

map<double, Mesh> MCTS::greedy_search(const Mesh& cur_mesh) {
    double cost = ConcavityMetric::concavity(cur_mesh);
    map<double, Mesh> cost_to_mesh;
    cost_to_mesh[cost] = cur_mesh;

    int i = 0;
    while (cost_to_mesh.size() > 0 && cost_to_mesh.size() < MAX_NUM_PIECES) {
        DEBUG_MSG("On iteration " << i++);
        // Find the Mesh with the worst (i.e. highest) concavity score
        auto it = std::prev(cost_to_mesh.end());
        Mesh worst_mesh = it->second;

        // Get all concave edges of the worst shape, sorted from furthest to closest distance to CH
        vector<EdgeIndices> concave_edge_indices = worst_mesh.get_concave_edges();
        cout << "num concave edges: " << concave_edge_indices.size() << endl;

        // if no concave edges, we're done
        if (concave_edge_indices.empty()) {
            cout << "worst mesh is convex!\n";
            break;
        }

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
        cost_to_mesh.erase(it);

        int cnt = 0;
        for (auto&& p : cuts) {
            p.save_to_file("out/iter" + to_string(i) + "plane" + to_string(cnt++) + ".obj");
        }
        for (auto& [cost, m] : frag_costs) {
            if (cost_to_mesh.contains(cost)) DEBUG_MSG(cost << " already exists in map");
            cost_to_mesh[cost] = m;
        }
        cnt = 0;
        for (auto& [_, m] : cost_to_mesh) {
            m.save_to_file("out/iter" + to_string(i) + "mesh" + to_string(cnt++) + ".obj");
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
    map<double, Mesh> best_frags;
    vector<Plane> best_cuts;
    double min_cut_score = 1e17;

    for (const EdgeIndices& ei : concave_edge_indices) {
        vector<Plane> cutting_planes = m.get_cutting_planes(ei, NUM_CUTTING_PLANES);
        for (Plane& plane : cutting_planes) {
            map<double, Mesh> curr_costs;

            DEBUG_MSG("Cutting plane...");
            vector<Mesh> frags = m.cut_plane(plane);
            DEBUG_MSG("Done cutting");

            if (frags.empty()) {
                // If failed to cut any components, skip
                DEBUG_MSG("Failed to cut plane along edge " << ei.ai_ << ", " << ei.bi_
                                                            << ". Saving and continuing...");
                for (int i = 0; i < cutting_planes.size(); i++) {
                    cutting_planes[i].save_to_file("out/plane_failed" + to_string(i) + ".obj");
                }
                m.save_to_file("out/mesh_failed.obj");
                continue;
                exit(EXIT_FAILURE);
            }

            DEBUG_MSG("Computing concavity...");
            auto t1 = chrono::high_resolution_clock::now();

            double cut_store = -1 * ConcavityMetric::concavity(m);

            for (const auto& frag : frags) {
                double cost = ConcavityMetric::concavity(frag);
                cut_store += cost;
                curr_costs[cost] = frag;
            }

            if (cut_store < min_cut_score) {
                min_cut_score = cut_store;
                best_cuts = cutting_planes;
                best_frags = curr_costs;
            }

            auto t2 = chrono::high_resolution_clock::now();
            DEBUG_MSG("Total time: " << chrono::duration_cast<chrono::milliseconds>(t2 - t1).count()
                                     << "ms");
        }
    }
    return {best_frags, best_cuts};
}
