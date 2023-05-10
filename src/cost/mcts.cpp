#include "mcts.h"

#include "cost/concavity.h"

// ============ MCTS =============

ComponentsQueue MCTS::MCTS_search(const Mesh& cur_mesh)  {

    // Create root node v0 with input mesh

    TreeNode root = {.depth = 0};


    for (int iter = 0; iter < ITERATIONS; ++iter) {


        // TreePolicy
        auto [root_planes, intermediate_planes] = tree_policy(&root, MAX_DEPTH);




    }


}


std::pair<std::unordered_set<Plane>, TreeNode*> MCTS::tree_policy(TreeNode* v, int depth) {
    std::unordered_set<Plane> S;




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
        vector<Edge> concave_edges = get_concave_edges(worst_mesh);
        deque<Edge> sorted_concave_edges =
            ConcavityMetric::sort_concave_edges(worst_mesh, concave_edges);

        // trim down number of concave edges
        vector<Edge> selected_concave_edges(MAX_NUM_EDGES);
        for (auto i = 0; i < MAX_NUM_EDGES && !sorted_concave_edges.empty(); i++) {
            selected_concave_edges[i] = sorted_concave_edges.front();
            sorted_concave_edges.pop_front();
        }

        // Get the best cut, remove the old Mesh, then insert the new fragments
        auto [frag1, frag2] = get_best_cut(selected_concave_edges, worst_mesh);
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
