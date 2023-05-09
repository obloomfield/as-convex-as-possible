#include "mcts.h"
#include "cost/concavity.h"
#include <set>
#include <algorithm>
MCTS::MCTS()
{

}

quickhull::Plane<double> MCTS::cuttingPlane(const Mesh& cur_mesh) {
    // TODO: implement
    quickhull::Plane<double> p;
    return p;
}



vector<Edge> MCTS::getConcaveEdges(const Mesh& mesh) {
    vector<Edge> concave_edges;
    for (const auto& pair : mesh.m_edge_tris) {
            Edge edge = pair.first;
            auto vector3iArray = pair.second;
            //first, we get the the indices that the triangles DONT share
            // so that we can construct two vectors and check their angle to check concavity
            Vector3i tri_one = vector3iArray[0];
            Vector3i tri_two = vector3iArray[1];
            set<int> tri_one_set = set<int>{tri_one[0],tri_one[1],tri_one[2]};
            set<int> tri_two_set = set<int>{tri_two[0],tri_two[1],tri_two[2]};
            vector<int> difference;
            set_difference(tri_one_set.begin(), tri_one_set.end(), tri_two_set.begin(), tri_two_set.end(), std::inserter(difference, difference.begin()));
            assert(difference.size() == 2);
            Vector3d u = mesh.m_verts[difference[0]] - edge.a_;
            Vector3d v = mesh.m_verts[difference[1]] - edge.a_;
            double dotProduct = u.dot(v);

            //is angle less than 180 (is edge concave)? Add to list:
            if (dotProduct >= 0.0) {
                concave_edges.push_back(edge);
            }
        }
        return concave_edges;
}


std::pair<Mesh,Mesh> MCTS::getBestCut(const vector<Edge>& concave_edges, Mesh& m) {

    Vector3d u = Vector3d{0.0,0.0,0.0};
    std::pair<Mesh,Mesh> best_pair;
    double min_cut_score = 1e17;

    for (const Edge& edge : concave_edges) {
        vector<Plane> edge_planes = m.get_cutting_planes(edge,5);
        for (Plane& plane : edge_planes) {
            vector<Mesh> frags = m.cut_plane(plane);
            if (frags.size() != 2) {
                continue;
            }
            Mesh frag_one = frags[0];
            Mesh frag_two = frags[1];
            double curr_cut_score = ConcavityMetric::concavity(frag_one) + ConcavityMetric::concavity(frag_two) - ConcavityMetric::concavity(m);
            if (curr_cut_score < min_cut_score) {
                min_cut_score = curr_cut_score;
                best_pair = std::make_pair(frag_one,frag_two);
            }

        }
    }
    return best_pair;




}


map<double,Mesh> MCTS::greedySearch(const Mesh& cur_mesh) {
    const int MAX_NUM_PIECES = 9;
    const int MAX_NUM_EDGES = 5;

    double cost = ConcavityMetric::concavity(cur_mesh);
    map<double,Mesh> cost_to_mesh;
    cost_to_mesh.insert(std::make_pair(cost,cur_mesh));

    while (cost_to_mesh.size() > 0 && cost_to_mesh.size() < MAX_NUM_PIECES ) {
        // TODO: find the piece inside the map that has the worst concavity score.
        auto lastElementIt = cost_to_mesh.rbegin();
        Mesh worst_mesh = lastElementIt->second;


        // TODO: get concave edges of worst shape
        vector<Edge> concave_edges = getConcaveEdges(worst_mesh);
        deque<Edge> sorted_concave_edges = ConcavityMetric::sort_concave_edges(worst_mesh,concave_edges);

        //trim down number of concave edges
        int i = MAX_NUM_EDGES;
        std::vector<Edge> selected_concave_edges(MAX_NUM_EDGES);
        while (i > 0 && !sorted_concave_edges.empty()) {
           selected_concave_edges[i] = (sorted_concave_edges.front());
            sorted_concave_edges.pop_front();
            i--;
        }

        // TODO: get best cut and insert new fragments, remove old piece
        std::pair<Mesh,Mesh> best_pair = getBestCut(selected_concave_edges,worst_mesh);
        cost_to_mesh.erase(lastElementIt->first);
        cost_to_mesh.insert(std::make_pair(ConcavityMetric::concavity(best_pair.first),best_pair.first));
        cost_to_mesh.insert(std::make_pair(ConcavityMetric::concavity(best_pair.second),best_pair.second));


    }
    return cost_to_mesh;

}
