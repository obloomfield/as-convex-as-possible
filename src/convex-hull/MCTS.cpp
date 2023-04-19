// Monte Carlo Tree Search
#include <iostream>
#include <cmath>   // For math library functions
#include <cstring> // For string library functions
#include <vector>
#include <tuple>

#include "MCTS.h"

using namespace Eigen;
using namespace std;

Plane MCTS::cuttingPlane(const Mesh& mesh) {
    return mcts(mesh, ITERATIONS, DEPTH);
}

Plane MCTS::mcts(const Mesh& mesh, int iterations, int depth)
{
    int i = 0;
    Node root = m_vertices[0] // TODO: use Mesh object instead of m_vertices
    while (i < iterations)
    {
        vector<Plane> select_planes, Node v_i = TreePolicy(root, depth);
        vector<Plane> default_planes = DefaultPolicy(root, depth);
        select_planes.insert(select_planes.end(), default_planes.begin(), default_planes.end());

        float score = PlaneQuality(select_planes);
        Backup(v_i, score);
    }
    Node v_star = ValueFunction(root->next);
    return node_to_plane[v_star.index];
}

tuple<vector<Plane>, int> MCTS::TreePolicy(Node v, int depth)
{
    vector<Plane> select_planes;
    while (depth(v) < depth)
    {
        float c_prime = argmax(ConcavMetric(v));
        bool all_expanded;
        if (all_expanded)
        {
            select_planes.push_back(v.getPlane());
        }
        else
        {
            Plane p = c_prime.getRandomPlane();
            c_prime_left, c_prime_right = cutPlane(c_prime);
            Node v_prime = makeNewChild(c_prime_left, c_prime_right, p);
            select_planes.push_back(p);
            return select_planes, v_prime;
        }
    }
    return select_planes, v;
}

tuple<vector<Plane>, int> MCTS::DefaultPolicy(Node v, int depth)
{
    vector<Plane> select_planes;
    while (depth(v) < depth)
    {
        float c_prime = argmax(ConcavMetric(v));
        bool all_expanded;
        if (all_expanded)
        {
            select_planes.push_back(v.getPlane());
        }
        else
        {
            Plane p = c_prime.getRandomPlane();
            c_prime_left, c_prime_right = cutPlane(c_prime);
            Node v_prime = makeNewChild(c_prime_left, c_prime_right, p);
            select_planes.push_back(p);
            return select_planes, v_prime;
        }
    }
    return select_planes, v;
}
