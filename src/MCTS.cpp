// Monte Carlo Tree Search
#include <iostream>
#include <cmath>   // For math library functions
#include <cstring> // For string library functions
#include <vector>
#include <tuple>

struct Plane
{

}

struct Node
{
    Vector3f position;
    int index;
    Node *next;

}

Plane
MCTS(int iterations, int depth)
{
    int i = 0;
    Node root = m_vertices[0] // root is first by convention
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

std::tuple<vector<Plane>, int> TreePolicy(Node v, int depth)
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

std::tuple<vector<Plane>, int> DefaultPolicy(Node v, int depth)
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
