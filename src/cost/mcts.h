#pragma once

#include "geom/mesh.h"

class MCTS
{
public:
    MCTS();

    static quickhull::Plane<double> cuttingPlane(const Mesh& cur_mesh);

    static map<double,Mesh> greedySearch(const Mesh& cur_mesh);

   static vector<Edge> getConcaveEdges(const Mesh& mesh);

   static std::pair<Mesh,Mesh> getBestCut(const vector<Edge>& concave_edges, Mesh& m);

private:

};
