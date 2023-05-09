#include "acap.h"

#include <array>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#include "cost/concavity.h"
#include "cost/mcts.h"
#include "graphics/meshloader.h"

using namespace std;
using namespace Eigen;

#define MESH_PATH "meshes/bunny.obj"

// Here are some helpful controls for the application
//
// - You start in first-person camera mode
//   - WASD to move, left-click and drag to rotate
//   - R and F to move vertically up and down
//
// - C to change to orbit camera mode
//
// - Right-click (and, optionally, drag) to anchor/unanchor points
//   - Left-click an anchored point to move it around
//
// - Minus and equal keys (click repeatedly) to change the size of the vertices
void ACAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax) {
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;

    // If this doesn't work for you, remember to change your working directory
    if (MeshLoader::loadTriMesh(MESH_PATH, vertices, triangles)) {
        m_shape.init(vertices, triangles);
    }

    Mesh mesh(m_shape);  // our custom datatype
        Plane p(Vector3d(-1.2, 1.6, 1.5), Vector3d(1.4, -1.3, 1.5), Vector3d(-1.2, 1.6, -1.005929),
                Vector3d(1.4, -1.3, -1.005929));
    vector<Mesh> fragments = mesh.cut_plane(p);

//    assert(fragments.size() == 2);

    return;

    vector<Mesh> decomp = ACD(mesh);

    // Students, please don't touch this code: get min and max for viewport stuff
    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
    int i = 0;
    for (unsigned long i = 0; i < vertices.size(); ++i) {
        all_vertices.row(i) = vertices[i];
    }
    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

// NEED REPRESENTATIONS FOR:
// - Mesh
// - Plane
vector<Mesh> ACAP::ACD(Mesh& mesh) {
    vector<Mesh> Q = {mesh};
    vector<Mesh> D, mesh_parts;

    while (Q.size() > 0) {
        vector<Mesh> new_Q;
        for (const Mesh &cur_mesh : Q) {
            float cost = ConcavityMetric::concavity(cur_mesh);

            if (cost > COST_THRESHOLD) {
                // TODO: MONTE CARLO TREE SEARCH
                quickhull::Plane p = MCTS::cuttingPlane(cur_mesh);

                // TODO: Clip by plane (refine and cut)
                vector<Mesh> fragments = mesh.cut_plane(p);
                Mesh cL = fragments[0], cR = fragments[1];

                if (cL.m_verts.size() > 0) new_Q.push_back(cL);
                if (cR.m_verts.size() > 0) new_Q.push_back(cR);
            } else {  // put back!
                auto convex = cur_mesh.computeCH();
                D.push_back(convex);
                mesh_parts.push_back(cur_mesh);
            }
        }
        Q.clear();
        Q = new_Q;
        new_Q.clear();
    }

    // TODO: implement merge
    return mesh.merge(Q);
}
