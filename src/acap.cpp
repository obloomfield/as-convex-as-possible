#include "acap.h"
#include "graphics/meshloader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <array>

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


/*
 *
 * Notes:
 *
 * Regenerate L and weights on click release. When L is regenerated, recompute the factorization
 * Also set "previous vertices" on click release to be used as initial positions
 *
 * Do we need the weights anywhere else other than for L? Might be good to store it regardless?
 *
 */

void ACAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;

    // If this doesn't work for you, remember to change your working directory
    if (MeshLoader::loadTriMesh(MESH_PATH, vertices, triangles)) {
        m_shape.init(vertices, triangles);
    }

    Mesh m_mesh(m_shape); // our custom datatyp

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


vector<Mesh> ACAP::ACD(Mesh mesh) {
    vector<Mesh> Q = {mesh};
    vector<Mesh> D;

    while (Q.size() > 0) {
        for (const Mesh& cur_mesh : Q) {
            Mesh convex = cur_mesh.VCH();
        }
    }
}


