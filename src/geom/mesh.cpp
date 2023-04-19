#include "mesh.h"

using namespace std;
using namespace Eigen;


Mesh Mesh::VCH() const {
    Mesh new_mesh;

    ConvexHull ch;
    ch.calculate(m_verts);
    for (const Vector3f& v : ch.getVertices()) new_mesh.m_verts.push_back(v);

    // TODO: how to populate edges and faces
    return new_mesh;
}
