#include "mesh.h"

using namespace std;
using namespace Eigen;

inline double signed_tri_volume(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3) {
    // From here: https://stackoverflow.com/a/1568551
    return p1.dot(p2.cross(p3)) / 6.;
}

double Mesh::volume() const {
    double volume = 0;
    for (auto const &tri : this->m_triangles) {
        int i0 = tri[0], i1 = tri[1], i2 = tri[2];
        volume += signed_tri_volume(this->m_verts[i0], this->m_verts[i1], this->m_verts[i2]);
    }
    return abs(volume);
}

Mesh Mesh::computeCH() const {
    bool success = true;
}
