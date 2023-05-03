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

    // Make quickhull and point cloud, for CH computation
    quickhull::QuickHull<float> qh;  // TODO: float or double?
    vector<quickhull::Vector3<float>> point_cloud;
    point_cloud.reserve(this->m_verts.size());
    // Populate point cloud
    for (const auto &v : this->m_verts) {
        point_cloud.emplace_back(v[0], v[1], v[2]);
    }

    quickhull::ConvexHull<float> hull = qh.getConvexHull(point_cloud, true, false, success);
    // If quick hull computation fails, fall back to stable CH computation
    if (!success) {
        return this->computeVCH();
    }

    // Convert back from QH representation to mesh
    const auto &ibuf = hull.getIndexBuffer();
    const auto &vbuf = hull.getVertexBuffer();

    vector<Vector3f> new_verts;
    vector<Vector3i> new_tris;
    new_verts.reserve(vbuf.size());
    new_tris.reserve(ibuf.size() / 3);
    for (const auto &v : vbuf) {
        new_verts.emplace_back(v.x, v.y, v.z);
    }
    for (int i = 0; i < ibuf.size(); i += 3) {
        // Preserve chirality from quickhull representation
        new_tris.emplace_back(ibuf[i + 2], ibuf[i + 1], ibuf[i]);
    }

    Mesh new_mesh(new_verts, new_tris);
    return new_mesh;
}

Mesh Mesh::computeVCH() const {
    btConvexHullComputer ch;
    // Convert verts to vector<array<double, 3>>
    vector<array<double, 3>> points;
    points.reserve(this->m_verts.size());
    for (const auto &v : this->m_verts) {
        points.push_back({v[0], v[1], v[2]});
    }

    // TODO: see if I can just tell btConvexHullComputer to interpret a vector<Vector3f> as a
    // vector<array<double, 3>>; looking at the internal CH computation, it looks like it should be
    // fine, but not confident.
    ch.compute(points, -1.0, -1.0);

    // Convert back from CH to Mesh
    vector<Vector3f> new_verts;
    vector<Vector3i> new_tris;
    new_verts.reserve(ch.vertices.size());
    new_tris.reserve(ch.faces.size());  // could need more, but use as a baseline

    for (int32_t i = 0; i < ch.vertices.size(); i++) {
        new_verts.emplace_back(ch.vertices[i].getX(), ch.vertices[i].getY(), ch.vertices[i].getZ());
    }
    for (int32_t i = 0; i < ch.faces.size(); i++) {
        const auto *e = &ch.edges[ch.faces[i]];
        const auto *next_e = e->getNextEdgeOfFace();
        auto a = e->getSourceVertex(), b = e->getTargetVertex(), c = next_e->getTargetVertex();
        while (a != c) {
            new_tris.emplace_back(a, b, c);
            next_e = next_e->getNextEdgeOfFace();
            b = c;
            c = next_e->getTargetVertex();
        }
    }

    Mesh new_mesh(new_verts, new_tris);
    return new_mesh;
}

// go through each triangle and calculate area
float Mesh::calc_triangle_areas() {

    float total_area = 0.f;

    for (int i = 0; i < m_triangles.size(); ++i) {

        auto& t = m_triangles[i];

        auto& v1 = m_verts[t[0]];
        auto& v2 = m_verts[t[1]];
        auto& v3 = m_verts[t[2]];

        // area of a triangle is two of its vectors crossed / 2
        auto tri_area = (v2 - v1).cross(v3 - v1).norm() / 2.f;
        total_area += tri_area;
        tri_to_area[i] = tri_area;
    }

    return total_area;
}

vector<Vector3f> Mesh::boundary_sample(int samples_per_unit_area) {

    // samples is based on the total surface area
    int num_samples = static_cast<int>(total_surface_area * samples_per_unit_area);

     vector<Vector3f> samples;
     samples.reserve(num_samples);

    // with the number of samples, sample a random triangle based on its area and sample a point on it
    for (int i = 0; i < num_samples; ++i) {



    }

    return samples;
}
