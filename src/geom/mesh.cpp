#include "mesh.h"

#include "geom/utils.h"
#include "mcut/mcut.h"

using namespace std;
using namespace Eigen;

#define ASSERT(cond)                                \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::exit(1);                               \
    }

constexpr string_view OUT_DIR = "fragments/";

inline double signed_tri_volume(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3) {
    // From here: https://stackoverflow.com/a/1568551
    return p1.dot(p2.cross(p3)) / 6.;
}

double Mesh::volume() const {
    double volume = 0;
    for (auto const& tri : this->m_triangles) {
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
    for (const auto& v : this->m_verts) {
        point_cloud.emplace_back(v[0], v[1], v[2]);
    }

    quickhull::ConvexHull<float> hull = qh.getConvexHull(point_cloud, true, false, success);
    // If quick hull computation fails, fall back to stable CH computation
    if (!success) {
        return this->computeVCH();
    }

    // Convert back from QH representation to mesh
    const auto& ibuf = hull.getIndexBuffer();
    const auto& vbuf = hull.getVertexBuffer();

    vector<Vector3f> new_verts;
    vector<Vector3i> new_tris;
    new_verts.reserve(vbuf.size());
    new_tris.reserve(ibuf.size() / 3);
    for (const auto& v : vbuf) {
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
    for (const auto& v : this->m_verts) {
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
        const auto* e = &ch.edges[ch.faces[i]];
        const auto* next_e = e->getNextEdgeOfFace();
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

std::vector<Mesh> Mesh::merge(const std::vector<Mesh>& Q) {
    // TODO: implement
    return {};
}

std::vector<Mesh> Mesh::cut_plane(Plane& p) {
    auto [p0, p1, p2, p3] = p.bounds();

    double cutMeshVertices[] = {p0.x(), p0.y(), p0.z(), p1.x(), p1.y(), p1.z(),
                                p2.x(), p2.y(), p2.z(), p3.x(), p3.y(), p3.z()};

    uint32_t cutMeshFaces[] = {// arbitrary, just to trimesh the plane
                               1, 2, 0, 1, 3, 2};

    uint32_t numCutMeshVertices = 4;
    uint32_t numCutMeshFaces = 2;

    int numVertices = m_verts.size();
    int numFaces = m_triangles.size();
    double vertices[numVertices * 3];
    uint32_t faces[numFaces * 3], faceSizes[numFaces];

    int i = 0;
    for (const Vector3f& v : m_verts) {
        for (int j = 0; j < 3; j++) {
            vertices[i++] = static_cast<double>(v[j]);
        }
    }

    i = 0;
    int i2 = 0;
    for (const Vector3i& f : m_triangles) {
        for (int j = 0; j < 3; j++) {
            faces[i++] = static_cast<uint32_t>(f[j]);
        }
        faceSizes[i2++] = 3;
    }

    McContext context = MC_NULL_HANDLE;
    McResult err = mcCreateContext(&context, MC_NULL_HANDLE);

    ASSERT(err == MC_NO_ERROR);

    auto MC_DISPATCH_FILTER_CLOSED_FRAGMENTS =
        (MC_DISPATCH_REQUIRE_THROUGH_CUTS | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE |
         MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE);

    // 3. do the magic!
    // ----------------
    err =
        mcDispatch(context, (MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_CLOSED_FRAGMENTS),
                   vertices, faces, faceSizes, numVertices, numFaces, cutMeshVertices, cutMeshFaces,
                   nullptr,  // cutMeshFaceSizes, // no need to give 'faceSizes' parameter since
                             // cut-mesh is a triangle mesh
                   numCutMeshVertices, numCutMeshFaces);

    ASSERT(err == MC_NO_ERROR);

    // 4. query the number of available connected component (here, we're only interested in the
    // fragments)
    // -------------------------------------------------------------
    uint32_t numConnComps;
    std::vector<McConnectedComponent> connComps;

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, 0, NULL,
                                   &numConnComps);

    ASSERT(err == MC_NO_ERROR);

    if (numConnComps == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    connComps.resize(numConnComps);

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT,
                                   (uint32_t)connComps.size(), connComps.data(), NULL);

    ASSERT(err == MC_NO_ERROR);

    // 5. query the data of each connected component from MCUT
    // -------------------------------------------------------

    for (int i = 0; i < (int)connComps.size(); ++i) {
        McConnectedComponent connComp = connComps[i];  // connected compoenent id

        McSize numBytes = 0;

        // query the vertices
        // ----------------------

        err = mcGetConnectedComponentData(
            context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &numBytes);

        ASSERT(err == MC_NO_ERROR);

        uint32_t numberOfVertices = (uint32_t)(numBytes / (sizeof(float) * 3));

        std::vector<float> vertices(numberOfVertices * 3u);

        err =
            mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT,
                                        numBytes, (void*)vertices.data(), NULL);

        ASSERT(err == MC_NO_ERROR);

        // query the faces
        // -------------------

        err = mcGetConnectedComponentData(
            context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);
        ASSERT(err == MC_NO_ERROR);
        std::vector<uint32_t> faceIndices(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp,
                                          MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes,
                                          faceIndices.data(), NULL);
        ASSERT(err == MC_NO_ERROR);

        std::vector<uint32_t> faceSizes(faceIndices.size() / 3, 3);
        printf("faces: %d\n", (int)faceSizes.size());

        ostringstream fname;
        fname << OUT_DIR << "cc" << i << ".obj";

        std::vector<float> f;
        for (uint32_t i = 0; i < (uint32_t)vertices.size(); ++i) f.push_back(vertices[i]);

        // save to mesh file (.obj)
        // ------------------------
        writeOBJ(fname.str(), (float*)f.data(), (uint32_t)vertices.size() / 3,
                 (uint32_t*)faceIndices.data(), (uint32_t*)faceSizes.data(),
                 (uint32_t)faceSizes.size());
    }

    // 6. free connected component data
    // --------------------------------
    err = mcReleaseConnectedComponents(context, 0, NULL);

    ASSERT(err == MC_NO_ERROR);

    // 7. destroy context
    // ------------------
    err = mcReleaseContext(context);

    ASSERT(err == MC_NO_ERROR);

    // TODO: remake Mesh objects for two connected components
    std::vector<Mesh> out;
    return out;
}

// go through each triangle and calculate area
float Mesh::compute_tri_areas() {
    float total_area = 0.f;

    for (int i = 0; i < m_triangles.size(); ++i) {
        auto& t = m_triangles[i];

        auto& v1 = m_verts[t[0]];
        auto& v2 = m_verts[t[1]];
        auto& v3 = m_verts[t[2]];

        // area of a triangle is two of its vectors crossed / 2
        auto tri_area = (v2 - v1).cross(v3 - v1).norm() / 2.f;
        total_area += tri_area;
        m_tri_areas[i] = tri_area;
    }

    return total_area;
}

vector<Vector3f> Mesh::boundary_sample(int samples_per_unit_area) {
    // samples is based on the total surface area
    int num_samples = static_cast<int>(m_surface_area * samples_per_unit_area);

    vector<Vector3f> samples;
    samples.reserve(num_samples);

    // with the number of samples, sample a random triangle based on its area and sample a point on
    // it
    for (int i = 0; i < num_samples; ++i) {
    }

    return samples;
}
