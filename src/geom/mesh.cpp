#include "mesh.h"

using namespace std;
using namespace Eigen;

const bool PRINT_INTERMEDIATE_OBJ = false;

#define ASSERT(cond)                                \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::exit(1);                               \
    }

constexpr string_view OUT_DIR = "fragments/";

Mesh::Mesh(std::vector<Eigen::Vector3d> verts, std::vector<Eigen::Vector3i> tris)
    : m_verts(verts), m_triangles(tris) {
    // Compute initial surface area and bounding box
    m_surface_area = compute_tri_areas();
    m_bbox = compute_bounding_box();

    // For each triangle, get its edges; assign them to this triangle (each edge should have two
    // triangles)
    for (auto&& tri : tris) {
        auto edges = this->get_triangle_edges(tri);
        for (auto&& e : edges) {
            // If edge already exists in map, assign to 2nd triangle; otherwise, assign to 1st
            this->m_edge_tris[e][this->m_edge_tris.contains(e) ? 1 : 0] = tri;
        }
    }

    // should have 1.5e = f?
    cout << "Num edges: " << this->m_edge_tris.size()
         << "\tNum triangles: " << this->m_triangles.size() << endl;
}

array<Edge, 3> Mesh::get_triangle_edges(const Vector3i& tri) const {
    auto t = this->get_triangle(tri);
    return {Edge(t[0], t[1]), Edge(t[1], t[2]), Edge(t[2], t[0])};
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

    vector<Vector3d> new_verts;
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
    // fine, but not confident. Also doesn't matter that much, just slightly inefficient to copy...
    ch.compute(points, -1.0, -1.0);

    // Convert back from CH to Mesh
    vector<Vector3d> new_verts;
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

double Mesh::volume() const {
    double volume = 0;
    for (auto const& tri : this->m_triangles) {
        int i0 = tri[0], i1 = tri[1], i2 = tri[2];
        volume += signed_tri_volume(this->m_verts[i0], this->m_verts[i1], this->m_verts[i2]);
    }
    return abs(volume);
}

vector<Plane> Mesh::get_cutting_planes(const Edge& concave_edge, int k) {
    auto bbox = this->bounding_box();
    vector<Plane> res;
    res.reserve(k);

    // Get triangles associated with concave edge of interest
    auto [t1_inds, t2_inds] = this->m_edge_tris.at(concave_edge);
    auto t1 = this->get_triangle(t1_inds), t2 = this->get_triangle(t2_inds);
    // Compute normals for the concave edge to each of the triangles
    auto n1 = edge_tri_norm(concave_edge, t1), n2 = edge_tri_norm(concave_edge, t2);
    // Get Quaternions for n1 -> n2
    Quaterniond qa = Quaterniond::Identity(), qb = Quaterniond::FromTwoVectors(n1, n2);
    // slerp it k times
    for (double t = 0., step = 1. / k; t < 1.; t += step) {
        auto norm = qa.slerp(t, qb) * n1;
        // Now, construct a plane from the concave edge, the norm, and the mesh's bounding box
        auto plane = Plane(concave_edge, norm, bbox);
        res.push_back(plane);
    }

    return res;
}

vector<Mesh> Mesh::cut_plane(quickhull::Plane<double>& p) const {
    Plane bound_plane = Plane(p, this->bounding_box());
    return cut_plane(bound_plane);
}

std::vector<Mesh> Mesh::cut_plane(Plane& p) const {
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
    for (const Vector3d& v : m_verts) {
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
    std::vector<Mesh> out;
    //    assert((int)connComps.size() == 2);
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
        // TODO: remove

        if (PRINT_INTERMEDIATE_OBJ)
            writeOBJ(fname.str(), (float*)f.data(), (uint32_t)vertices.size() / 3,
                     (uint32_t*)faceIndices.data(), (uint32_t*)faceSizes.data(),
                     (uint32_t)faceSizes.size());

        vector<Vector3d> verts = vec3f_to_vec3d(float_to_vec3f(vertices));
        vector<Vector3i> faces = uint_to_vec3i(faceIndices);
        out.push_back(Mesh(verts, faces));
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

    return out;
}

pair<vector<Vector3d>, vector<int>> Mesh::sample_point_set(int resolution) const {
    // TODO: implement (extract samples, and get their corresponding triangles)

    // samples is based on the total surface area
    int num_samples = static_cast<int>(m_surface_area * resolution);

    // initialize sample vectors
    vector<Vector3d> point_samples;
    vector<int> tri_ind_samples;
    point_samples.reserve(num_samples);
    tri_ind_samples.reserve(num_samples);

    // with the number of samples, sample a random triangle based on its area and sample a point on
    // it
    for (int i = 0; i < num_samples; ++i) {
        // draw a random number and scale it by the total surface area
        float random_area = rand_f() * m_surface_area;
        // next, find the index the draw corresponds to
        auto it = std::lower_bound(m_cdf.begin(), m_cdf.end(), random_area);
        size_t index = std::distance(m_cdf.begin(), it);  // index of triangle sampled

        // if lower_bound returns end, set it to be last index
        if (index == m_triangles.size()) index = m_triangles.size() - 1;

        // add triangle index to sample
        tri_ind_samples[i] = index;

        // get the triangle
        Vector3i tri = m_triangles[index];

        // sample barycentric coordinate of tri and add it to the samples
        point_samples[i] = random_barycentric_coord(m_verts[0], m_verts[1], m_verts[2]);
    }

    return {point_samples, tri_ind_samples};
}

vector<Edge> Mesh::shared_edges(const Vector3i& tri1, const Vector3i& tri2) {
    vector<Edge> shared;
    auto edges1 = this->get_triangle_edges(tri1), edges2 = this->get_triangle_edges(tri2);
    for (auto&& e1 : edges1) {
        for (auto&& e2 : edges2) {
            if (e1 == e2) shared.push_back(e1);
        }
    }
    return shared;
}

vector<Edge> Mesh::get_concave_edges() const {
    vector<Edge> concave_edges;
    for (const auto& [edge, tris] : this->m_edge_tris) {
        // Get the triangle points corresponding to each triangle's indices
        auto [tri1_indices, tri2_indices] = tris;
        Triangle tri1 = this->get_triangle(tri1_indices);
        Triangle tri2 = this->get_triangle(tri2_indices);

        // Get the vertices that the triangles don't share
        Vector3d v1 = get_third_point(tri1, edge), v2 = get_third_point(tri2, edge);

        // Check angle between the two triangles
        Vector3d u = v1 - edge.a_, v = v2 - edge.a_;
        double dot = u.dot(v);

        // is angle less than 180 (is edge concave)? Add to list:
        if (dot >= 0.0) {
            concave_edges.push_back(edge);
        }
    }
    return concave_edges;
}

Mesh Mesh::load_from_file(const std::string& path) {
    vector<Vector3i> faces;
    vector<Vector3f> fverts;
    if (!MeshLoader::loadTriMesh(path, fverts, faces)) {
        exit(EXIT_FAILURE);
    }

    auto verts = vec3f_to_vec3d(fverts);

    // Construct and return a mesh from the vertices
    return Mesh(verts, faces);
}

void Mesh::save_to_file(const string& path) {
    ofstream outfile;
    outfile.open(path);

    // Write vertices
    for (size_t i = 0; i < m_verts.size(); i++) {
        const Vector3d& v = m_verts[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces
    for (size_t i = 0; i < m_triangles.size(); i++) {
        const Vector3i& f = m_triangles[i];
        outfile << "f " << (f[0] + 1) << " " << (f[1] + 1) << " " << (f[2] + 1) << endl;
    }

    outfile.close();
}

std::vector<Mesh> Mesh::merge(const std::vector<Mesh>& Q) {
    // TODO: implement
    return {};
}

array<double, 6> Mesh::compute_bounding_box() {
    if (m_verts.empty()) {
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    // Initialize min and max coordinates to the first vertex
    Vector3d minCoords = m_verts[0];
    Vector3d maxCoords = m_verts[0];

    // Iterate over all vertices to update min and max coordinates
    for (const auto& v : m_verts) {
        minCoords.x() = std::min(minCoords.x(), v.x());
        minCoords.y() = std::min(minCoords.y(), v.y());
        minCoords.z() = std::min(minCoords.z(), v.z());

        maxCoords.x() = std::max(maxCoords.x(), v.x());
        maxCoords.y() = std::max(maxCoords.y(), v.y());
        maxCoords.z() = std::max(maxCoords.z(), v.z());
    }

    return {minCoords.x(), minCoords.y(), minCoords.z(),
            maxCoords.x(), maxCoords.y(), maxCoords.z()};
}

Vector3d Mesh::random_barycentric_coord(const Vector3d& p1, const Vector3d& p2,
                                        const Vector3d& p3) {
    float w1 = rand_f();
    float w2 = rand_f();

    if (w1 + w2 >= 1.f) {
        w1 = 1.f - w1;
        w2 = 1.f - w2;
    }

    return (p1 * w1) + (p2 * w2) + (p3 * (1.f - (w1 + w2)));
}

double Mesh::angle_between_tris(const Vector3i& t1, const Vector3i& t2) {
    const vector<Vector3d>& verts = m_verts;
    Vector3d u0 = verts[t1[0]], u1 = verts[t1[1]], u2 = verts[t1[2]];
    Vector3d v0 = verts[t2[0]], v1 = verts[t2[1]], v2 = verts[t2[2]];

    Vector3d n1 = (u1 - u0).cross(u2 - u0);
    n1.normalize();

    Vector3d n2 = (v1 - v0).cross(v2 - v0);
    n2.normalize();

    double dotProduct = n1.dot(n2);
    double angle = acos(dotProduct);
    return angle;
}

// go through each triangle and calculate area
float Mesh::compute_tri_areas() {
    float total_area = 0.f;
    m_tri_areas.reserve(m_triangles.size());
    m_cdf.reserve(m_triangles.size());

    for (int i = 0; i < m_triangles.size(); ++i) {
        auto& t = m_triangles[i];

        auto& v1 = m_verts[t[0]];
        auto& v2 = m_verts[t[1]];
        auto& v3 = m_verts[t[2]];

        // area of a triangle is two of its vectors crossed / 2
        auto tri_area = (v2 - v1).cross(v3 - v1).norm() / 2.f;
        total_area += tri_area;
        m_cdf[i] = total_area;
        m_tri_areas[i] = tri_area;
    }

    return total_area;
}
