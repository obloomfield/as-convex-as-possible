#include "mesh.h"

using namespace std;
using namespace Eigen;

constexpr bool PRINT_INTERMEDIATE_OBJ = false;

#define ASSERT(cond)                                                                      \
    do {                                                                                  \
        if (!(cond)) {                                                                    \
            fprintf(stderr, "MCUT error (line %d): %s. Got %d\n", __LINE__, #cond, cond); \
            std::exit(1);                                                                 \
        }                                                                                 \
    } while (0)

#define ASSERT_NO_ERROR(err)                                               \
    do {                                                                   \
        if ((err) != MC_NO_ERROR) {                                        \
            fprintf(stderr, "MCUT error (line %d): %lu\n", __LINE__, err); \
            std::exit(1);                                                  \
        }                                                                  \
    } while (0)

constexpr string_view OUT_DIR = "fragments/";

Mesh::Mesh(std::vector<Eigen::Vector3d> verts, std::vector<Eigen::Vector3i> tris, int scale)
    : m_verts(verts), m_triangles(tris) {
    // Scale each vertex by a scale factor
    //    int n_verts = m_verts.size();
    //    for (int i = 0; i < n_verts; i++) {
    //        m_verts[i] *= scale;
    //    }

    // Compute initial surface area and bounding box
    m_surface_area = compute_tri_areas();
    m_bbox = compute_bounding_box();

    // For each triangle, get its edges; assign them to this triangle (each edge
    // should have two triangles)
    for (auto &&tri : tris) {
        auto edges = this->get_triangle_edge_indices(tri);
        for (const auto &e : edges) {
            // If edge already exists in map, assign to 2nd triangle; otherwise,
            // assign to 1st
            if (this->m_edge_tris.contains(e))
                this->m_edge_tris[e][1] = tri;
            else
                this->m_edge_tris[e][0] = tri;
        }
    }
}

array<Edge, 3> Mesh::get_triangle_edges(const Vector3i &tri) const {
    auto t = this->get_triangle(tri);
    return {Edge(t[0], t[1]), Edge(t[1], t[2]), Edge(t[2], t[0])};
}

array<EdgeIndices, 3> Mesh::get_triangle_edge_indices(const Vector3i &tri) const {
    return {EdgeIndices(tri[0], tri[1]), EdgeIndices(tri[1], tri[2]), EdgeIndices(tri[2], tri[0])};
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

    vector<Vector3d> new_verts;
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

    // TODO: see if I can just tell btConvexHullComputer to interpret a
    // vector<Vector3f> as a vector<array<double, 3>>; looking at the internal CH
    // computation, it looks like it should be fine, but not confident. Also
    // doesn't matter that much, just slightly inefficient to copy...
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

double Mesh::volume() const {
    double volume = 0;
    for (auto const &tri : this->m_triangles) {
        int i0 = tri[0], i1 = tri[1], i2 = tri[2];
        volume += signed_tri_volume(this->m_verts[i0], this->m_verts[i1], this->m_verts[i2]);
    }
    return abs(volume);
}

//
double MIN_INTERVAL = 0.01;
vector<Plane> Mesh::get_axis_aligned_planes(int k) const {
    // k: number of cuts per each axis
    auto [a, b, c, x, y, z] = this->bounding_box();
    Vector3d minBounds(a, b, c);
    Vector3d maxBounds(x, y, z);
    auto bbox = this->bounding_box();

    vector<Plane> res;
    res.reserve(k * 3);
    double interval;
    interval = max(MIN_INTERVAL, abs(a - x) / ((double)k + 1));
    for (double i = a + interval; i <= x - interval; i += interval) {
        Vector3d norm(1.0, 0.0, 0.0);
        Plane p(norm, -i, bbox);
        res.push_back(p);
    }
    interval = max(MIN_INTERVAL, abs(b - y) / ((double)k + 1));
    for (double i = b + interval; i <= y - interval; i += interval) {
        Vector3d norm(0.0, 0.0, 0.0);
        Plane p(norm, -i, bbox);
        res.push_back(p);
    }
    interval = max(MIN_INTERVAL, abs(c - z) / ((double)k + 1));
    for (double i = c + interval; i <= z - interval; i += interval) {
        Vector3d norm(0.0, 0.0, 1.0);
        Plane p(norm, -i, bbox);
        res.push_back(p);
    }
    return res;
}

vector<Plane> Mesh::get_cutting_planes(const EdgeIndices &ei, int k) const {
    auto bbox = this->bounding_box();
    vector<Plane> res;
    res.reserve(k);

    // Get triangles associated with concave edge of interest
    auto [t1_inds, t2_inds] = this->m_edge_tris.at(ei);

    auto e = this->get_edge(ei);

    //    print_triangle(t1_inds);
    //    print_triangle(t2_inds);

    auto t1 = this->get_triangle(t1_inds), t2 = this->get_triangle(t2_inds);
    // Compute normals for the concave edge to each of the triangles
    auto n1 = edge_tri_norm(e, t1), n2 = edge_tri_norm(e, t2);
    // Get Quaternions for n1 -> n2
    Quaterniond qa = Quaterniond::Identity(), qb = Quaterniond::FromTwoVectors(n1, n2);
    // slerp it k times
    for (int i = 0; i < k; i++) {
        double t = double(i) / k;
        // Alter first and last ones by a bit
        //        if (i == 0) {
        //            t = .05;
        //        } else if (i == k - 1) {
        //            t = .95;
        //        }
        auto norm = qa.slerp(t, qb) * n1;
        // Now, construct a plane from the concave edge, the norm, and the mesh's
        // bounding box
        auto plane = Plane(e, norm, bbox);
        res.push_back(plane);
    }

    return res;
}

vector<Mesh> Mesh::cut_plane(quickhull::Plane<double> &p) const {
    Plane bound_plane = Plane(p, this->bounding_box());
    return cut_plane(bound_plane);
}

std::vector<Mesh> Mesh::cut_plane(Plane &p) const {
    auto round = [](double d) { return std::round(d * 1e3) / 1e3; };
    auto [p0, p1, p2, p3] = p.bounds();

    double cutMeshVertices[] = {round(p0.x()), round(p0.y()), round(p0.z()),   // p0
                                round(p1.x()), round(p1.y()), round(p1.z()),   // p1
                                round(p2.x()), round(p2.y()), round(p2.z()),   // p2
                                round(p3.x()), round(p3.y()), round(p3.z())};  // p3

    uint32_t cutMeshFaces[] = {1, 2, 0, 1, 3, 2};

    uint32_t numCutMeshVertices = 4;
    uint32_t numCutMeshFaces = 2;

    int n_verts = m_verts.size();
    int n_faces = m_triangles.size();
    vector<double> vertices(3 * n_verts);
    vector<uint32_t> faces(3 * n_faces), face_sizes(n_faces);

    int i = 0;
    for (const Vector3d &v : m_verts) {
        for (int j = 0; j < 3; j++) {
            vertices[i++] = v[j];
        }
    }

    i = 0;
    int i2 = 0;
    for (const Vector3i &f : m_triangles) {
        for (int j = 0; j < 3; j++) {
            faces[i++] = f[j];
        }
        face_sizes[i2++] = 3;
    }

    McContext context = MC_NULL_HANDLE;
    McResult err = mcCreateContext(&context, MC_DEBUG);

    ASSERT_NO_ERROR(err);

    auto MC_DISPATCH_FILTER_CLOSED_FRAGMENTS =
        (MC_DISPATCH_REQUIRE_THROUGH_CUTS | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE |
         MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE);

    // 3. do the magic!
    // ----------------
    err = mcDispatch(
        context, (MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_CLOSED_FRAGMENTS),
        vertices.data(), faces.data(), face_sizes.data(), n_verts, n_faces, cutMeshVertices,
        cutMeshFaces,
        nullptr,  // no need to give 'faceSizes' parameter since cut-mesh is a triangle mesh
        numCutMeshVertices, numCutMeshFaces);

    if (err != MC_NO_ERROR) {
        return {};
    }
    //    ASSERT_NO_ERROR(err);

    // 4. query the number of available connected component (here, we're only
    // interested in the fragments)
    // -------------------------------------------------------------
    uint32_t numConnComps;
    std::vector<McConnectedComponent> connComps;

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, 0, NULL,
                                   &numConnComps);

    if (err != MC_NO_ERROR) {
        return {};
    }
    //    ASSERT_NO_ERROR(err);

    if (numConnComps == 0) {
        fprintf(stdout, "no connected components found\n");
        return {};
    }

    connComps.resize(numConnComps);

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT,
                                   (uint32_t)connComps.size(), connComps.data(), NULL);

    if (err != MC_NO_ERROR) {
        DEBUG_MSG("Failed to get connected components");
        return {};
    }

    //    ASSERT_NO_ERROR(err);

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

        if (err != MC_NO_ERROR) {
            return {};
        }

        uint32_t numberOfVertices = (uint32_t)(numBytes / (sizeof(float) * 3));

        std::vector<float> vertices(numberOfVertices * 3u);

        err =
            mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT,
                                        numBytes, (void *)vertices.data(), NULL);

        if (err != MC_NO_ERROR) {
            return {};
        }

        // query the faces
        // -------------------

        err = mcGetConnectedComponentData(
            context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);
        if (err != MC_NO_ERROR) {
            return {};
        }

        std::vector<uint32_t> faceIndices(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp,
                                          MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes,
                                          faceIndices.data(), NULL);
        if (err != MC_NO_ERROR) {
            return {};
        }

        std::vector<uint32_t> faceSizes(faceIndices.size() / 3, 3);
        //        printf("faces: %d\n", (int)faceSizes.size());

        ostringstream fname;
        fname << OUT_DIR << "cc" << i << ".obj";

        std::vector<float> f;
        for (uint32_t i = 0; i < (uint32_t)vertices.size(); ++i) f.push_back(vertices[i]);

        // save to mesh file (.obj)
        // ------------------------
        // TODO: remove

        if (PRINT_INTERMEDIATE_OBJ)
            writeOBJ(fname.str(), (float *)f.data(), (uint32_t)vertices.size() / 3,
                     (uint32_t *)faceIndices.data(), (uint32_t *)faceSizes.data(),
                     (uint32_t)faceSizes.size());

        vector<Vector3d> verts = vec3f_to_vec3d(float_to_vec3f(vertices));
        vector<Vector3i> faces = uint_to_vec3i(faceIndices);
        out.push_back(Mesh(verts, faces));
    }

    // 6. free connected component data
    // --------------------------------
    err = mcReleaseConnectedComponents(context, 0, NULL);

    ASSERT_NO_ERROR(err);

    // 7. destroy context
    // ------------------
    err = mcReleaseContext(context);

    ASSERT_NO_ERROR(err);

    // TODO: remake Mesh objects for two connected components

    return out;
}

pair<vector<Vector3d>, vector<int>> Mesh::sample_point_set(int resolution) const {
    // samples is based on the total surface area
    int num_samples = static_cast<int>(m_surface_area * resolution);

    // initialize sample vectors
    vector<Vector3d> point_samples(num_samples);
    vector<int> tri_ind_samples(num_samples);

    // with the number of samples, sample a random triangle based on its area and
    // sample a point on it
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
        point_samples[i] =
            random_barycentric_coord(m_verts[tri[0]], m_verts[tri[1]], m_verts[tri[2]]);
    }

    return {point_samples, tri_ind_samples};
}

vector<Edge> Mesh::shared_edges(const Vector3i &tri1, const Vector3i &tri2) {
    vector<Edge> shared;
    auto edges1 = this->get_triangle_edges(tri1), edges2 = this->get_triangle_edges(tri2);
    for (auto &&e1 : edges1) {
        for (auto &&e2 : edges2) {
            if (e1 == e2) shared.push_back(e1);
        }
    }
    return shared;
}

bool Mesh::is_convex() const {
    //    Mesh CH = this->computeCH();
    //    cout << this->volume() << endl;
    //    cout << CH.volume() << endl;
    if (std::abs(this->volume() - this->computeCH().volume()) < VOL_EPSILON) return true;
    return false;
}

bool Mesh::is_concave() const {
    return !this->is_convex();
}

vector<EdgeIndices> Mesh::get_concave_edges() const {
    vector<EdgeIndices> concave_edge_indices;
    for (const auto &[ei, tris] : this->m_edge_tris) {
        // Get the triangle points corresponding to each triangle's indices
        auto [tri1, tri2] = tris;

        Vector3i tri1i = tri1, tri2i = tri2;

        // Get the vertices that the triangles don't share
        int v0_ind = get_third_point(tri1, ei), v3_ind = get_third_point(tri2, ei);
        Vector3d v0 = m_verts[v0_ind], v3 = m_verts[v3_ind];

        // Get the edge vertices in the correct order
        Triangle t1 = this->get_triangle(tri1), t2 = this->get_triangle(tri2);

        // if either is a sliver, continue
        if (t1.area() < AREA_EPSILON || t2.area() < AREA_EPSILON) continue;

        Vector3d v1 = t1.next(v0);
        Vector3d v2 = t1.next(v1);

        // Get triangle norms
        Vector3d t1_norm = (v1 - v2).cross(v2 - v0).normalized(),
                 t2_norm = (v2 - v1).cross(v1 - v3).normalized();

        // Get direction vector dvec from v0 -> v3; if dvec and t1_norm have the same direction and
        // dvec and t2_norm have different directions, concave
        Vector3d dvec = (v3 - v0).normalized();
        if (same_dir(dvec, t1_norm) && !same_dir(dvec, t2_norm)) {
            auto dot = t1_norm.dot(t2_norm);
            if (dot < DOT_THRESHOLD) {
                //                cout << t1.area() << " " << t2.area() << endl;
                concave_edge_indices.push_back(ei);
            }
        }
    }

    return concave_edge_indices;
}

Mesh Mesh::load_from_file(const std::string &path, int scale) {
    vector<Vector3i> faces;
    vector<Vector3f> fverts;
    if (!MeshLoader::loadTriMesh(path, fverts, faces)) {
        exit(EXIT_FAILURE);
    }

    auto verts = vec3f_to_vec3d(fverts);

    // Construct and return a mesh from the vertices
    return Mesh(verts, faces, scale);
}

void Mesh::save_to_file(const string &path) const {
    ofstream outfile;
    outfile.open(path);

    // Write vertices
    for (size_t i = 0; i < m_verts.size(); i++) {
        const Vector3d &v = m_verts[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces
    for (size_t i = 0; i < m_triangles.size(); i++) {
        const Vector3i &f = m_triangles[i];
        outfile << "f " << (f[0] + 1) << " " << (f[1] + 1) << " " << (f[2] + 1) << endl;
    }

    outfile.close();
}

std::vector<Mesh> Mesh::merge(const std::vector<Mesh> &Q) {
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
    for (const auto &v : m_verts) {
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

Vector3d Mesh::random_barycentric_coord(const Vector3d &p1, const Vector3d &p2,
                                        const Vector3d &p3) {
    float w1 = rand_f();
    float w2 = rand_f();

    if (w1 + w2 >= 1.f) {
        w1 = 1.f - w1;
        w2 = 1.f - w2;
    }

    return (p1 * w1) + (p2 * w2) + (p3 * (1.f - (w1 + w2)));
}

double Mesh::angle_between_tris(const Vector3i &t1, const Vector3i &t2) {
    const vector<Vector3d> &verts = m_verts;
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
        auto &t = m_triangles[i];

        auto &v1 = m_verts[t[0]];
        auto &v2 = m_verts[t[1]];
        auto &v3 = m_verts[t[2]];

        // area of a triangle is two of its vectors crossed / 2
        auto tri_area = (v2 - v1).cross(v3 - v1).norm() / 2.f;
        total_area += tri_area;
        m_cdf[i] = total_area;
        m_tri_areas[i] = tri_area;
    }

    return total_area;
}
