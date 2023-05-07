#include "concavity.h"

using namespace nanoflann;
using namespace std;
using namespace Eigen;

constexpr double DMAX = numeric_limits<double>::max();
constexpr double HB_EPSILON = 1e-14;

double ConcavityMetric::concavity(const Mesh& S) {
    // calculate R_v(S)
    auto rv = R_v(S);

    // calculate H_b(S)
    auto hb = H_b(S);

    // Return Max(R_v(S), H_b(S))
    return max(rv, hb);
}

double ConcavityMetric::R_v(const Mesh& S) {
    // Get the convex hull of the mesh
    auto CH_S = S.computeCH();

    // Compute the volumes of both the mesh and its convex hull
    auto v1 = S.volume(), v2 = CH_S.volume();

    // Compute R_v
    double d = k * pow(3 * abs(v2 - v1) / (4 * M_PI), 1. / 3);
    return d;
}

double ConcavityMetric::H_b(const Mesh& S) {
    // Get the boundary of the mesh, and its CH
    auto CH_S_b = S.computeCH();

    // Compute Hausdorff distance between S_b and CH(S_b)
    double d = hausdorff_distance(S, CH_S_b);
    return d;
}

double ConcavityMetric::hausdorff_distance(const Mesh& A, const Mesh& B) {
    /*
     * We compute the Hausdorff distance as follows:
     * 	- Initialize h := 0
     * 	- For every point a in A:
     * 		- Initialize shortest := 0
     * 		- For every point b in B:
     * 			- Compute d = d(a, b)
     * 			- if d < shortest:
     * 				- Set shortest := d (can break if d sufficiently low, e.g. <=1e-14)
     * 		- If shortest > h:
     * 			- Set h := shortest
     * 	- Repeat the same process for every point b in B.
     *  - Return h
     *
     *  To speed up computation of shortest distances between a and B, we can use a KDTree here.
     */

    // KDTree in 3D; 10 is a reasonable default for max KD tree depth
    constexpr int DIM = 3, MAX_LEAF = 10;
    typedef KDTreeVectorOfVectorsAdaptor<vector<Vector3d>, double, DIM> VecKDTree;

    // Sample points from A and B, and get their corresponding triangles TODO: actually implement
    auto [A_samples, A_sample_tris] = A.sample_point_set();
    auto [B_samples, B_sample_tris] = B.sample_point_set();

    // Construct KDTrees for A and B
    VecKDTree A_kdt(DIM, A_samples, MAX_LEAF), B_kdt(DIM, B_samples, MAX_LEAF);

    // Compute Hausdoff distance!
    double h = 0.;

    // Helper function to compute d(a, B) or d(b, A)
    // - M the mesh that the sample pt came from
    // - kdt the KDTree of the *other* mesh (so if M = A, kdt = B_KDTree)
    auto compute_d = [](const Mesh& M, const Vector3d& pt, VecKDTree& kdt) {
        // Do KNN search; here we limit to 10
        constexpr size_t n_results = 10;
        // Map out results to their corresponding indices in the sample vector
        vector<size_t> ret_indices(n_results);
        // It seems that it populates the vector with the squared distance
        vector<double> out_dists_sqrd(n_results);

        // Store result set
        KNNResultSet<double> result_set(n_results);
        result_set.init(&ret_indices[0], &out_dists_sqrd[0]);

        // Search B for nearest neighbors to a!
        array<double, 3> query_vert{pt[0], pt[1], pt[2]};
        kdt.index->findNeighbors(result_set, &query_vert[0]);

        double shortest = DMAX;
        for (size_t i = 0; i < n_results; i++) {
            // Get the corresponding triangle of the result sampled vertex
            auto tri = M.m_triangles[ret_indices[i]];
            array<Vector3d, 3> tri_pts = {M.m_verts[tri[0]], M.m_verts[tri[1]], M.m_verts[tri[2]]};

            // Compute distance from a to the triangle, and see if smaller than current min
            // TODO: need to implement
            double dist = dist_pt2tri(pt, tri_pts);
            if (dist < shortest) {
                shortest = dist;
                // If basically on surface, break
                if (shortest < HB_EPSILON) break;
            }
        }

        // If shortest distance to triangle is too large, just get the shortest dist from the search
        // TODO: why do they do this
        if (shortest > 10.) shortest = sqrt(out_dists_sqrd[0]);

        return shortest;
    };

    // Compute sup d(a, B)
    for (auto&& a : A_samples) {
        // Compute d(a, B)
        auto shortest = compute_d(A, a, B_kdt);
        // Update Hausdorff distance, if applicable
        if (shortest > h) h = shortest;
    }

    // Compute sup d(b, A)
    for (auto&& b : B_samples) {
        auto shortest = compute_d(B, b, A_kdt);
        // Update Hausdorff distance, if applicable
        if (shortest > h) h = shortest;
    }

    return h;
}

// THIS IS WRONG
//// implements reservoir sampling algorithm -- IDK IF THIS WORKS LOL
// std::vector<int> ConcavityMetric::sample(int orig_len, int samples) {

//    std::vector<int> reservoir(samples);
//    int i, j;

//    // TODO: move these declaration to be declared once for efficiency
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<double> dist(0.0, 1.0);

//    // Fill the reservoir array with the first k elements from the stream.
//    for (i = 0; i < samples; i++) {
//        reservoir[i] = i;
//    }

//    // Iterate over the remaining elements and decide whether to include them in the reservoir.
//    for (i = samples; i < orig_len; i++) {
//        j = floor(dist(gen) * i);
//        if (j < samples) {
//            reservoir[j] = i;
//        }
//    }

//    return reservoir;
//}
