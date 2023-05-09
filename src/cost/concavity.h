#pragma once

#include <Eigen/Dense>
#include <array>
#include <map>
#include <queue>
#include <random>
#include <vector>

#include "KDTreeVectorOfVectorsAdaptor.h"
#include "geom/mesh.h"
#include "nanoflann.h"

#define k 0.3
#define POINTS_PER_UNIT_AREA 3000

class ConcavityMetric {
 public:
    /**
     * @brief We define the concavity of a solid shape (i.e. mesh) S to be:
     * 			Concavity(S) = max(kR_v(S), H_b(S))
     * Where:
     * 	kR_v(S) = (3 * Vol(CH(S)) - Vol(S) / 4pi)^1/3
     * 		- Approximates H_i(S) = H(Sample(Int(S)), Sample(Int(CH(S))).
     * 		- Originally, the Hausdorff distance between a sample on the original
     * 			shape's boundary surface and a sample on the Convex Hull of the
     * 			boundary surface of the shape. The volume-based surrogate term
     * 			accelerates computation and avoids having to compute interior
     * 			points, which requires nearest neighbor calculations.
     * 		- In practice, R_v(S) usually overestimates H_i(S), so a constant
     * 			0 < k <= 1 is used to scale R_v(S).
     * (Here, surrogate term just refers to an alternative computation.)
     *
     * 	H_b(S) = H(Sample(Boundary(S)), Sample(Boundary(CH(S)))
     * 		- The Hausdorff distance of the boundary of the shape, and its convex hull.
     *
     * @param S the mesh of which we're computing concavity
     * @return the concavity of the mesh
     */
    static double concavity(const Mesh& S);

    /**
     * @brief Sorts a list of concave edges by shortest distance to a convex hull CH, descending
     * (i.e. the first edge in the queue has the furthest distance to the CH).
     * @param S the mesh to check distance of; not a convex hull!
     * @param concave_edges the list of concave edges to check
     * @return a deque of concave edges, sorted descending by furthest to closest distance to the
     * convex hull.
     */
    static std::deque<Edge> sort_concave_edges(const Mesh& S,
                                               const std::vector<Edge>& concave_edges);

 private:
    /**
     * @brief Computes the volume-based surrogate term
     * 			R_v(S) = (3 * Vol(CH(S)) - Vol(S) / 4pi)^1/3
     * 	to estimate the Hausdorff distance between a mesh's interior and its convex hull's interior.
     *
     * @param S the mesh of which we're computing R_v
     * @return the surrogate term R_v(S) of H_i(S)
     */
    static double R_v(const Mesh& S);

    /**
     * @brief Computes the Hausdorff distance of the boundary of a mesh, and its convex hull:
     * 			H_b(S) = H(Sample(Boundary(S)), Sample(Boundary(CH(S)))
     * @param S the mesh of which we're computing H_b(S)
     * @return the Hausdorff distance
     */
    static double H_b(const Mesh& S);

    /**
     * @brief Approximates the Hausdorff distance for two meshes A and B:
     * 		H(A, B) = max(sup d(a, B), sup d(b, A)) for all a in A, b in B
     * 	where d(x, Y) = inf d(x, y) for all y in Y, and d(x, y) is the Euclidean distance between
     * the two points.
     *
     *  Here, random sampling of the meshes is used to approximate the Hausdorff distance.
     * @param A the first mesh
     * @param B the second mesh
     * @return the Hausdorff distance
     */
    static double hausdorff_distance(const Mesh& A, const Mesh& B);

    /*
     * calculate the number of samples to be used by taking 3000 samples per total surface area
     */
    static int calc_num_samples(const Mesh& S);

    //    /*
    //     * given a len of a vector and a number of samples (should it be a percent of the vector
    //     len?),
    //     * return a vector of indices which represent a random sample from the vector
    //     */
    //    static std::vector<int> sample(int orig_len, int samples);
};
