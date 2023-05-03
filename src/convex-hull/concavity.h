#pragma once

#include <random>

#include "geom/mesh.h"


double K = 100.;

class ConcavityMetric {
public:

    /*
     * The concavity of a solid shape S is:
            Concavity(S) = max(H_b(S), H_i(S))

       Where:
        H_b(S) = H(Sample(𝜕S), Sample(𝜕CH(S)))
            - the Hausdorff distance between a sample on the original shape's boundary surface
              and a sample on the convex hull of the bonudary surface of the shape

        H_i(S) = H(Sample(IntS), Sample(Int CH(S)))
            - the Hausdorff distance

     */
    static float concavity(const Mesh& S);


private:

    /*
     * Hausdorff distance for two point sets:
            H(𝐴, 𝐵) = max{sup 𝑑(𝑎, 𝐵), sup 𝑑(𝑏, 𝐴)} ; 𝑎∈𝐴, 𝑏∈𝐵

      - where 𝐴 and 𝐵 are two point sets, 𝑑(𝑥, 𝑌) = inf 𝑑(𝑥, 𝑦), 𝑦∈𝑌
        and 𝑑(𝑥, 𝑦) indicates the Euclidean distance between the two points.
     *
     */
    static float hausdorff_distance(std::vector<Eigen::Vector3f>& A, std::vector<Eigen::Vector3f>& B);


    // should we store the convex hull as a field here? or should it be passed in along with the mesh to the concavity function


    /*
     * given a len of a vector and a number of samples (should it be a percent of the vector len?),
     * return a vector of indices which represent a random sample from the vector
     */
    static std::vector<int> sample(int orig_len, int samples);

};
