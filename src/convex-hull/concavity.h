#pragma once

#include <random>

#include "geom/mesh.h"


#define k 0.3
#define POINTS_PER_UNIT_AREA 3000

class ConcavityMetric {
public:

    /*
     * The concavity of a solid shape S is:
            Concavity(S) = max(𝑘R_v(S), H_i(S))

       Where:

        𝑘Rv(S) = ((3Vol(CH(S) - Vol(S)) / 4pi)^1/3
            - approximates H_i(S) = H(Sample(IntS), Sample(Int CH(S)))
            - originally, the Hausdorff distance between a sample on the original shape's boundary surface
              and a sample on the convex hull of the bonudary surface of the shape
            - volume-based surrogate term accelerates computation and avoids having to compute interior points
            - Rv(S) often overestimaets H_i(S), so use a constant k (0, 1] to estimates


        H_b(S) = H(Sample(𝜕S), Sample(𝜕CH(S)))
            - the Hausdorff distance

     */
    static float concavity(const Mesh& S);


private:

    static float R_vol_surrogate();

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
     * calculate the number of samples to be used by taking 3000 samples per total surface area
     */
    static int calc_num_samples(const Mesh& S);



//    /*
//     * given a len of a vector and a number of samples (should it be a percent of the vector len?),
//     * return a vector of indices which represent a random sample from the vector
//     */
//    static std::vector<int> sample(int orig_len, int samples);

};
