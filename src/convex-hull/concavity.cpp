#include "concavity.h"


float ConcavityMetric::concavity(const Mesh& S) {

    // calculate R_v(S)
        // use volume approximation, need way to get Vol(S)

    // calculate H_b(S)
        // need surface boundary of mesh -> just normal trimesh vertices?
        // calculate convex hull of surface boundary
        // sample from surface boundary and its convex hull
        // calculate hausdorff distance of samples

    // return max of the two

    return 0.f;
}


inline float distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return std::sqrt(std::pow(a.x() - b.x(), 2) +
                     std::pow(a.y() - b.y(), 2) +
                     std::pow(a.z() - b.z(), 2));
}

float ConcavityMetric::hausdorff_distance(std::vector<Eigen::Vector3f>& A, std::vector<Eigen::Vector3f>& B) {

    /*
        Brute force algorithm : (http://cgm.cs.mcgill.ca/~godfried/teaching/cg-projects/98/normand/main.html)
        1.  h = 0
        2.  for every point ai of A,
              2.1  shortest = Inf ;
              2.2  for every point bj of B
                            dij = d (ai , bj )
                            if dij < shortest then
                                      shortest = dij
              2.3  if shortest > h then
                            h = shortest
     *
     */
    float max_dist = 0.0;

    for (auto& a_i : A)
    {
        float min_dist = std::numeric_limits<float>::max();

        for (auto& b_j : B)
        {

            float d_ij = distance(a_i, b_j);

            if (d_ij < min_dist)
            {
                min_dist = d_ij;
            }
        }

        if (min_dist > max_dist)
        {
            max_dist = min_dist;
        }
    }

    // might need to repeat with B and A for symmetry?

    return max_dist;


}


// THIS IS WRONG
//// implements reservoir sampling algorithm -- IDK IF THIS WORKS LOL
//std::vector<int> ConcavityMetric::sample(int orig_len, int samples) {

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
