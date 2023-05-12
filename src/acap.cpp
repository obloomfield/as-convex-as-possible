#include "acap.h"

#include <array>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#include "cost/concavity.h"
#include "cost/mcts.h"
#include "graphics/meshloader.h"

using namespace std;
using namespace Eigen;

#define MESH_PATH "meshes/bunny.obj"
#define EPSILON 0.05

#define TESTING_INTERMED

// Here are some helpful controls for the application
//
// - You start in first-person camera mode
//   - WASD to move, left-click and drag to rotate
//   - R and F to move vertically up and down
//
// - C to change to orbit camera mode
//
// - Right-click (and, optionally, drag) to anchor/unanchor points
//   - Left-click an anchored point to move it around
//
// - Minus and equal keys (click repeatedly) to change the size of the vertices
void ACAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax) {
//    Plane p(Vector3d(-1.2, 1.6, 1.5), Vector3d(1.4, -1.3, 1.5), Vector3d(-1.2, 1.6, -1.005929),
//            Vector3d(1.4, -1.3, -1.005929));
//    vector<Mesh> fragments = mesh.cut_plane(p);

    //    assert(fragments.size() == 2);

    return;

////    vector<Mesh> decomp = ACD(mesh);

//    // Students, please don't touch this code: get min and max for viewport stuff
//    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
//    int i = 0;
//    for (unsigned long i = 0; i < vertices.size(); ++i) {
//        all_vertices.row(i) = vertices[i];
//    }
//    coeffMin = all_vertices.colwise().minCoeff();
//    coeffMax = all_vertices.colwise().maxCoeff();
}

// ACD using MCTS
void ACAP::ACD(const std::string& mesh_path, const std::string& out_path) {

    std::cout << "beginning MCTS ACD..." << std::endl;

    // Load mesh from file
    Mesh m = Mesh::load_from_file(mesh_path);

    // concavity queue
    std::map<double, Mesh> Q;
    // decomposition results
    std::vector<Mesh> D;

    // add mesh to queue
    double conc = ConcavityMetric::concavity(m);
    Q[conc] = m;

    int count = 0;

    while (Q.size() > 0) {

        // dequeue
        auto it = Q.rbegin();

        // if concavity is below threshold
        if (it->first < EPSILON) {
             std::cout << "fragment is below threshold" << std::endl;
            // add to decomposition
            D.push_back(it->second);
            // erase it from Q
            Q.erase(it->first);
        } else {
            std::cout << "cutting component..." << std::endl;

            Mesh mesh = Q[it->first];
            auto [c_l, c_r] = MCTS::MCTS_search(mesh);

            // the previous piece could not be decomposed.
            if (!c_l || !c_r) {
                // add to final decompositions
                D.push_back(mesh);
                // erase the original
                Q.erase(it->first);
                continue;
            }

#ifdef TESTING_INTERMED
            std::string out1 = "out_mcts/mcts" + to_string(count) + ".obj";
            ++count;
            std::string out2 = "out_mcts/mcts" + to_string(count) + ".obj";
            ++count;

            c_l->save_to_file(out1);
            c_r->save_to_file(out2);
#endif
            double c_l_score = ConcavityMetric::concavity(*c_l);
            double c_r_score = ConcavityMetric::concavity(*c_l);
            // erase pre-cut
            Q.erase(it->first);
            // add cuts
            Q[c_l_score] = *c_l;
            Q[c_r_score] = *c_r;

        }
    }

    // write each resulting fragment to file
    for (int i = 0; i < D.size(); ++i) {
        std::string out_file = out_path + "frag" + to_string(i) + ".obj";
        m.save_to_file(out_file);
    }

}
