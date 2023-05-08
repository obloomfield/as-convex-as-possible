#include "mcts.h"

MCTS::MCTS()
{

}

quickhull::Plane<double> MCTS::cuttingPlane(const Mesh& cur_mesh) {
    // TODO: implement
    quickhull::Plane<double> p;
    return p;
}

int MAX_NUM_PIECES = 5; // TODO: move to some parameters header / class
quickhull::Plane<double> MCTS::cuttingPlaneGreedy(const Mesh& cur_mesh) {

    vector<Mesh> Q = {cur_mesh};
    while (Q.size() > 0 and Q.size() < MAX_NUM_PIECES) {
        // TODO: find the piece inside the Q that has the worst concavity score.
        //         - ()
        Mesh* worst_shape = nullptr;
        for (const Mesh& piece : Q) {
            // if (...) {
            //  worst_shape = piece;
            // }
        }
        assert(worst_shape != nullptr);
        // TODO: pop worst_shape off Q.
        vector<Edge> concave_edges = worst_shape->concave_edges();

        // TODO: figure out how to make cutting plane from concave edges.
    }
    quickhull::Plane<double> p;
    return p;
}
