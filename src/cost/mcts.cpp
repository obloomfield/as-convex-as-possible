#include "mcts.h"

MCTS::MCTS()
{

}


quickhull::Plane<double> MCTS::cuttingPlane(const Mesh& cur_mesh) {
    // TODO: implement
    quickhull::Plane<double> p;
    return p;
}


// TODO: move these helpers to the mesh class:

////std::vector<Edge> sharedEdges(const Vector3i& tri1, const Vector3i& tri2);

//std::tuple<Vector3d,Vector3d,Vector3d> trianglePoints(const Vector3i& tri,const Mesh& m) {
//    const vector<Vector3f>& verts = m.m_verts;
//    return make_tuple(verts[tri[0]],verts[tri[1]],verts[tri[2]]);
//}

//std::tuple<Edge,Edge,Edge> triangleEdges(const Vector3i& tri,const Mesh& m) {
//    const vector<Vector3f>& verts = m.m_verts;
//    // TODO: implement using richard edges
////    return make_tuple(verts[tri[0]],verts[tri[1]],verts[tri[2]]);
//}

// TODO: Waiting on Richard's Edges
// TODO: move to the mesh class.
//vector<Edge> concave_edges() {
//    vector<Edge> concave_edges;
//    for (const Vector3i& tri_1 : m_triangles) {
//        for (const Vector3i& tri_1 : m_triangles) {
//            vector<Edge> shared_edges = shared_edges(tri_1,tri_2);
//            if (shared_edges.size() == 1 && angle_between_tris(tri_1,tri_2) < M_PI) {
//                concave_edges.push_back(shared_edges[0]);
//            }
//        }
//    }
//    return concave_edges;
//}


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

//        vector<Edge> concave_edges = worst_shape->concave_edges();

        // TODO: figure out how to make cutting plane from concave edges.
    }
    quickhull::Plane<double> p;
    return p;
}
