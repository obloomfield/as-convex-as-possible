#include "shapes.h"

#include "mesh.h"

using namespace Eigen;

float BOUNDS_PADDING = 1.f;

Plane::Plane(Edge e, const Vector3d &norm, array<double, 6> bbox) {
    // Get the minimum and maximum coordinates of the bounding box
    auto [a, b, c, x, y, z] = bbox;
    Vector3d minCoords(a, b, c);
    Vector3d maxCoords(x, y, z);

    // Get diagonal distance between bounding box
    auto dist_diag = dist(minCoords, maxCoords);

    // Get direction vector
    auto n = (e.b_ - e.a_).normalized();

    // Scale edge endpoints by that distance, in the appropriate direction
    e.a_ -= dist_diag * n;
    e.b_ += dist_diag * n;

    // Take those endpoints, and add/subtract the normal in which we wish to create the plane
    n = norm.normalized();
    p0 = e[1] + dist_diag * n;
    p1 = e[1] - dist_diag * n;
    p2 = e[0] + dist_diag * n;
    p3 = e[0] - dist_diag * n;

    // ensure that all points make a plane
    //    double EPSILON = 1e-9;
    //    Eigen::Matrix<double, 3, 3> mat;
    //    mat << (p1 - p0), (p2 - p0), (p3 - p0);
    //    assert(mat.determinant() < EPSILON);
}

// TODO: fix the spacing, fix the missing axis
Plane::Plane(const Eigen::Vector3d &norm, double d, std::array<double, 6> bbox) {
    auto [a, b, c, x, y, z] = bbox;
    Vector3d minCoords(a, b, c);
    Vector3d maxCoords(x, y, z);

    // Get diagonal distance between bounding box
    double dist_diag = dist(minCoords, maxCoords);

    // compute the center of the bounding box
    Eigen::Vector3d center = 0.5 * (minCoords + maxCoords);

    // Compute a unit vector in the direction of the normal vector
    Eigen::Vector3d normal = norm.normalized();

    // Compute a vector perpendicular to the normal vector
    Eigen::Vector3d v1 = normal.unitOrthogonal();

    // Compute another vector perpendicular to both the normal vector and v1
    Eigen::Vector3d v2 = normal.cross(v1).normalized();

    // Compute the four points that define the plane
    p0 = center + v1 * dist_diag / 2 + v2 * dist_diag / 2;
    p1 = center - v1 * dist_diag / 2 + v2 * dist_diag / 2;
    p2 = center + v1 * dist_diag / 2 - v2 * dist_diag / 2;
    p3 = center - v1 * dist_diag / 2 - v2 * dist_diag / 2;

    // Translate the plane along its normal vector by the given distance
    Eigen::Vector3d translation = d * normal;
    p0 += translation;
    p1 += translation;
    p2 += translation;
    p3 += translation;
}

Plane::Plane(const quickhull::Plane<double> &p, std::array<double, 6> bbox) {
    // Convert plane types
    Vector3d planeNormal(p.m_N.x, p.m_N.y, p.m_N.z);
    Vector3d planePoint = -p.m_D * planeNormal;

    // Get the minimum and maximum coordinates of the bounding box
    auto [a, b, c, x, y, z] = bbox;
    Vector3d minCoords(a, b, c);
    Vector3d maxCoords(x, y, z);

    // Define the four corner points of the bounding region plane
    p0 = minCoords - BOUNDS_PADDING * Vector3d::Ones();
    p1 = minCoords - BOUNDS_PADDING * Vector3d::UnitX() +
         (maxCoords.x() - minCoords.x() + 2 * BOUNDS_PADDING) * Vector3d::UnitX();
    p2 = minCoords - BOUNDS_PADDING * Vector3d::UnitY() +
         (maxCoords.y() - minCoords.y() + 2 * BOUNDS_PADDING) * Vector3d::UnitY();
    p3 = planePoint + (p0 - planePoint).dot(planeNormal) / planeNormal.dot(p0 - p3) * (p3 - p0);
}

Plane Plane::load_from_file(const std::string &path) {
    // Just read the first four vertices lol
    ifstream ifs(path);
    string line;
    vector<Vector3d> verts;
    while (std::getline(ifs, line)) {
        if (line[0] != 'v') break;
        istringstream iss(line.substr(2));
        double v0, v1, v2;
        iss >> v0 >> v1 >> v2;
        verts.emplace_back(v0, v1, v2);
    }
    // Construct and return a mesh from the vertices
    return Plane(verts[0], verts[1], verts[2], verts[3]);
}

void Plane::save_to_file(const std::string &path) {
    ofstream outfile;
    outfile.open(path);

    // Write the four vertices
    for (auto &&v : {p0, p1, p2, p3}) {
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces; hardcode to trimesh the plane: (1, 2, 0) and (1, 3, 2); note .obj files are
    // 1-indexed
    for (auto &&f : {array{1, 2, 0}, {1, 3, 2}}) {
        outfile << "f";
        for (auto &&fi : f) outfile << " " << (fi + 1);
        outfile << endl;
    }

    outfile.close();
}
