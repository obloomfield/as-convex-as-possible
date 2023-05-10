#include <QApplication>
#include <QScreen>
#include <QSurfaceFormat>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <string_view>

#include "cost/concavity.h"
#include "cost/mcts.h"
#include "mainwindow.h"

using namespace std::chrono;

const std::string MESH_FILE = "meshes/cactus.obj";

const std::string OUT_DIR = "out/";

int main(int argc, char *argv[]) {
    srand(static_cast<unsigned>(time(0)));

    //    for (int i = 0; i < 17; i++) {
    //        Mesh::load_from_file("out/iter13mesh" + to_string(i) + ".obj")
    //            .computeCH()
    //            .save_to_file("out/finalfrag" + to_string(i) + ".obj");
    //    }

    cout << "starting mesh load...\n";
    auto t1 = chrono::high_resolution_clock::now();

    // Load mesh from file
    Mesh m = Mesh::load_from_file(MESH_FILE);

    auto t2 = chrono::high_resolution_clock::now();
    cout << "Total time: " << chrono::duration_cast<chrono::milliseconds>(t2 - t1).count()
         << "ms\n";

    Mesh ch = m.computeCH();
    ch.save_to_file(OUT_DIR + "ch.obj");

    cout << "starting greedy search...\n";
    t1 = chrono::high_resolution_clock::now();

    auto frag_map = MCTS::greedy_search(m);

    t2 = chrono::high_resolution_clock::now();
    cout << "Total time: " << chrono::duration_cast<chrono::milliseconds>(t2 - t1).count()
         << "ms\n";

    cout << endl << endl;

    int i = 0;
    for (auto &[_, m] : frag_map) {
        // Check if still concave; if not, convert to convex hull
        if (m.is_concave()) m = m.computeCH();
        string out_file = OUT_DIR + "frag" + to_string(i++) + ".obj";
        m.save_to_file(out_file);
    }

    return 0;

    // Test speed of concavity calculations
    cout << "starting concavity...\n";
    t1 = chrono::high_resolution_clock::now();

    auto concavity = ConcavityMetric::concavity(m);

    t2 = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<chrono::milliseconds>(t2 - t1).count() << "ms\n";

    cout << "concavity: " << concavity << endl;

    // Get concave edges, then print their triangles
    auto c_edges = m.get_concave_edges();
    cout << "Number of concave edges: " << c_edges.size() << endl;

    // Get the cutting planes for the first concave edge
    auto c_planes = m.get_cutting_planes(c_edges[0], 3);
    // Save the planes to files
    for (int i = 0; i < c_planes.size(); i++) {
        string out_file = OUT_DIR + "plane" + to_string(i) + ".obj";
        c_planes[i].save_to_file(out_file);
    }

    cout << "starting cut...\n";
    t1 = chrono::high_resolution_clock::now();

    auto frags = m.cut_plane(c_planes[0]);

    t2 = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<chrono::milliseconds>(t2 - t1).count() << "ms\n";

    cout << frags.size() << endl;
    for (int i = 0; i < frags.size(); i++) {
        string out_file = OUT_DIR + "frag" + to_string(i) + ".obj";
        frags[i].save_to_file(out_file);
    }

    return 0;

    // Create a Qt application
    QApplication a(argc, argv);
    QCoreApplication::setApplicationName("ACAP");
    QCoreApplication::setOrganizationName("CS 2240");
    QCoreApplication::setApplicationVersion(QT_VERSION_STR);

    // Set OpenGL version to 4.1 and context to Core
    QSurfaceFormat fmt;
    fmt.setVersion(4, 1);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    // Create a GUI window
    MainWindow w;
    w.resize(600, 500);
    int desktopArea = QGuiApplication::primaryScreen()->size().width() *
                      QGuiApplication::primaryScreen()->size().height();
    int widgetArea = w.width() * w.height();
    if (((float)widgetArea / (float)desktopArea) < 0.75f)
        w.show();
    else
        w.showMaximized();

    return a.exec();
}
