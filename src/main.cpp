#include <QApplication>
#include <QScreen>
#include <QSurfaceFormat>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <string_view>

#include "mainwindow.h"

using namespace std::chrono;

const std::string MESH_FILE = "meshes/teapot.obj";

const std::string OUT_DIR = "out/";

int main(int argc, char *argv[]) {
    srand(static_cast<unsigned>(time(0)));

    // Load mesh from file
    Mesh m = Mesh::load_from_file(MESH_FILE);
    Mesh ch = m.computeCH();
    ch.save_to_file(OUT_DIR + "ch.obj");

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
    auto t1 = chrono::high_resolution_clock::now();
    auto frags = m.cut_plane(c_planes[0]);
    auto t2 = chrono::high_resolution_clock::now();
    /* Getting number of milliseconds as an integer. */
    auto ms_int = chrono::duration_cast<chrono::milliseconds>(t2 - t1);
    cout << ms_int.count() << "ms\n";

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
