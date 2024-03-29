#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

#include <sstream>

using namespace std;
using namespace Eigen;

// path to trajectory file
string trajectory_file = "../trajectory.txt";

// try to use a template function to change string to double
template <class Type>
Type str2num(const string& str) {
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> >);

int main(int argc, char **argv) {

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream fin(trajectory_file);
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        // fscanf()
        // cout << qx << " " << qy << " " << qz << " " << qw << endl;
        // double txd = str2num<double>(tx);
        // double tyd = str2num<double>(ty);
        // double tzd = str2num<double>(tz);
        // double qxd = str2num<double>(qx);
        // double qyd = str2num<double>(qy);
        // double qzd = str2num<double>(qz);
        // double qwd = str2num<double>(qw);
        Quaterniond q(qw, qx, qy, qz);
        Vector3d t(tx, ty, tz);
        // q.normalize();
        Sophus::SE3d SE3d_qt(q, t);
        poses.push_back(SE3d_qt);
    }
    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}