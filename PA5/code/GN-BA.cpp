//
// Created by xiang on 12/21/17.
//

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream f3d(p3d_file);
    ifstream f2d(p2d_file);
    while (!f3d.eof() && !f2d.eof()) {
        double u, v, x, y, z;
        f3d >> x >> y >> z;
        p3d.push_back(Vector3d(x, y, z));
        f2d >> u >> v;
        p2d.push_back(Vector2d(u, v));
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector4d p4d;
            p4d << p3d[i], 1;
            Vector4d p_t = T_esti * p4d;
            Vector3d p_k = K * p_t.head(3);
            Vector2d e = p2d[i] - Vector2d(p_k[0] / p_k[2], p_k[1] / p_k[2]);
            cost += e.squaredNorm()/2;
	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J(0,0) = fx / p_t[2];
            J(0,1) = 0;
            J(0,2) = -fx * p_t[0] / p_t[2] / p_t[2];
            J(0,3) = -fx * p_t[0] * p_t[1] / p_t[2] / p_t[2];
            J(0,4) = fx + fx * p_t[0] * p_t[0] / p_t[2] / p_t[2];
            J(0,5) = -fx * p_t[1] / p_t[2];
            J(1,0) = 0;
            J(1,1) = fy / p_t[2];
            J(1,2) = -fx * p_t[1] / p_t[2] / p_t[2];
            J(1,3) = -fy - fy * p_t[1] * p_t[1] / p_t[2] / p_t[2];
            J(1,4) = fy * p_t[0] * p_t[1] / p_t[2] / p_t[2];
            J(1,5) = fy * p_t[0] / p_t[2];
            J = -J;
	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3d::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}

// points: 76
// iteration 0 cost=622769.1141257
// iteration 1 cost=12206.604990492
// iteration 2 cost=12150.675967157
// iteration 3 cost=12150.675326905
// iteration 4 cost=12150.675326904
// iteration 5 cost=12150.675326904
// iteration 6 cost=12150.675326904
// iteration 7 cost=12150.675326904
// cost: 150.675326904, last cost: 150.675326904
// estimated pose: 
//    0.997866186832  -0.0516724446462   0.0399128004625   -0.127226618385
//   0.0505959254896    0.998339770707   0.0275273418463 -0.00750675455728
//  -0.0412689411098  -0.0254491785635    0.998823915317   0.0613861980543
//                 0                 0                 0                 1