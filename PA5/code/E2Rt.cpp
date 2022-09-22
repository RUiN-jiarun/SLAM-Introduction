//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<MatrixXd> svd(E, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Vector3d sigma = svd.singularValues();
    DiagonalMatrix<double, 3> Sigma((sigma(0) + sigma(1)) / 2, (sigma(0) + sigma(1)) / 2, 0);
    Matrix3d Rz = AngleAxisd(M_PI/2, Vector3d(0,0,1)).toRotationMatrix();
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;

    t_wedge1 = U * Rz * Sigma * U.transpose();
    R1 = U * Rz.transpose() * V.transpose();
    t_wedge2 = U * Rz.transpose() * Sigma * U.transpose();
    R2 = U * Rz * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}
// R1 =   -0.365887  -0.0584576    0.928822
// -0.00287462    0.998092   0.0616848
//    0.930655  -0.0198996    0.365356
// R2 =  -0.998596  0.0516992 -0.0115267
// -0.0513961   -0.99836 -0.0252005
//  0.0128107  0.0245727  -0.999616
// t1 =  -0.581301
// -0.0231206
//   0.401938
// t2 =  0.581301
// 0.0231206
// -0.401938
// t^R =  -0.0203619   -0.400711  -0.0332407
//    0.393927   -0.035064    0.585711
// -0.00678849   -0.581543  -0.0143826