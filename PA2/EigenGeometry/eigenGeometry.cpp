#include <bits/stdc++.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    q1.normalize();
    q2.normalize();
    Vector3d t1(0.7, 1.1, 0.2);
    Vector3d t2(-0.1, 0.4, 0.8);
    Vector3d p1(0.5, -0.1, 0.2);

    // Vector3d p2 = q2 * (t1 + q1.conjugate() * p1 - t2);

    Isometry3d T1(q1), T2(q2);  // build translation matrix according to quaternion
    T1.pretranslate(t1);        // assign to the translate part
    T2.pretranslate(t2);
    
    Vector3d p2 = T2 * T1.inverse() * p1;

    cout << p2.transpose() << endl;
    return 0;
}