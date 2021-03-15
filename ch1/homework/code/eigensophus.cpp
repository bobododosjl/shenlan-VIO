#include <iostream>
#include <sophus/se3.h>
#include <sophus/so3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main( int argc, char** argv )
{
    ///rotate around y axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0)).toRotationMatrix();

    Sophus::SO3 SO3_R(R);
    Eigen::Quaterniond q(R);

    ///SO3 update
    Eigen::Vector3d update_so3(0.01, 0.02, 0.03);
    Sophus::SO3 SO3_updated = SO3_R * Sophus::SO3::exp(update_so3);
    cout << "SO3 updated" << endl << SO3_updated.matrix() << endl;

    ///quaternion update
    Eigen::Quaterniond q_update(1, update_so3(0)/2, update_so3(1)/2, update_so3(2)/2);
    Eigen::Quaterniond q_updated = (q*q_update).normalized();
    cout << "q2R" << endl << q_updated.toRotationMatrix() << endl;

    return 0;
}
