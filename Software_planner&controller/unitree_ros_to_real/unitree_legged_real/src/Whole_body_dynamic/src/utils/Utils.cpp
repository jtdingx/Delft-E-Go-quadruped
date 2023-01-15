//
// Created by shuoy on 10/19/21.
//

#include "Utils.h"
// #define ;

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}

Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}


Eigen::Vector3d rotm_to_euler(Eigen::Matrix3d rotmat) {
    Eigen::Vector3d rpy;
    rpy.setZero();
    double a = 0;
    double b = 0;
    double c = 0;
    double cosC = 0;
    double sinC = 0;
    double pi = 3.141592653;
    if (abs(rotmat(2 ,0) - 1.0) <1.0e-5)   // singularity
    {
        a = 0.0;
        b = - pi * 0.5;
        c = atan2(-rotmat(0, 1), -rotmat(0, 2));
    }
    else
    {
        if (abs(rotmat(2, 0) + 1.0) < 1.0e-5)   //singularotmatity
        {
            a = 0.0; 
            b = pi / 2.0;
            c = -atan2(rotmat(0, 1), rotmat(0, 2));
        }    
        else
        {
            a = atan2(rotmat(2, 1), rotmat(2, 2));
            c = atan2(rotmat(1, 0), rotmat(0, 0));
            //     a = atan2(-rotmat(3, 2), -rotmat(3, 3)); //a另一个解
            //    c = atan2(-rotmat(2, 1), -rotmat(1, 1));  //c另一个解
            cosC = cos(c);
            sinC = sin(c);
            
            if (abs(cosC) > abs(sinC))
            {
                b = atan2(-rotmat(2, 0), rotmat(0, 0) / cosC);
            }

            else
            {
                b = atan2(-rotmat(2, 0), rotmat(1, 0) / sinC);
            }

        }
    }
    rpy<< a,
          b,
          c;

    return rpy;
}


// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

double Utils::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}

Eigen::Vector3d BezierUtils::get_foot_pos_curve(float t,
                                   Eigen::Vector3d foot_pos_start,
                                   Eigen::Vector3d foot_pos_final, 
                                   double terrain_pitch_angle = 0)
{
    Eigen::Vector3d foot_pos_target;
    // X-axis
    std::vector<double> bezierX{foot_pos_start(0),
                               foot_pos_start(0),
                                foot_pos_final(0),
                                foot_pos_final(0),
                                foot_pos_final(0)};
    foot_pos_target(0) = bezier_curve(t, bezierX);

    // Y-axis
    std::vector<double> bezierY{foot_pos_start(1),
                                foot_pos_start(1),
                                foot_pos_final(1),
                                foot_pos_final(1),
                                foot_pos_final(1)};
    foot_pos_target(1) = bezier_curve(t, bezierY);

    // Z-axis
    std::vector<double> bezierZ{foot_pos_start(2),
                                foot_pos_start(2),
                                foot_pos_final(2),
                                foot_pos_final(2),
                                foot_pos_final(2)};
    bezierZ[1] += 0;
    bezierZ[2] += 0.4 + 0.5*sin(terrain_pitch_angle);
    foot_pos_target(2) = bezier_curve(t, bezierZ);

    return foot_pos_target;
}


double BezierUtils::bezier_curve(double t, const std::vector<double> &P) {
    std::vector<double> coefficients{1, 4, 6, 4, 1};
    double y = 0;
    for (int i = 0; i <= bezier_degree; i++) {
        y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
    }
    return y;
}