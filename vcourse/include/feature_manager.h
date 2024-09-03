#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class FeaturePerFrame
{
    public:
        Feature(const Eigen::Matrix<double, 7, 1> & point, double td)
        {

        }

};

class FeatureManager
{
    public:
        explicit FeatureManager(Matix3d Rs[]);
};