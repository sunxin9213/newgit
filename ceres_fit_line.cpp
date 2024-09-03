// This is the main.cpp file for the CMake project.
#include <iostream>
#include "ceres/ceres.h"
#include <opencv2/opencv.hpp>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct point
{
    double x_;
    double y_;
};

struct abcResidual
{
    abcResidual(double x, double y):x_(x), y_(y){}
    template <typename T> bool operator()(const T* a, const T* b, const T* c, T* residual) const
    {
        residual[0] = y_ - exp(a[0] * x_ * x_ + b[0] * x_ + c[0]);
        return true;
    }

    private:
        const double x_;//观测数据
        const double y_;
};

int main(int argc, char ** argv)
{
    double aa = 0.1, bb = 0.5, cc = 2;
    double a = 0.0, b = 0.0, c = 0.0;

    int N = 1000;
    cv::RNG rng(cv::getTickCount());
    double x = 0.0, y = 0.0;

    std::vector<point> points_obs;//存储观测变量 

    for (size_t i = 0; i < N; i++)
    {
        x = rng.uniform(0.0, 3.0);//产生数据x
        y = exp(aa * x * x + bb * x + cc) + rng.gaussian(0.5);//产生带有高斯噪声的y值 
        point point_temp;
        point_temp.x_ = x;
        point_temp.y_ = y;
        points_obs.push_back(point_temp);
    }

    ceres::Problem problem;

    for(int i = 0; i < points_obs.size(); i++)
    {
        problem.AddResidualBlock(new AutoDiffCostFunction<abcResidual,1,1,1,1>(new abcResidual(points_obs[i].x_, points_obs[i].y_)), NULL, &a, &b, &c);
    }

    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << "a: " << a << ", b:" << b << ", c:" << c << std::endl;


    return 0;
}
