#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv)
{
    NonlinearFactorGraph graph;

    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1));///第一个顶点的方差
    graph.add(PriorFactor<Pose2>(Symbol('x', 1), Pose2(0, 0, 0), priorModel));///添加初始节点，其实也是一元边

    noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));
    ///添加odom边
    graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
    ///这里的位姿变化(边)是在上一位姿坐标系下的，不是世界坐标系下的
    graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, -M_PI_2), odomModel));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 3), Symbol('x', 4), Pose2(5, 0, -M_PI_2), odomModel));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 4), Symbol('x', 5), Pose2(5, 0, -M_PI_2), odomModel));

    ///添加回环边和方差，类似于信息矩阵
    noiseModel::Diagonal::shared_ptr loopModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 5), Symbol('x', 2), Pose2(5, 0, -M_PI_2), loopModel));

    graph.print("\nFactor Graph:\n");

    Values initials;
    initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));///添加每个顶点的初始值，是在全局坐标系下的
    initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));///节点名称为x1、x2....
    initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -M_PI_2 - 0.2));
    initials.insert(Symbol('x', 4), Pose2(10.2, -5.0, -M_PI + 0.1));
    initials.insert(Symbol('x', 5), Pose2(5.1, -5.1, M_PI_2 - 0.1));

    initials.print("\nInitials Values:\n");

    GaussNewtonParams parameters;

    parameters.setVerbosity("ERROR");

    GaussNewtonOptimizer optimizer(graph, initials, parameters);

    Values results = optimizer.optimize();

    results.print("Final Result:\n");

    Marginals marginals(graph, results);

    cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;
    cout << "x4 covariance:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
    cout << "x5 covariance:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;


    return 0;
}