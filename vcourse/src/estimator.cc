#include "estimator.h"

Estimator::Estimator()
{

}

Estimator::~Estimator()
{

}

void Estimator::BackendOptimization()
{

}

void Estimator::ClearState()
{

}

void Estimator::Double2Vector()
{
    
}

void Estimator::FailureDetect()
{

}

void Estimator::InitialStucture()
{

}

void Estimator::MargOldFrame()
{

}

void Estimator::MargNewFrame()
{

}

void Estimator::Optimization()
{

}

 void Estimator::ProcessIMU(const double & t, const Vector3d & linear_acc, const Vector3d & angular_vec)
 {

 }

 void Estimator::ProcessIMG(const std::map<int , std::vector<std::pair<int , Eigen::Matrix<double, 7, 1>>>> & image, const double & stamp)
 {

 }

 void Estimator::ProblemSolve()
 {

 }

 void Estimator::RelativePose(const Matrix3d & relative_R, const Vector3d & relative_T, const int & l_frame)
 {

 }

 void Estimator::SetParameters()
 {

 }

 void Estimator::SetReloFrame(const double & frame_stamp, const int & frame_index, const std::vector<Vector3d> & match_points,const Vector3d & relo_t, const Matrix3d & relo_r)
 {

 }

 void Estimator::SlideWindow()
 {

 }

void Estimator::SolveOdometry()
 {
    
 }

void Estimator::SlideNewWindow()
{

}

void Estimator::SlideOldWindow()
{

}

 void Estimator::VisualInitialAlign()
 {

 }

 void Estimator::Vector2Double()
 {

 }

