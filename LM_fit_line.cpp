// This is the main.cpp file for the CMake project.
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Cholesky>

using namespace std;

class LM
{
    public:
        LM()
        {
            a_ = 0.0;
            b_ = 0.0;
            c_ = 0.0;
            max_iter_ = 100;
            epsilon_1_ = 1e-10;
            epsilon_2_ = 1e-10;
            mu = 1e-3;
        }

        void add_obeservation(const float & x, const float & y)//添加观测数据
        {
            obs_.push_back(Eigen::Vector2f(x, y));
        }

        void com_H_g()
        {
            H_ = J_.transpose() * J_;//H矩阵
            g_ = -J_.transpose() * fx_;//H * delta_x = b，这个g_就是b矩阵
        }

        void calcJ_fx()
        {
            J_.resize(obs_.size(), 3);//N个观测，3个参数
            fx_.resize(obs_.size(), 1);
            float ex_temp = 0;
            float x_temp = 0;
            float y_temp = 0;

            for(size_t i = 0; i < obs_.size(); i++)
            {
                x_temp = obs_[i][0];
                y_temp = obs_[i][1];
                ex_temp = -exp(a_ * x_temp * x_temp + b_ * x_temp + c_);

                J_(i, 0) = ex_temp * x_temp * x_temp;
                J_(i, 1) = ex_temp * x_temp;
                J_(i, 2) = ex_temp;
                fx_(i, 0) = y_temp + ex_temp;
            }

            // fx_.resize(obs_.size(), 1);
            // fx_ = com_error(a_, b_, c_);
        }

        Eigen::MatrixXf com_error(const float & a_in, const float & b_in, const float & c_in)//计算y与x的残差
        {
            float x_temp = 0;
            float y_temp = 0;
            Eigen::MatrixXf fx_temp;
            fx_temp.resize(obs_.size(), 1);//N个观测，1个参数作为误差

            for(size_t i = 0; i < obs_.size(); i++)
            {
                const Eigen::Vector2f & ob = obs_.at(i); 
                // x_temp = obs_[i][0];
                // y_temp = obs_[i][1];
                x_temp = ob(0);
                y_temp = ob(1);
                //ex_temp = -exp(a_in * x_temp * x_temp + b_in * x_temp + c_in);

                fx_temp(i, 0) = y_temp - exp(a_in * x_temp * x_temp + b_in * x_temp + c_in);//计算残差
            }

            return fx_temp;
        }

        float F(float a, float b, float c)
        {
            Eigen::MatrixXf fx;
            fx.resize(obs_.size(), 1);
        
            for ( size_t i = 0; i < obs_.size(); i ++)
            {
                const Eigen::Vector2f& ob = obs_.at(i);
                const float& x = ob(0);
                const float& y = ob(1);
                fx(i, 0) = y - exp( a *x*x + b*x +c);//sx:x,y都是观测值
            }
            Eigen::MatrixXf F = 0.5 * fx.transpose() * fx;
            return F(0,0);
        }

        float L0_L( Eigen::Vector3f& h)
        {
            Eigen::MatrixXf L = -h.transpose() * J_.transpose() * fx_ - 0.5 * h.transpose() * J_.transpose() * J_ * h;
            return L(0,0);
        }

        void solve()
        {
            int iterations = 0;
            float nu = 2.0;

            calcJ_fx();

            com_H_g();

            found = (g_.lpNorm<Eigen::Infinity>() < epsilon_1_);
            
            // if(g_.lpNorm<Eigen::Infinity>() < epsilon_1_)
            // {
            //     found = true;
            // }

            std::vector<double> A;
            A.push_back( H_(0, 0) );
            A.push_back( H_(1, 1) );
            A.push_back( H_(2, 2) );
            auto max_p = std::max_element(A.begin(), A.end());
            mu = *max_p;
            
            while(!found && iterations < max_iter_)
            {
                iterations++;
                
                Eigen::Matrix3f G;
                G = H_ + mu * Eigen::Matrix3f::Identity();
                Eigen::Vector3f h = G.ldlt().solve(g_);

                if(h.norm()<= epsilon_2_ * (sqrt(a_ * a_ + b_ * b_ + c_ * c_) + epsilon_2_))
                {
                    found = true;
                    //cout << "no ok" << endl;
                }
                else
                {
                    n_a_ = a_ + h(0);
                    n_b_ = b_ + h(1);
                    n_c_ = c_ + h(2);
                    //cout << "ok" << endl;

                    // Eigen::MatrixXf err_matrix_0 = com_error(a_, b_, c_);
                    // Eigen::MatrixXf err_matrix_1 = com_error(n_a_, n_b_, n_c_);
                    // Eigen::MatrixXf fx_real = 0.5 * err_matrix_0.transpose() * err_matrix_0 - 0.5 * err_matrix_1.transpose() * err_matrix_1;//为F(x)值的变化量
                    // Eigen::MatrixXf fx_tarlor = -H_.transpose() * J_.transpose() * fx_ - 0.5 * H_.transpose() * J_.transpose() * J_ * h; 
                    // float rho = fx_real(0, 0) / fx_tarlor(0, 0);
                    //分子是根据观测数据得到的误差平方的变化量，分母是根据泰勒展开得到的误差平方的变化量，描述泰勒展开模型准确度，当使得这个误差变得足够小时，说明这个h是极值点
                    float rho =( F(a_, b_, c_) - F(n_a_, n_b_, n_c_) )  / L0_L(h);
                    if(rho > 0)//只有方向相反的情况下才是负数
                    {
                        // if(g_.lpNorm<Eigen::Infinity>() < epsilon_1_)//如果误差合格了
                        // {
                        //     found = true;
                        // }

                        a_ = n_a_;
                        b_ = n_b_;
                        c_ = n_c_;
                        calcJ_fx();
                        //缺少一步残差的计算
                        com_H_g();

                        found = (g_.lpNorm<Eigen::Infinity>() < epsilon_1_);//这里的问题？
                        
                        mu = mu * std::max<double>(0.33, 1 - std::pow(2*rho -1, 3));
                        //这里的mu会根据rho自适应调整。
                        nu = 2.0;
                    }
                    else
                    {//步子打算迈大一点
                        mu = mu * nu;
                        nu = 2 * nu;
                    }
                }
            }

            if(found)
            {
                std::cout << iterations << endl;
                std::cout << "Converged!\n";
                std::cout << "a: " << a_ << " ,b: " << b_ << ", c: " << c_ << ".\n";
            }
            else
            {
                std::cout << "NoConverged!\n";
            }
        }

    public:
        float a_, b_, c_;
        float n_a_, n_b_, n_c_;
        int max_iter_;
        float epsilon_1_, epsilon_2_;//epsilon_1_是残差的阈值，小于则认为找到了解,epsilon_2_控制参数的精度
        float mu = 1e-3;
        bool found = false;

        Eigen::MatrixXf fx_;//残差
        Eigen::MatrixXf J_;
        Eigen::MatrixXf H_;
        Eigen::MatrixXf g_;

        std::vector<Eigen::Vector2f> obs_;
};


int main(int argc, char ** argv)
{

    float aa = 0.1, bb = 0.5, cc = 2;//想要求的参数
    float a = 0.0, b = 0.0, c = 0.0;//初始参数的估计值
    LM lm;

    int N = 100;
    cv::RNG rng(cv::getTickCount());
    float x = 0.0, y = 0.0;

    for (size_t i = 0; i < N; i++)
    {
        x = rng.uniform(0.0, 2.0);//产生数据x
        y = exp(aa * x * x + bb * x + cc) + rng.gaussian(0.5);//产生带有高斯噪声的y值 

        lm.add_obeservation(x, y);
    }
    lm.solve();
    
    return 0;
}