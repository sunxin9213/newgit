#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>

#include <cmath>
#include <iostream>

namespace gtsamexamples{
struct Point2c
{
    double x;
    double y;
    Point2c(double xi, double yi):x(xi), y(yi){}
};
}


namespace gtsam{

template<>
struct traits<gtsamexamples::Point2c>
{
    typedef lie_group_tag structure_category;///这到底是个啥？

    static void Print(const gtsamexamples::Point2c & m, const std::string & str = "")
    {
        std::cout << str << "(" << m.x << ", " << m.y << ")" << std::endl;
    }///静态成员函数，它不属于类的特定实例，而是属于类本身

    static bool Equals(const gtsamexamples::Point2c & m1, const gtsamexamples::Point2c & m2, double tol = 1e-8)
    {
        if(fabs(m1.x - m2.x) < tol && fabs(m1.y - m2.y) < tol)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    enum {dimension = 2};
    static int GetDimension(const gtsamexamples::Point2c &){return dimension;}

    typedef gtsamexamples::Point2c ManifoldType;
    typedef Eigen::Matrix<double , dimension, 1> TangentVector;

    ///这函数啥功能
    static TangentVector Local(const gtsamexamples::Point2c & origin, const gtsamexamples::Point2c & other)
    {
        return Vector2(other.x - origin.x, other.y - origin.y);
    }

    static gtsamexamples::Point2c Retract(const gtsamexamples::Point2c & origin, const TangentVector & v)
    {
        return gtsamexamples::Point2c(origin.x + v(0), origin.y + v(1));
    }

    typedef multiplicative_group_tag group_flavor;

    typedef OptionalJacobian<dimension, dimension> ChartJacobian;

    static gtsamexamples::Point2c Identity()
    {
        return gtsamexamples::Point2c(0, 0);
    }

    static TangentVector Logmap(const gtsamexamples::Point2c & m, ChartJacobian Hv = boost::none)
    {
        if(Hv)
        {
            *Hv = Matrix2::Identity();
        }
        return Vector2(m.x, m.y);
    }

    static gtsamexamples::Point2c Expmap(const TangentVector & v, ChartJacobian Hv = boost::none)
    {
        if(Hv)
        {
            *Hv = Matrix2::Identity();
        }
        return gtsamexamples::Point2c(v(0), v(1));
    }

    ///向量加和
    static gtsamexamples::Point2c Compose(const gtsamexamples::Point2c & m1, const gtsamexamples::Point2c & m2,
                                            ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none)
    {
        if(H1)  *H1 = Matrix2::Identity();
        if(H2)  *H2 = Matrix2::Identity();
        return gtsamexamples::Point2c(m1.x + m2.x, m1.y + m2.y);
    }

    static gtsamexamples::Point2c Between(const gtsamexamples::Point2c & m1,
                                            const gtsamexamples::Point2c & m2, 
                                            ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none)
    {
        if(H1)  *H1 = -Matrix2::Identity();
        if(H2)  *H2 = Matrix2::Identity();
        return gtsamexamples::Point2c(m2.x - m1.x, m2.y - m1.y);
    }

    static gtsamexamples::Point2c Inverse(const gtsamexamples::Point2c & m,
                                            ChartJacobian H = boost::none)
    {
        if(H) *H = -Matrix2::Identity();
        return gtsamexamples::Point2c(-m.x, -m.y);
    }

};

}

namespace gtsamexamples{

    Point2c operator*(const Point2c & m1, const Point2c & m2)
    {
        return Point2c(m1.x + m2.x, m1.y + m2.y);
    }
} 