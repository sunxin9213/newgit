#include "common.h"
#include <math.h>

std::vector<POINT> read_source_data()
{
    std::vector<POINT> source_data;
    /*读取点云数据*/


    return source_data;
}

/*根据点云计算平面-直接法*/
MODEL_PARAMS plane_from_points(const std::vector<POINT> & data_points)
{
    //https://blog.csdn.net/qq_41188371/article/details/116237621
    //https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
    //https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
    //优化方法，使用ceres或者g2o
    //Ax = b的svd分解
    //当前采用的方法

    MODEL_PARAMS result;

    if(data_points.size() < 3)
    {
        std::cout << "Can not get plane by current points!\n";
        return result;
    }
    if(data_points.size() == 3)//避免3个点在同一条直线的情况
    {
        VEC3D v1{data_points[1].x - data_points[0].x, data_points[1].y - data_points[0].y, data_points[1].z - data_points[0].z};
        VEC3D v2{data_points[2].x - data_points[0].x, data_points[2].y - data_points[0].y, data_points[2].z - data_points[0].z};
        if(abs(get_cos_vector(v1, v2)) > 0.95)
        {
            std::cout << "Can not get plane by current points!\n";
            return result;
        }
    }

    POINT points_temp = {0, 0, 0};
    POINT point_center = {0, 0, 0};

    for(int i = 0; i < data_points.size(); i++)
    {
        points_temp.x += data_points[i].x;
        points_temp.y += data_points[i].y;
        points_temp.z += data_points[i].z;
    }

    point_center.x = points_temp.x / data_points.size();
    point_center.y = points_temp.y / data_points.size();
    point_center.z = points_temp.z / data_points.size();

    float xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
    POINT point_temp = {0, 0, 0};

    for(int i = 0; i < data_points.size(); i++)
    {
        point_temp.x = data_points[i].x - point_center.x;
        point_temp.y = data_points[i].y - point_center.y;
        point_temp.z = data_points[i].z - point_center.z;

        xx += pow(point_temp.x, 2);
        yy += pow(point_temp.y, 2);
        zz += pow(point_temp.z, 2);
        xy += point_temp.x * point_temp.y;
        xz += point_temp.x * point_temp.z;
        yz += point_temp.y * point_temp.z;
    }
    /*计算det D*/
    float det_x = yy * zz - yz * yz;
    float det_y = xx * zz - xy * xy;
    float det_z = xx * yy - xy * xy;

    float det_max = std::max(det_x, std::max(det_y, det_x));

    VEC3D plane_direction_vec = {0, 0, 0};

    if(det_max = det_x)
    {
        float b = (xz * yz - xy * zz);
        float c = (xy * yz - xz * yy);
        plane_direction_vec = {det_max, b, c};//平面的方向向量
    }
    else if(det_max = det_y)
    {
        float a = (yz * xy - xz * yy);
        float c = (xy * xz - xx * yz);
        plane_direction_vec = {a, det_max, c};//平面的方向向量

    }
    else
    {
        float a = (yz * xy - xz * yy);
        float b = (xy * xz - xx * yz);
        plane_direction_vec = {a, b, det_max};//平面的方向向量
    }

    float temp = sqrt(pow(plane_direction_vec.x, 2) + pow(plane_direction_vec.y, 2) + pow(plane_direction_vec.z, 2));//归一化法向量，方便后面计算d
    VEC3D plane_vec_normalized = {plane_direction_vec.x / temp, plane_direction_vec.y / temp, plane_direction_vec.z / temp};//归一化后的法向量

    result.a = plane_vec_normalized.x;
    result.b = plane_vec_normalized.y;
    result.c = plane_vec_normalized.z;
    result.d = -(plane_vec_normalized.x * point_center.x + plane_vec_normalized.y * point_center.y + plane_vec_normalized.z * point_center.z);
    result.status = true;

    return result;
}
/*根据点云计算平面-svd法*/
MODEL_PARAMS plane_from_points_svd(const std::vector<POINT> & data_points)
{

}
/*根据点云计算平面-优化法*/
MODEL_PARAMS plane_from_points_opt(const std::vector<POINT> & data_points)
{

}

/*由向量求夹角cos值*/
float get_cos_vector(const VEC3D & v1, const VEC3D & v2)
{
    float temp1 = pow(v1.x, 2) + pow(v1.y, 2) + pow(v1.z, 2);
    float temp2 = pow(v2.x, 2) + pow(v2.y, 2) + pow(v2.z, 2);
    float cos_alpha = dot_multity(v1, v2) / (sqrt(temp1) * sqrt(temp2));

    return cos_alpha;
}

/*点积运算*/
float dot_multity(const VEC3D & v1, const VEC3D & v2)
{
    return(v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

float pt_dis_to_plane(const MODEL_PARAMS & param, const float & px, const float & py, const float & pz)
{
    if(param.model_type == "plane")
    {
        return (param.a * px + param.b * py + param.c * pz + param.d);//平面的三个参数已经归一化了
    } 
}

std::vector<POINT> find_inliners(const std::vector<POINT> & source_points, const MODEL_PARAMS & param, const float & dis_thre)
{
    float point_dis_error;
    std::vector<POINT> inliners_temp;
    for(int i = 0; i < source_points.size(); i++)
    {
        // POINT point_temp;
        point_dis_error = pt_dis_to_plane(param, source_points[i].x, source_points[i].y, source_points[i].z);
        if(point_dis_error < dis_thre)
        {
            // point_temp.x = source_points[i].x;
            // point_temp.y = source_points[i].y;
            // point_temp.z = source_points[i].z;

            inliners_temp.emplace_back(source_points[i].x, source_points[i].y, source_points[i].z);//保存内点，最后再更新一次模型参数
            // inliners_temp.push_back(point_temp);
        }
    }

    return inliners_temp;
}