#include <vector>
#include <algorithm>

#include "common.h"

using namespace std;

geometry_type geometry = plane;//设定默认的模型为plane

int main(int argc, char *argv[])
{
    if(argc > 1)
    {
        geometry = static_cast<geometry_type>(atoi(argv[1]));//根据main设置拟合类型，默认为plane
    }
    /*设置ransac参数*/
    RANSAC_PARAMS ransac_params;
    ransac_params.inlier_pro = 0.9;
    ransac_params.model_point_k = 10;
    ransac_params.num_iteration = 10;//可以设置成动态的
    ransac_params.dis_threshold = 0.02;

    /*读取点云数据*/
    std::vector<POINT> source_data = read_source_data();

    std::vector<POINT> child_source_data;
    POINT child_point;

    bool ransacking = true;
    int iteration = 0;

    std::vector<POINT> inliners;//存储内点
    MODEL_PARAMS result;

    /*根据模型抽取随机点*/
    while(ransacking)
    {
        iteration++;
        std::random_shuffle(source_data.begin(), source_data.end());

        for(int i = 0; i < ransac_params.model_point_k; i++)
        {
            child_point.x = source_data[i].x;
            child_point.y = source_data[i].y;
            child_point.z = source_data[i].z;
            child_source_data.push_back(child_point);
        }

        switch (geometry)//考虑兼容其他模型
        {
            case 0:
                break;
            case 1:
                break;
            case 2:
                result = plane_from_points(child_source_data);
                child_source_data.clear();
                break;
            default:
                break;
        }

        inliners.clear();
        inliners = find_inliners(source_data, result, ransac_params.dis_threshold);

        if(iteration > ransac_params.num_iteration)
        {
            if(inliners.size() > source_data.size() * ransac_params.inlier_pro)
            {
                ransacking = false;
                result.status = true;//sx：已经找到一个参数满足最低要求了

                MODEL_PARAMS result_better;

                result_better = plane_from_points(inliners);//sx:重新使用内点进行求解
                std::vector<POINT> new_inliners;
                new_inliners = find_inliners(source_data, result_better, ransac_params.dis_threshold);//根据新的点数求解一次方程
            }
            else
            {
                ransacking = false;
                //result.status = false;//没有找到合理的参数
                std::cout << "Can't fit a model!\n";
            }
        }
        else
        {
            if(inliners.size() > source_data.size() * ransac_params.inlier_pro)//这是一个最低标准
            {
                ransacking = false;
                result.status = true;//sx：已经找到一个参数满足最低要求了，可能不使用太多的迭代次数，降低资源占用，这个要根据实际情况权衡

                MODEL_PARAMS result_better;

                result_better = plane_from_points(inliners);//sx:重新使用内点进行求解
                std::vector<POINT> new_inliners;
                new_inliners = find_inliners(source_data, result_better, ransac_params.dis_threshold);//根据新的点数求解一次方程
                if(new_inliners.size() > inliners.size())
                {
                    result_better.status = true;
                    result = result_better;//更新一下结果
                }
            }
            else
            {
                continue;
            }
        }
    }

    if(result.status)
    {
        std::cout << "Model type: " << result.model_type <<\ 
        " , params abcd is : " << result.a << " ," << result.b<<\
         " ," << result.c << result.d << " .\n";
    }
    else
    {
        std::cout << "Can't fit a model!\n";
    }
    
    return 0;
}