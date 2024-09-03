#include "parameters.h"

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int NUM_OF_CAM = 1;///相机参数相关
std::vector<std::string> CMA_NAMES;
int ROW, COL;
float FREQ;
double F_THRESHOLD;
bool SHOW_TRACK;
bool STEREO_TRACK;
bool EQUALIZE;
bool FISHEYE;
bool PUB_THIS_FRAME;
float FOCAL_LENGTH;///使用extern表示改变量将在所有文件中共享，其定义通常在对应的.cc文件中，相当于它是一个全局变量，其他文件引用这个文件即可
std::string FISHEYE_MASK;

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;

int MAX_CNT;
int MIN_DIST;

int WINDOW_SIZE = 10;
inf NUM_OF_F = 1000;//？特征点数

double INIT_DEPTH;///关键帧判别参数
double MIN_PARALLAX;

int ESTIMATE_EXTRINSIC;///外参模式

double ACC_N, ACC_W;//?
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;//相机与imu之间的外参，vector的size与相机数目相同
std::vector<Eigen::Matrix3d> TIC;
Eigen::Vector3d G(0.0, 0.0, 9.8);

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVE_TIME;
int NUM_ITERATIONS;

std::string EX_CALIB_RESULT_PATH;//路径
std::string VINS_RESULT_PATH;

double TD;//？
double TR;
int ESTIMATE_TD;
bool ROLLING_SHUTTER;

void ReadParameters(std::string config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ)
    if(!fsSettings.isOpened())
    {
        std::cout << "Can not read config file!" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["focal_length"] >> FOCAL_LENGTH;
    fsSettings["solver_time"] >> SOLVER_TIME;
    fsSettings["num_of_cam"] >> NUM_OF_CAM;
    fsSettings["row"] >> ROW;
    fsSettings["col"] >> COL;
    fsSettings["freq"] >> FREQ;
    fsSettings["f_threshhold"] >> F_THRESHOLD;
    fsSettings["show_track"] >> SHOW_TRACK;
    fsSettings["stereo_track"] >> STEREO_TRACK;
    fsSettings["equalize"] >> EQUALIZE;
    fsSettings["fisheye"] >> FISHEYE;
    fsSettings["pub_this_frame"] >> PUB_THIS_FRAME;
    fsSettings["fish_mask"] >> FISHEYE_MASK;
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["max_cnt"] >> MAX_CNT;
    fsSettings["min_dist"] >> MIN_DIST;
    fsSettings["window_size"] >> WINDOW_SIZE;
    fsSettings["num_of_f"] >> NUM_OF_F;
    fsSettings["init_depth"] >> INIT_DEPTH;
    fsSettings["min_parallax"] >> MIN_PARALLAX;
    fsSettings["estimate_extrinsic"] >> ESTIMATE_EXTRINSIC;

    fsSettings["acc_n"] >> ACC_N;
    fsSettings["acc_w"] >> ACC_W;
    fsSettings["gyr_n"] >> GYR_N;
    fsSettings["gyr_w"] >> GYR_W;
    fsSettings["bias_acc_threshold"] >> BIAS_ACC_THRESHOLD;
    fsSettings["bias_gyr_threshold"] >> BIAS_GYR_THRESHOLD;
    fsSettings["num_of_f"] >> NUM_OF_F;
    fsSettings["init_depth"] >> INIT_DEPTH;
    fsSettings["min_parallax"] >> MIN_PARALLAX;
    fsSettings["num_iterations"] >> NUM_ITERATIONS;

    fsSettings["ex_calibr_result_path"] >> EX_CALIB_RESULT_PATH;
    fsSettings["vins_result_path"] >> VINS_RESULT_PATH;
    fsSettings["td"] >> TD;
    fsSettings["tr"] >> TR;
    fsSettings["estimate_td"] >> ESTIMATE_TD;
    fsSettings["rolling_shutter"] >> ROLLING_SHUTTER;




    fsSettings.release();
    
}