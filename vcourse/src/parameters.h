#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <fstream>

const int NUM_OF_CAM = 1;///相机参数相关
extern std::vector<std::string> CMA_NAMES;
extern int ROW, COL;
extern float FREQ;
extern double F_THRESHOLD;
extern bool SHOW_TRACK;
extern bool STEREO_TRACK;
extern bool EQUALIZE;
extern bool FISHEYE;
extern bool PUB_THIS_FRAME;
extern float FOCAL_LENGTH;///使用extern表示改变量将在所有文件中共享，其定义通常在对应的.cc文件中，相当于它是一个全局变量，其他文件引用这个文件即可
extern std::string FISHEYE_MASK;

extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;

extern int MAX_CNT;
extern int MIN_DIST;

extern int WINDOW_SIZE;
extern inf NUM_OF_F;//？特征点数

extern double INIT_DEPTH;///关键帧判别参数
extern double MIN_PARALLAX;

extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;//?
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;//相机与imu之间的外参，vector的size与相机数目相同
extern std::vector<Eigen::Matrix3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVE_TIME;
extern int NUM_ITERATIONS;

extern std::string EX_CALIB_RESULT_PATH;//路径
extern std::string VINS_RESULT_PATH;

extern double TD;
extern double TR;//roll shutter相机参数
extern int ESTIMATE_TD;
extern bool ROLLING_SHUTTER;

void ReadParameters(std::string config_file);

enum PARAMETERS_SIZE
{
    POSE_SIZE = 7,
    SPEEDBIAS_SIZE = 9,
    FEATURN_SIZE = 1
};

enum StateOrder///状态量15维
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
}

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
























#endif