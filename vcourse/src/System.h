#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <string>

#include <cv.h>
#include <opencv2/opencv.hpp>

struct IMU_MSG
{
    double header;
    Eigen::Vector3d linear_accleration;
    Eigen::Vector3d angular_velocity;
};

typedef std::shared_ptr<IMU_MSG const> ImuConstPtr;

sturct IMG_MSG
{
    double header;
    vector<Vector3d> points;
    vector<int> ids;
    vector<float> u_point;
    vector<float> v_point;
    vector<float> velocity_x;
    vector<float> velocity_y;
}

typedef std::shared_ptr<IMG_MSG const> ImgConstPtr;

class System
{
    public:
        //System();
        explicit System(std::string config_path);
        ~System();

        void ImageDataCallback(double sStamp, cv::Mat & img);
        void ImuDataCallback(double sStamp, const Eigen::Vector3d & vGyr, const Eigen::Vector3d & vAcc);
        void ProcessBackEnd();

    private:
        Estimator estimator_;

        double current_time_ = -1;
        double first_image_time_;
        bool first_image_flag_ = true;
        bool init_pub_ = 0;
        double last_image_time_ = 0.0;
        int pub_count_ = 1;

        std::queue<ImuConstPtr> imu_buf_;
        std::queue<ImgConstPtr> img_buf_;
        std::condition_variable con_;
        int sum_if_wait_ = 0;

        std::mutex m_buf_;
        std::mutex m_state_;
        std::mutex i_buf_;
        std::mutex m_estimator_;

        double latest_time_;
        Eigen::Vector3d cur_acc_;
        Eigen::Vector3d cur_gyr_; 
        Eigen::Vector3d tmp_P_;
        Eigen::Quaterniond tmp_Q_;
        Eigen::Vector3d tmp_V_;
        Eigen::Vector3d tmp_ba_;
        Eigen::Vector3d tmp_bg_;
        bool init_feature_ = false;///记录初始化状态？
        bool init_imu_ = 1;
        double last_imu_time_;
        std::vector<Eigen::Vector3d> vPath_;
        bool bStart_backend_;
        std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();               


        std::vector<uchar> v_status_;
        std::vector<float> v_err_;
};
















#endif