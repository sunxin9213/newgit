#include "System.h"

System::System(std::string config_path): bStart_backend_(true)
{   
    std::string config_path_ = config_path + "config.yaml";
    


}

System::~System()
{
    bStart_backend_ = false;
}

void System::ImageDataCallback(double stamp, cv::mat & img)
{

}

void System::ImuDataCallback(double stamp, const Eigen::Vector3d & vGyr, const Eigen::Vector3d & vAcc)
{

}

std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements()
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurement;

    while (true)
    {
        if(imu_buf_.empty() || img_buf_.empty())
        {
            return measurement;
        }

        if(!(imu_buf_.back()->header > img_buf_.front()->header))
        {
            std::cout << "wait for imu" << std::endl;
            return measurement;
        }

        if(!(imu_buf_.front()->header < img_buf_.front()->header))
        {
            std::cout << "throw image" << std::endl;
            img_buf_.pop();
            continue;
        }

        ImgConstPtr img_msg = img_buf_.front();
        img_buf_.pop();

        std::vector<ImuConstPtr> imus;
        while(imu_buf_.front()->header < img_msg->header)
        {
            imus.emplace_back(imu_buf_.front());
            imu_buf_.pop();
        }

        imus.emplace_back(imu_buf_.front());/// . . . | .  , .表示imu，|表示图像数据， 这里添加图像后一帧的imu数据

        if(imu_buf_.empty())
        {
            std::cout << "end , no imu data" << std::endl;
        }

        measurement.emplace_back(imus, img_msg);

    }
    
    return measurement;
}

void System::ProcessBackEnd()
{
    std::cout << "1 processbackend start" << std::endl;

    while(bStart_backend_)
    {
        std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;

        unique_lock<mutex> lk(m_buf);

        con.wait(lk, [&]
            { 
                return (measurements = getMeasurements()).size() != 0;
            }
        )

        if(measurements.siee() > 1)
        {
            std::cout << "something not right!" << std::end;
        }

        lk.unlock();

        m_estimator_.lock();///锁住estimator要用的数据
        for(auto & measurement : measurements)
        {
            auto img_msg = measurements.second;
            double img_t = img_msg->header;
            double dx = 0, dy = 0, double z = 0, wx = 0, wy = 0, wz = 0;

            for(auto & imu_msg : measurements.second)
            {
                double t = imu_msg->header;
                if(t <= img_t)
                {
                    if(current_time_ < 0)
                    {
                        current_time_ = t;
                    }

                    double dt = t - current_time_;///以第一帧imu数据为开始
                    current_time_ = t;
                    dx = imu_msg->linear_accleration.x();
                    dy = imu_msg->linear_accleration.y();
                    dz = imu_msg->linear_accleration.z();
                    wx = imu_msg->angular_velocity.x();
                    wy = imu_msg->angular_velocity.y();
                    wz = imu_msg->angular_velocity.z();
                    estimator.processImu(dt, Vector3d(dx, dy, dz), Vector3d(wx, wy, wz));
                }
                else
                {
                    double dt_1 = img_t - current_time_;
                    double dt_2 = t - img_t;
                    current_time_ = img_t;

                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);

                    dx = w1 * dx + w2 * img_msg->linear_acceleration.x();
                    dy = w1 * dy + w2 * img_msg->linear_acceleration.y();
                    dz = w1 * dz + w2 * img_msg->linear_acceleration.z();
                    wx = w1 * wx + w2 * img_msg->angular_velocity.x();
                    wy = w1 * wy + w2 * img_msg->angular_velocity.y();
                    wz = w1 * wz + w2 * img_msg->angular_velocity.z();
                    estimator.processImu(dt, Vector3d(dx, dy, dz), Vector3d(wx, wy, wz));
                }    
            }

            std::map<int , std::vector<std::pair<int , Eigen::Matrix<double, 7, 1>>>> image;
            for(int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->ids[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;

                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();///这里z都是1，就是归一化的3D坐标

                double u_2d = img_msg->u_point[i]; 
                double v_2d = img_msg->v_point[i];

                double vec_x = img_msg->velocity_x[i];
                double vec_y = img_msg->velocity_y[i];

                Eigen::Matrix<double 7, 1> xyz_uv_vec;///归一化的3d坐标，像素坐标，速度
                xyz_uv_vec << x, y, z, u_2d, v_2d, vec_x, vec_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_vec);
            }

            estimator.ProcessIMG(image, img_msg->header);

            if(estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
            {
                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs_[WINDOW_SIZE]);
                p_wi = estimator.Ps_[WINDOW_SIZE];
                vPath_.push_back(p_wi);///画轨迹
                double stamp = estimator.Headers_[WINDOW_SIZE];
                ///输出balabala
            }
        }
        m_estimator_.unlock();
    }

}
