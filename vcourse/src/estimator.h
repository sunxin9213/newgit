#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include <unordered_map>

#include "utility/utility.h"
#include "factor/interation_bash.h"

#include "feature_manager.h"
#include "initial/intitial_ex_rotaion.h"

class Estimator
{
    public:
        Estimator();
        ~Estimator();

        enum SolverFlag
        {
            INITIAL,
            NON_LINEAR
        };

        enum MarginalizationFlag
        {
            MARGIN_OLD = 0,
            MARGIN_SECOND_NEW = 1
        };

        /// 贺一家修改的
        MatXX Hprior_;
        VecX bprior_;
        VecX errprior_;
        MatXX Jprior_inv_;

        Eigen::Matrix2d project_sqrt_info_;
        SolverFlag solver_flag_;
        Maginalization marginalization_flag_;
        Vector3d g_;
        MatrixXd Ap_[2], backup_A_;
        VectorXd bp_[2], backup_b_;

        Matrix3d ric_[NUM_OF_CAM];///外参
        Matrix3d tic_[NUM_OF_CAM];

        Vector3d Ps_[(WINDOW_SIZE + 1)];
        Vector3d Vs_[(WINDOW_SIZE + 1)];
        Vector3d Rs_[(WINDOW_SIZE + 1)];
        Vector3d Bas_[(WINDOW_SIZE + 1)];
        Vector3d Bgs_[(WINDOW_SIZE + 1)];

        double td_;

        Matrix3d back_R0_, last_R_, last_R0_;
        Vector3d back_P0_, last_p_, last_P0_;

        double Headers_[(WINDOW_SIZE+1)];

        IntegrationBase * pre_integration[(WINDOW_SIZE + 1)];

        Vector3d acc_0_, gyr_0_;

        std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
        std::vector<Vector3d> linear_acc_buf_[(WINDOW_SIZE + 1)];
        std::vector<Vector3d> angular_vec_buf_[(WINDOW_SIZE + 1)];

        int frame_count_;
        int sum_of_outlier_, sum_of_back_, sum_of_front_, sum_if_invalid_;

        FeatureManager f_manager_;
        MotionEstimator m_estimator_;
        InitialEXRotation initial_ex_rotation_;

        bool first_imu_;
        bool is_valid_, is_key_;
        bool failure_occur_;

        std::vector<Vector3d> point_cloud_;
        std::vector<Vector3d> margin_cloud_;
        std::vector<Vector3d> key_pose_;

        double initial_timestamp_;

        double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
        double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
        double para_Feature[NUM_OF_F][SIZE_FEATURE];
        double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
        double para_Retrive_Pose[SIZE_POSE];
        double para_Td[1][1];
        double para_Tr[1][1];

        int loop_window_index_;

        std::vector<double *> last_marginalization_parameter_bocks_;
        std::map<double, ImageFrame> all_image_frame_;
        IntegrationBase * tmp_pre_integration_;

        bool relocalization_info_;
        double relo_frame_stamp_;
        double relo_frame_index_;
        int relo_frame_local_index_;
        std::vector<Vector3d> match_points_;
        double relo_Pose[SIZE_POSE];
        Matrix3d drift_correct_r_;
        Vector2d drift_correct_t_;
        Vector3d pre_relo_t_;
        Matrix3d pre_relo_r_;
        Vector3d relo_relation_t_;
        Quaterniond relo_relative_q;
        double relo_relative_yae_;

        void BackendOptimization();
        void ClearState();
        void Double2Vector();
        void FailureDectect();
        void InitialStucture();

        void MargOldFrame();
        void MargNewFrame();

        void Optimization();

        void ProcessIMU(const double & t, const Vector3d & linear_acc, const Vector3d & angular_vec);
        void ProcessIMG(const std::map<int , std::vector<std::pair<int , Eigen::Matrix<double, 7, 1>>>> & image, const double & stamp);
        void ProblemSolve();

        void RelativePose(const Matrix3d & relative_R, const Vector3d & relative_T, const int & l_frame);

        void SetParameters();
        void SetReloFrame(const double & frame_stamp, const int & frame_index, 
                const std::vector<Vector3d> & match_points,const Vector3d & relo_t, const Matrix3d & relo_r);
        void SlideWindow();
        void SolveOdometry();
        void SlideNewWindow();
        void SlideOldWindow();

        void VisualInitialAlign();
        void Vector2Double();

};

#endif