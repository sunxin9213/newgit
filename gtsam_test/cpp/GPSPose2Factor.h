#pragma

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>


class GPSPose2Factor: public gtsam::NoiseModelFactor1<gtsam::Pose2>
{
    private:
        double mx_, my_;
    public:
        GPSPose2Factor(gtsam::Key poseKey, const gtsam::Point2 m, gtsam::SharedNoiseModel model):
        gtsam::NoiseModelFactor1<gtsam::Pose2>(model, poseKey), mx_(m.x()), my_(m.y()){}

        gtsam::Vector evaluateError(const gtsam::Pose2 & p, boost::optional<gtsam::Matrix&> H = boost::none) const
        {
            if(H)
                *H = (gtsam::Matrix23() << 1.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0).finished();
            return (gtsam::Vector2() << p.x() - mx_, p.y() - my_).finished();
        }
};