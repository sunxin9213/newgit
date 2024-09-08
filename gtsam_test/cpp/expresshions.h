#pragma pnce

#include "functions.h"

#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

namespace gtsamexamples{

    inline gtsam::Point2_ projectPose2_(const gtsam::Pose2_ & pose)
    {
        return gtsam::Point2_(&projectPose2, pose);
    }
}