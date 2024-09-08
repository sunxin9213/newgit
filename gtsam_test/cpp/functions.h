#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsamexamples
{
    gtsam::Point2 projectPose2(const gtsam::Pose2 & pose, gtsam::OptionalJacobian<2, 3> H = boost::none);
}