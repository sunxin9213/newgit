#include "functions.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

namespace gesamexamples
{
    gtsam::Point2 projectPose2(const gtsam::Pose2 & pose, gtsam::OptionalJacobian<2, 3> H)
    {
        if(H)
            *H = (gtsam::Matrix23() << 1, 0, 0,
                                        0, 1, 0).finished();
        return gtsam::Point2(pose.x(), pose.y());
    }
}