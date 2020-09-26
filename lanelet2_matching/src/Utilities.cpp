#include "lanelet2_matching/Utilities.h"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace {

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
double positiveFloatModulo(double x, double y) {
  if (std::abs(y) < 10.e-9) {
    return x;
  }
  // Result is always positive; not similar to fmod()
  return x - y * floor(x / y);
}

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
double normalizeAngleRadians(double x) { return positiveFloatModulo((x + M_PI), 2.0 * M_PI) - M_PI; }

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
double angleDifference(double targetAngle, double sourceAngle) {
  double angleDiff = targetAngle - sourceAngle;
  return normalizeAngleRadians(angleDiff);
}

// from https://github.com/coincar-sim/util_eigen_geometry/blob/release/src/util_eigen_geometry.cpp
double yawFromAffine2d(const Eigen::Affine2d& pose) {
  Eigen::Rotation2D<double> rot;
  rot.fromRotationMatrix(pose.linear());
  return rot.smallestAngle();
}
}

namespace lanelet {
namespace matching {
namespace utils {

double getMahalanobisDistSq(ConstLanelet lanelet, const ObjectWithCovariance2d& obj) {
  if ((fabs(obj.orientationCovariance) < 10e-9) || obj.positionCovariance.isZero()) {
    throw MatchingError("Covariance must not be zero");
  }
  if (fabs(obj.positionCovariance.determinant()) < 10e-9) {
    throw MatchingError("Determinant must not be zero");
  }
  auto centerline2d = lanelet::utils::to2D(lanelet.centerline());
  ArcCoordinates closestOnCenter = geometry::toArcCoordinates(centerline2d, obj.pose.translation());
  BasicPoint2d pAt = geometry::interpolatedPointAtDistance(centerline2d, closestOnCenter.length);
  BasicPoint2d pBefore =
      geometry::interpolatedPointAtDistance(centerline2d, std::max(closestOnCenter.length - 0.5, 0.));
  BasicPoint2d pAfter = geometry::interpolatedPointAtDistance(centerline2d, closestOnCenter.length + 0.5);
  BasicPoint2d pDirection = pAfter - pBefore;

  double yawCenter = normalizeAngleRadians(std::atan2(pDirection.y(), pDirection.x()));
  double yawObj = normalizeAngleRadians(yawFromAffine2d(obj.pose));
  double yawDiff = angleDifference(yawCenter, yawObj);
  // todo: check if lanelet is one-way
  if (true) {
    double yawCenterReverse = normalizeAngleRadians(yawCenter - M_PI);
    double yawDiffReverse = angleDifference(yawCenterReverse, yawObj);
    if (fabs(yawDiffReverse) < fabs(yawDiff)) {
      yawDiff = yawDiffReverse;
    }
  }
  double mahaDistYawSq = (yawDiff * yawDiff) / (obj.orientationCovariance * obj.orientationCovariance);

  BasicPoint2d pDiff = obj.pose.translation() - pAt;
  double mahaDistPosSq = pDiff.transpose() * obj.positionCovariance.inverse() * pDiff;

  return mahaDistYawSq + mahaDistPosSq;
}

}  // namespace utils
}  // namespace matching
}  // namespace lanelet
