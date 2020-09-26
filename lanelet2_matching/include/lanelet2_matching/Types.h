#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lanelet {
namespace matching {

using Pose2d = Eigen::Transform<double, 2, Eigen::Isometry, Eigen::DontAlign>;  //!< a 2d pose
using PositionCovariance2d = Eigen::Matrix<double, 2, 2, Eigen::DontAlign>;     //!< a covariance of a 2d position
using Hull2d =
    std::vector<lanelet::BasicPoint2d, Eigen::aligned_allocator<lanelet::BasicPoint2d>>;  //!< a hull of 2d-points, as
                                                                                          //!objects are usually rings,
                                                                                          //!closing the ring by
                                                                                          //!appending the first point
                                                                                          //!of the polygon as last
                                                                                          //!point again is suggested

struct Object2d {
  Id objectId{InvalId};
  Pose2d pose{Pose2d::Identity()};
  Hull2d relativeHull;
};

struct ObjectWithCovariance2d : Object2d {
  PositionCovariance2d positionCovariance{PositionCovariance2d::Zero()};
  double orientationCovariance{0.};
};

struct LaneletMatch {
  Lanelet lanelet;
  double distance;
};

struct ConstLaneletMatch {
  ConstLanelet lanelet;
  double distance;
};

struct LaneletMatchProbabilistic : LaneletMatch {
  double mahalanobisDistSq;
};

struct ConstLaneletMatchProbabilistic : ConstLaneletMatch {
  double mahalanobisDistSq;
};

}  // namespace matching
}  // namespace lanelet
