#pragma once

#include "Exceptions.h"
#include "Types.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace lanelet {
namespace matching {
namespace utils {

/**
 * @brief Compute the absolute hull from pose and relative hull
 * @param obj
 * @return
 */
inline BasicPolygon2d absoluteHull(const Object2d& obj) {
  BasicPolygon2d hullPoints;
  hullPoints.reserve(obj.relativeHull.size());
  for (const auto& hullPt : obj.relativeHull) {
    hullPoints.push_back(obj.pose * hullPt);
  }
  return hullPoints;
}

/**
 * @brief Compute the axis aligned bounding box from a hull and extend it by dist
 * @param absoluteHull
 * @param dist
 * @return
 */
inline BoundingBox2d boundingBox2d(const BasicPolygon2d& absoluteHull, double dist) {
  auto minmax_x = std::minmax_element(begin(absoluteHull), end(absoluteHull),
                                      [](const auto& lhs, const auto& rhs) { return lhs.x() < rhs.x(); });
  auto minmax_y = std::minmax_element(begin(absoluteHull), end(absoluteHull),
                                      [](const auto& lhs, const auto& rhs) { return lhs.y() < rhs.y(); });

  BasicPoint2d min{minmax_x.first->x() - dist, minmax_y.first->y() - dist};
  BasicPoint2d max{minmax_x.second->x() + dist, minmax_y.second->y() + dist};

  return {min, max};
}

/**
 * @brief Find all primitives as close as or closer than maxDist to an object
 * @param map
 * @param obj
 * @param maxDist
 * @return
 */
template <typename LayerT>
auto findWithin(LayerT& map, const Object2d& obj, double maxDist)
    -> std::vector<std::pair<double, decltype(map.get(0))>> {
  if (obj.relativeHull.empty()) {
    throw MatchingError("Hull must not be empty");
  }
  using PrimT = decltype(map.get(0));
  BasicPolygon2d absoluteHull = utils::absoluteHull(obj);
  BoundingBox2d boundingBoxWithDist = utils::boundingBox2d(absoluteHull, maxDist);
  std::vector<PrimT> boxBoxApproximation = map.search(boundingBoxWithDist);

  std::vector<std::pair<double, PrimT>> withinVec;
  withinVec.reserve(boxBoxApproximation.size());

  for (auto const& elem : boxBoxApproximation) {
    double dist = boost::geometry::distance(absoluteHull, lanelet::utils::toHybrid(elem.polygon2d()));
    if (dist <= maxDist) {
      withinVec.push_back(std::make_pair(dist, elem));
    }
  }

  return withinVec;
}

/**
 * @brief Compute squared mahalanobis distance based on pose and covariance, hull is not used
 * @param lanelet
 * @param obj
 * @return
 * see D. Petrich, T. Dang, D. Kasper, G. Breuel and C. Stiller,
 * "Map-based long term motion prediction for vehicles in traffic environments,"
 * 16th International IEEE Conference on Intelligent Transportation Systems (ITSC 2013),
 * The Hague, 2013, pp. 2166-2172. doi: 10.1109/ITSC.2013.6728549
 * https://ieeexplore.ieee.org/document/6728549
 */
double getMahalanobisDistSq(ConstLanelet lanelet, const ObjectWithCovariance2d& obj);

}  // namespace utils
}  // namespace matching
}  // namespace lanelet
