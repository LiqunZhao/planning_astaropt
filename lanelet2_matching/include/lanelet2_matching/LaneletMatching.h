#pragma once

#include "Exceptions.h"
#include "Types.h"
#include "Utilities.h"

#include <lanelet2_core/LaneletMap.h>

namespace lanelet {
namespace matching {

/**
 * @brief get deterministic lanelet matches of an object with a maximum distance of maxDist, sorted ascending by
 * distance
 * @param map lanelet map
 * @param obj object
 * @param maxDist
 * @return
 */
std::vector<LaneletMatch> getDeterministicMatches(LaneletMap& map, const Object2d& obj, double maxDist);
std::vector<ConstLaneletMatch> getDeterministicMatches(const LaneletMap& map, const Object2d& obj, double maxDist);

/**
 * @brief get probabilistic lanelet matches of an object with a maximum deterministic euler distance of maxDist, sorted
 * ascending by Mahalanobis distance
 * @param map lanelet map
 * @param obj object
 * @param maxDist
 * @return
 * for the distance computation see D. Petrich, T. Dang, D. Kasper, G. Breuel and C. Stiller,
 * "Map-based long term motion prediction for vehicles in traffic environments,"
 * 16th International IEEE Conference on Intelligent Transportation Systems (ITSC 2013),
 * The Hague, 2013, pp. 2166-2172. doi: 10.1109/ITSC.2013.6728549
 * https://ieeexplore.ieee.org/document/6728549
 */
std::vector<LaneletMatchProbabilistic> getProbabilisticMatches(LaneletMap& map, const ObjectWithCovariance2d& obj,
                                                               double maxDist);
std::vector<ConstLaneletMatchProbabilistic> getProbabilisticMatches(const LaneletMap& map,
                                                                    const ObjectWithCovariance2d& obj, double maxDist);

/**
 * @brief Determine whether an object is within a maximum distance to any primitive of the layer
 * @param map map layer
 * @param obj object
 * @param maxDist
 * @return
 */
template <typename LayerT>
bool isCloseTo(const LayerT& map, const Object2d& obj, double maxDist) {
  if (obj.relativeHull.empty()) {
    throw MatchingError("Hull must not be empty");
  }
  using PrimT = decltype(map.get(0));
  BasicPolygon2d absoluteHull = utils::absoluteHull(obj);
  BoundingBox2d boundingBoxWithDist = utils::boundingBox2d(absoluteHull, maxDist);
  std::vector<PrimT> boxBoxApproximation = map.search(boundingBoxWithDist);

  auto closeTo = [&maxDist, &absoluteHull](const auto& elem) {
    double dist = boost::geometry::distance(absoluteHull, lanelet::utils::toHybrid(elem.polygon2d()));
    return dist <= maxDist;
  };
  return std::any_of(std::begin(boxBoxApproximation), std::end(boxBoxApproximation), closeTo);
}

/**
 * @brief Determine whether an object is (at least partially) within any primitive of the layer
 * @param map map layer
 * @param obj object
 * @return
 */
template <typename LayerT>
bool isWithin(const LayerT& map, const Object2d& obj) {
  return isCloseTo(map, obj, 0.);
}

}  // namespace matching
}  // namespace lanelet
