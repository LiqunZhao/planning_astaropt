#include "lanelet2_matching/LaneletMatching.h"
#include "lanelet2_matching/Utilities.h"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace {
template <typename LaneletT, typename MatchT>
std::vector<MatchT> toMatchVector(std::vector<std::pair<double, LaneletT>> pairVec) {
  std::vector<MatchT> matchVec;
  matchVec.reserve(pairVec.size());
  std::transform(pairVec.begin(), pairVec.end(), std::back_inserter(matchVec),
                 [](std::pair<double, LaneletT> pair) -> MatchT {
                   MatchT match;
                   match.distance = pair.first;
                   match.lanelet = pair.second;
                   return match;
                 });

  // sort ascending by distance
  std::sort(matchVec.begin(), matchVec.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.distance < rhs.distance; });

  return matchVec;
}

template <typename LaneletT, typename MatchT>
std::vector<MatchT> getProbabilisticMatchesImpl(std::vector<std::pair<double, LaneletT>> pairVec,
                                                const lanelet::matching::ObjectWithCovariance2d& obj) {
  std::vector<MatchT> matchVec;
  matchVec.reserve(pairVec.size());
  std::transform(pairVec.begin(), pairVec.end(), std::back_inserter(matchVec),
                 [&](std::pair<double, LaneletT> pair) -> MatchT {
                   MatchT match;
                   match.distance = pair.first;
                   match.lanelet = pair.second;
                   match.mahalanobisDistSq = lanelet::matching::utils::getMahalanobisDistSq(pair.second, obj);
                   return match;
                 });

  // sort ascending by mahalanobisDistSq
  std::sort(matchVec.begin(), matchVec.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.mahalanobisDistSq < rhs.mahalanobisDistSq; });

  return matchVec;
}
}

namespace lanelet {
namespace matching {

std::vector<LaneletMatch> getDeterministicMatches(LaneletMap& map, const Object2d& obj, double maxDist) {
  return toMatchVector<Lanelet, LaneletMatch>(utils::findWithin(map.laneletLayer, obj, maxDist));
}

std::vector<ConstLaneletMatch> getDeterministicMatches(const LaneletMap& map, const Object2d& obj, double maxDist) {
  return toMatchVector<ConstLanelet, ConstLaneletMatch>(utils::findWithin(map.laneletLayer, obj, maxDist));
}

std::vector<LaneletMatchProbabilistic> getProbabilisticMatches(LaneletMap& map, const ObjectWithCovariance2d& obj,
                                                               double maxDist) {
  auto pairVec = utils::findWithin(map.laneletLayer, obj, maxDist);
  return getProbabilisticMatchesImpl<Lanelet, LaneletMatchProbabilistic>(pairVec, obj);
}

std::vector<ConstLaneletMatchProbabilistic> getProbabilisticMatches(const LaneletMap& map,
                                                                    const ObjectWithCovariance2d& obj, double maxDist) {
  auto pairVec = utils::findWithin(map.laneletLayer, obj, maxDist);
  return getProbabilisticMatchesImpl<ConstLanelet, ConstLaneletMatchProbabilistic>(pairVec, obj);
}

}  // namespace matching
}  // namespace lanelet
