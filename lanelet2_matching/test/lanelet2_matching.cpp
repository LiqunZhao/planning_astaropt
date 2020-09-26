#include "gtest/gtest.h"

#include "LaneletMatching.h"

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

using namespace lanelet;

class MatchingUtils : public ::testing::Test {
 public:
  MatchingUtils() { map = load(exampleMapPath, projector); }
  std::string exampleMapPath = std::string(PKG_DIR) + "/../lanelet2/lanelet2_maps/res/mapping_example.osm";
  projection::UtmProjector projector{Origin({49, 8.4})};
  LaneletMapPtr map;
};

TEST_F(MatchingUtils, fixtureSetupSuccessful) {  // NOLINT
  EXPECT_TRUE(map->laneletLayer.exists(42440));
}

TEST_F(MatchingUtils, absoluteHull) {  // NOLINT
  matching::Object2d obj;

  obj.pose.translation() = BasicPoint2d{10, 0};                        //!< at point x=10 y=0
  obj.pose.linear() = Eigen::Rotation2D<double>(0.5 * M_PI).matrix();  //!< rotated by pi/2

  obj.relativeHull = matching::Hull2d{BasicPoint2d{-0.5, -1}, BasicPoint2d{2, 1}};

  BasicPolygon2d absoluteHull = matching::utils::absoluteHull(obj);

  EXPECT_DOUBLE_EQ(10 + 1, absoluteHull.at(0).x());
  EXPECT_DOUBLE_EQ(0 - 0.5, absoluteHull.at(0).y());

  EXPECT_DOUBLE_EQ(10 - 1, absoluteHull.at(1).x());
  EXPECT_DOUBLE_EQ(0 + 2, absoluteHull.at(1).y());
}

TEST_F(MatchingUtils, findWithin) {  // NOLINT
  ASSERT_TRUE(map->pointLayer.exists(41656));
  matching::Object2d obj;
  obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
  obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();

  EXPECT_THROW(matching::utils::findWithin(map->laneletLayer, obj, 0.1), MatchingError);

  obj.relativeHull =
      matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}};

  std::vector<std::pair<double, Lanelet>> laneletsWithDistance =
      matching::utils::findWithin(map->laneletLayer, obj, 0.1);

  EXPECT_EQ(4, laneletsWithDistance.size());

  Ids laneletIds;
  std::transform(laneletsWithDistance.begin(), laneletsWithDistance.end(), std::back_inserter(laneletIds),
                 [](const auto& elem) { return elem.second.id(); });

  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45356) != std::end(laneletIds))
      << "Lanelet before the roundabout";
  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45328) != std::end(laneletIds))
      << "Lanelet leaving the roundabout";
  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45334) != std::end(laneletIds))
      << "Lanelet entering the roundabout";
  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45344) != std::end(laneletIds))
      << "Zebra crossing before the roundabout";
  EXPECT_FALSE(std::find(std::begin(laneletIds), std::end(laneletIds), 45332) != std::end(laneletIds))
      << "Lanelet inside the roundabout";
}

TEST_F(MatchingUtils, getMahalanobisDistSq) {  // NOLINT
  ASSERT_TRUE(map->pointLayer.exists(41656));
  matching::ObjectWithCovariance2d obj;
  obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
  obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
  obj.relativeHull =
      matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{-1, 0.9}};

  using namespace matching::utils;
  obj.orientationCovariance = 2.;
  EXPECT_THROW(getMahalanobisDistSq(map->laneletLayer.get(45356), obj), MatchingError);

  obj.positionCovariance = obj.positionCovariance.Ones();
  EXPECT_THROW(getMahalanobisDistSq(map->laneletLayer.get(45356), obj), MatchingError);

  obj.orientationCovariance = 0.;
  obj.positionCovariance = obj.positionCovariance.Identity() * 2.;
  EXPECT_THROW(getMahalanobisDistSq(map->laneletLayer.get(45356), obj), MatchingError);

  obj.orientationCovariance = 10. / 180. * M_PI;

  double mahaDist45334 = getMahalanobisDistSq(map->laneletLayer.get(45334), obj);
  EXPECT_NEAR(0.28817716004330124, mahaDist45334, 10e-6) << "Lanelet entering the roundabout";

  double mahaDist45356 = getMahalanobisDistSq(map->laneletLayer.get(45356), obj);
  EXPECT_NEAR(3.2487924075660697, mahaDist45356, 10e-6) << "Lanelet before the roundabout";

  double mahaDist45328 = getMahalanobisDistSq(map->laneletLayer.get(45328), obj);
  EXPECT_NEAR(11.919014409926168, mahaDist45328, 10e-6) << "Lanelet leaving the roundabout";

  double mahaDist45344 = getMahalanobisDistSq(map->laneletLayer.get(45344), obj);
  EXPECT_NEAR(57.419088438833477, mahaDist45344, 10e-6) << "Zebra crossing before the roundabout";

  double mahaDist45332 = getMahalanobisDistSq(map->laneletLayer.get(45332), obj);
  EXPECT_NEAR(47.481075513744784, mahaDist45332, 10e-6) << "Zebra crossing before the roundabout";
}

class Matching : public MatchingUtils {
 public:
  Matching() {
    obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
    obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
    obj.relativeHull =
        matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{-1, 0.9}};
    obj.positionCovariance = obj.positionCovariance.Identity() * 2.;
    obj.orientationCovariance = 10. / 180. * M_PI;
  }
  matching::ObjectWithCovariance2d obj;
};

TEST_F(Matching, deterministicNonConst) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getDeterministicMatches(*map, obj, 4.);
  for (size_t i = 1; i < matches.size(); i++) {
    EXPECT_TRUE(matches.at(i).distance >= matches.at(i - 1).distance)
        << "Not sorted: at i=" << i - 1 << " dist = " << matches.at(i - 1).distance << "at i=" << i
        << " dist = " << matches.at(i).distance;
  }
  EXPECT_EQ(7, matches.size());
  EXPECT_NEAR(0.69, matches.at(4).distance, 0.1);
  EXPECT_EQ(45330, matches.at(4).lanelet.id());
}

TEST_F(Matching, deterministicConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getDeterministicMatches(constMap, obj, 4.);
  EXPECT_EQ(7, matches.size());
}

TEST_F(Matching, probabilisticNonConst) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getProbabilisticMatches(*map, obj, 4.);
  for (size_t i = 1; i < matches.size(); i++) {
    EXPECT_TRUE(matches.at(i).mahalanobisDistSq >= matches.at(i - 1).mahalanobisDistSq) << "Not sorted at i=" << i;
  }
  EXPECT_NEAR(0.288177, matches.at(0).mahalanobisDistSq, 0.001);
  EXPECT_EQ(45334, matches.at(0).lanelet.id());
  EXPECT_EQ(7, matches.size());
}

TEST_F(Matching, probabilisticConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getProbabilisticMatches(constMap, obj, 4.);
  EXPECT_EQ(7, matches.size());
}

TEST_F(Matching, isCloseTo) {  // NOLINT
  using namespace lanelet::matching;

  LaneletLayer emptyLayer;
  EXPECT_TRUE(isCloseTo(map->laneletLayer, obj, 4.));
  EXPECT_FALSE(isCloseTo(emptyLayer, obj, 4.));

  EXPECT_TRUE(isWithin(map->laneletLayer, obj));
  EXPECT_FALSE(isWithin(emptyLayer, obj));

  Object2d objWithEmptyHull;
  EXPECT_THROW(matching::isCloseTo(map->laneletLayer, objWithEmptyHull, 0.1), MatchingError);
}
