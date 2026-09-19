/*
 * Copyright 2026 Zone 404
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <noether_tpp/core/tool_path_recipe.h>
#include <noether_tpp/serialization.h>

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

using namespace noether;

namespace
{
/** @brief Fewest waypoints a delivered pass may carry, as z404_sdg_planning reads them */
const std::size_t kMinPoses = 4;

/** @brief Largest position difference still counted as the same generated pass, in metres */
const double kFingerprintTolerance = 1e-9;

/** @brief Builds a straight run of waypoints along x, one every 10 mm, starting at @p x0 */
ToolPathSegment straightSegment(double x0, std::size_t count)
{
  ToolPathSegment segment;
  for (std::size_t i = 0; i < count; ++i)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x0 + 0.01 * static_cast<double>(i), 0.0, 0.0);
    segment.push_back(pose);
  }
  return segment;
}

/** @brief A straight run of `count` waypoints from `from` to `to` */
ToolPathSegment lineSegment(const Eigen::Vector3d& from, const Eigen::Vector3d& to, std::size_t count)
{
  ToolPathSegment segment;
  for (std::size_t i = 0; i < count; ++i)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = from + (to - from) * (static_cast<double>(i) / static_cast<double>(count - 1));
    segment.push_back(pose);
  }
  return segment;
}

/**
 * @brief Three bands in a U on the plane: the left side upward, the bottom rightward, the
 * right side upward. Sides one metre high, bottom two metres long, so no two distances tie.
 */
std::vector<ToolPathSegment> uBands()
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(lineSegment(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 0), 6));
  generated.push_back(lineSegment(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 0, 0), 11));
  generated.push_back(lineSegment(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(2, 1, 0), 6));
  return generated;
}

/** @brief Builds a source entry */
SourceSegment source(std::size_t index, bool reversed)
{
  SourceSegment entry;
  entry.segment = index;
  entry.reversed = reversed;
  return entry;
}

/** @brief Reads a saved tool path file and flattens it into the list of passes */
std::vector<ToolPathSegment> loadFlattened(const std::string& file)
{
  const YAML::Node root = YAML::LoadFile(file);
  return flattenToolPaths(root.as<std::vector<ToolPaths>>());
}

/** @brief The retouching the maintainer applied by hand to the 5803024014 tool path */
PathRecipe handEditRecipe()
{
  PathRecipe recipe;
  std::vector<SourceSegment> pass;

  pass.clear();
  pass.push_back(source(0, false));
  pass.push_back(source(2, false));
  recipe.push_back(pass);

  pass.clear();
  pass.push_back(source(6, false));
  recipe.push_back(pass);

  pass.clear();
  pass.push_back(source(5, true));
  pass.push_back(source(3, true));
  pass.push_back(source(1, true));
  recipe.push_back(pass);

  pass.clear();
  pass.push_back(source(4, false));
  recipe.push_back(pass);

  return recipe;
}

}  // namespace

TEST(ToolPathRecipe, IdentityDeliversTheGenerationUnchanged)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 6));
  generated.push_back(straightSegment(1.0, 5));

  const ToolPath delivered = assemblePasses(generated, identityRecipe(generated.size()));

  ASSERT_EQ(delivered.size(), 2u);
  EXPECT_EQ(delivered[0].size(), 6u);
  EXPECT_EQ(delivered[1].size(), 5u);
  EXPECT_TRUE(delivered[0][0].translation().isApprox(generated[0][0].translation()));
}

TEST(ToolPathRecipe, WeldDropsOneWaypointOnEachSideOfTheJoint)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 6));
  generated.push_back(straightSegment(1.0, 5));

  PathRecipe recipe;
  std::vector<SourceSegment> pass;
  pass.push_back(source(0, false));
  pass.push_back(source(1, false));
  recipe.push_back(pass);

  const ToolPath delivered = assemblePasses(generated, recipe);

  ASSERT_EQ(delivered.size(), 1u);
  // 6 + 5, moins le retrait de la premiere et l'approche de la seconde
  EXPECT_EQ(delivered[0].size(), 9u);
  EXPECT_TRUE(delivered[0].front().translation().isApprox(generated[0].front().translation()));
  EXPECT_TRUE(delivered[0].back().translation().isApprox(generated[1].back().translation()));
  EXPECT_TRUE(delivered[0][4].translation().isApprox(generated[0][4].translation()));
  EXPECT_TRUE(delivered[0][5].translation().isApprox(generated[1][1].translation()));
}

TEST(ToolPathRecipe, ReversalTurnsThePassRound)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 5));

  PathRecipe recipe;
  recipe.push_back(std::vector<SourceSegment>(1, source(0, true)));

  const ToolPath delivered = assemblePasses(generated, recipe);

  ASSERT_EQ(delivered.size(), 1u);
  ASSERT_EQ(delivered[0].size(), 5u);
  EXPECT_TRUE(delivered[0].front().translation().isApprox(generated[0].back().translation()));
  EXPECT_TRUE(delivered[0].back().translation().isApprox(generated[0].front().translation()));
}

TEST(ToolPathRecipe, AnUnlistedPassIsDropped)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 6));
  generated.push_back(straightSegment(1.0, 6));

  PathRecipe recipe;
  recipe.push_back(std::vector<SourceSegment>(1, source(1, false)));

  const ToolPath delivered = assemblePasses(generated, recipe);

  ASSERT_EQ(delivered.size(), 1u);
  EXPECT_TRUE(delivered[0].front().translation().isApprox(generated[1].front().translation()));
}

TEST(ToolPathRecipe, RefusesAnIndexOutOfRangeOrUsedTwice)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 6));

  PathRecipe out_of_range;
  out_of_range.push_back(std::vector<SourceSegment>(1, source(3, false)));
  EXPECT_THROW(assemblePasses(generated, out_of_range), std::runtime_error);

  PathRecipe twice;
  twice.push_back(std::vector<SourceSegment>(1, source(0, false)));
  twice.push_back(std::vector<SourceSegment>(1, source(0, false)));
  EXPECT_THROW(assemblePasses(generated, twice), std::runtime_error);
}

TEST(ToolPathRecipe, ValidationRefusesAPassShorterThanTheContract)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 3));
  generated.push_back(straightSegment(1.0, 2));

  PathRecipe recipe;
  std::vector<SourceSegment> pass;
  pass.push_back(source(0, false));
  pass.push_back(source(1, false));
  recipe.push_back(pass);

  // 3 + 2, moins une pose de chaque cote de la soudure : 3 poses, une de moins que le contrat
  const ToolPath delivered = assemblePasses(generated, recipe);
  ASSERT_EQ(delivered[0].size(), 3u);

  const std::vector<std::string> faults = validateRecipe(generated, recipe, kMinPoses);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_NE(faults[0].find("Delivered pass 1"), std::string::npos);

  // Livree seule, sans soudure, une passe assez longue passe la validation
  std::vector<ToolPathSegment> long_enough;
  long_enough.push_back(straightSegment(0.0, kMinPoses));
  EXPECT_TRUE(validateRecipe(long_enough, identityRecipe(1), kMinPoses).empty());
}

TEST(ToolPathRecipe, ValidationRefusesARecipeThatDeliversNothing)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 6));

  const std::vector<std::string> faults = validateRecipe(generated, PathRecipe(), kMinPoses);
  ASSERT_EQ(faults.size(), 1u);
}

TEST(ToolPathRecipe, WeldMetricsMeasureTheWaypointsThatBecomeNeighbours)
{
  const ToolPathSegment before = straightSegment(0.0, 5);  // derniere pose de contact en x = 0.03
  const ToolPathSegment after = straightSegment(1.0, 5);   // premiere pose de contact en x = 1.01

  const WeldMetrics metrics = computeWeldMetrics(before, after);

  EXPECT_NEAR(metrics.gap, 0.98, 1e-12);
  EXPECT_NEAR(metrics.angle, 0.0, 1e-12);

  EXPECT_LT(computeWeldMetrics(straightSegment(0.0, 1), after).gap, 0.0);
}

TEST(ToolPathRecipe, FingerprintsFlagAStaleRecipe)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 6));
  generated.push_back(straightSegment(1.0, 6));
  const std::vector<SegmentFingerprint> recorded = fingerprintSegments(generated);

  EXPECT_TRUE(staleSegments(recorded, recorded, kFingerprintTolerance).empty());

  std::vector<ToolPathSegment> moved = generated;
  moved[1] = straightSegment(1.5, 6);
  const std::vector<std::size_t> stale = staleSegments(recorded, fingerprintSegments(moved), kFingerprintTolerance);
  ASSERT_EQ(stale.size(), 1u);
  EXPECT_EQ(stale[0], 1u);

  std::vector<ToolPathSegment> shorter;
  shorter.push_back(generated[0]);
  const std::vector<std::size_t> missing =
      staleSegments(recorded, fingerprintSegments(shorter), kFingerprintTolerance);
  ASSERT_EQ(missing.size(), 1u);
  EXPECT_EQ(missing[0], 1u);
}

TEST(ToolPathRecipe, RecipeFileSurvivesAWriteAndARead)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 6));
  generated.push_back(straightSegment(1.0, 6));

  RecipeFile written;
  written.source = fingerprintSegments(generated);
  written.recipe.push_back(std::vector<SourceSegment>(1, source(1, true)));
  std::vector<SourceSegment> joined;
  joined.push_back(source(0, false));
  written.recipe.push_back(joined);

  const std::string file = std::string(TEST_TMP_DIR) + "/recipe_round_trip.yaml";
  writeRecipeFile(file, written);
  const RecipeFile read = readRecipeFile(file);
  std::remove(file.c_str());

  ASSERT_EQ(read.recipe.size(), written.recipe.size());
  EXPECT_EQ(read.recipe[0][0].segment, 1u);
  EXPECT_TRUE(read.recipe[0][0].reversed);
  EXPECT_EQ(read.recipe[1][0].segment, 0u);
  EXPECT_FALSE(read.recipe[1][0].reversed);
  EXPECT_TRUE(staleSegments(read.source, written.source, kFingerprintTolerance).empty());
}

TEST(PoseMoves, ReplaceThePoseAndIgnoreWhatNoLongerExists)
{
  const std::vector<ToolPathSegment> generated = { straightSegment(0.0, 6), straightSegment(1.0, 6) };
  PoseMove move;
  move.segment = 1;
  move.index = 2;
  move.pose.translation() = Eigen::Vector3d(5.0, 5.0, 5.0);
  PoseMove stale;
  stale.segment = 7;
  stale.index = 0;
  const std::vector<ToolPathSegment> moved = applyPoseMoves(generated, { move, stale });
  ASSERT_EQ(moved.size(), 2u);
  EXPECT_TRUE(moved[1][2].translation().isApprox(Eigen::Vector3d(5.0, 5.0, 5.0)));
  EXPECT_TRUE(moved[1][1].isApprox(generated[1][1]));
  EXPECT_TRUE(moved[0][0].isApprox(generated[0][0]));
}

TEST(PoseMoves, TheLastMoveOfTheSamePoseWins)
{
  const std::vector<ToolPathSegment> generated = { straightSegment(0.0, 6) };
  PoseMove first;
  first.pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  PoseMove second;
  second.pose.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  EXPECT_NEAR(applyPoseMoves(generated, { first, second })[0][0].translation().x(), 2.0, 1e-12);
}

TEST(PoseMoves, LocateFindsTheNearestPoseWithinTolerance)
{
  const std::vector<ToolPathSegment> generated = { straightSegment(0.0, 6), straightSegment(1.0, 6) };
  PoseMove found;
  ASSERT_TRUE(locatePose(generated, Eigen::Vector3d(1.031, 0.002, 0.0), 0.01, found));
  EXPECT_EQ(found.segment, 1u);
  EXPECT_EQ(found.index, 3u);
  EXPECT_FALSE(locatePose(generated, Eigen::Vector3d(0.5, 0.0, 0.0), 0.01, found));
}

TEST(PoseMoves, SurviveTheRecipeFile)
{
  RecipeFile written;
  written.source = fingerprintSegments({ straightSegment(0.0, 6) });
  written.recipe = identityRecipe(1);
  PoseMove move;
  move.segment = 0;
  move.index = 4;
  move.pose.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
  move.pose.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
  written.moves.push_back(move);

  const std::string file = std::string(TEST_TMP_DIR) + "/recipe_moves.yaml";
  writeRecipeFile(file, written);
  const RecipeFile read = readRecipeFile(file);
  std::remove(file.c_str());

  ASSERT_EQ(read.moves.size(), 1u);
  EXPECT_EQ(read.moves[0].segment, 0u);
  EXPECT_EQ(read.moves[0].index, 4u);
  EXPECT_TRUE(read.moves[0].pose.isApprox(move.pose, 1e-9));
}

TEST(PoseMoves, AVersionOneFileReadsWithoutMoves)
{
  const std::string file = std::string(TEST_TMP_DIR) + "/recipe_v1.yaml";
  std::ofstream stream(file.c_str());
  stream << "schema_version: 1\nsource:\n  segments: []\npasses: []\n";
  stream.close();
  const RecipeFile read = readRecipeFile(file);
  std::remove(file.c_str());
  EXPECT_TRUE(read.moves.empty());
  EXPECT_TRUE(read.recipe.empty());
}

TEST(ToolPathRecipe, RefusesAMalformedRecipeFileNamingIt)
{
  const std::string file = std::string(TEST_TMP_DIR) + "/recipe_malformed.yaml";

  // Bonne version de schema, mais rien d'autre : sans refus, la recette degenererait en
  // silence vers une livraison vide
  {
    std::ofstream out(file.c_str());
    out << "schema_version: 1\n";
  }
  EXPECT_THROW(readRecipeFile(file), std::runtime_error);

  // `passes` present mais `source` absent
  {
    std::ofstream out(file.c_str());
    out << "schema_version: 1\npasses: []\n";
  }
  EXPECT_THROW(readRecipeFile(file), std::runtime_error);

  // Version de schema inconnue
  {
    std::ofstream out(file.c_str());
    out << "schema_version: 99\n";
  }
  EXPECT_THROW(readRecipeFile(file), std::runtime_error);

  // Le message nomme toujours le fichier, sinon l'operateur ne sait pas lequel des siens
  try
  {
    readRecipeFile(file);
    FAIL() << "la lecture aurait du refuser";
  }
  catch (const std::runtime_error& ex)
  {
    EXPECT_NE(std::string(ex.what()).find(file), std::string::npos);
  }
  std::remove(file.c_str());

  EXPECT_THROW(readRecipeFile(std::string(TEST_TMP_DIR) + "/aucun_fichier.yaml"), std::runtime_error);
}

TEST(ToolPathRecipe, ReproducesTheHandEditOfWorkpiece5803024014)
{
  const std::vector<ToolPathSegment> generated =
      loadFlattened(std::string(TOOL_PATH_DIR) + "/generated_tool_path.yaml");
  const std::vector<ToolPathSegment> expected =
      loadFlattened(std::string(TOOL_PATH_DIR) + "/hand_edited_tool_path.yaml");

  ASSERT_EQ(generated.size(), 7u);
  ASSERT_EQ(expected.size(), 4u);

  const PathRecipe recipe = handEditRecipe();
  EXPECT_TRUE(validateRecipe(generated, recipe, kMinPoses).empty());

  const ToolPath delivered = assemblePasses(generated, recipe);

  ASSERT_EQ(delivered.size(), expected.size());
  for (std::size_t p = 0; p < delivered.size(); ++p)
  {
    ASSERT_EQ(delivered[p].size(), expected[p].size()) << "pass " << p;
    for (std::size_t w = 0; w < delivered[p].size(); ++w)
    {
      EXPECT_TRUE(delivered[p][w].isApprox(expected[p][w], 1e-12)) << "pass " << p << ", waypoint " << w;
    }
  }
}

TEST(ToolPathRecipe, NoWaypointIsInventedByTheHandEdit)
{
  const std::vector<ToolPathSegment> generated =
      loadFlattened(std::string(TOOL_PATH_DIR) + "/generated_tool_path.yaml");
  const ToolPath delivered = assemblePasses(generated, handEditRecipe());

  for (std::size_t p = 0; p < delivered.size(); ++p)
  {
    for (std::size_t w = 0; w < delivered[p].size(); ++w)
    {
      bool found = false;
      for (std::size_t g = 0; g < generated.size() && !found; ++g)
      {
        for (std::size_t i = 0; i < generated[g].size() && !found; ++i)
        {
          found = delivered[p][w].isApprox(generated[g][i], 1e-12);
        }
      }
      EXPECT_TRUE(found) << "pass " << p << ", waypoint " << w << " exists nowhere in the generation";
    }
  }
}

TEST(ToolPathRecipe, NestingRoundTripsThroughTheFileFormat)
{
  const std::vector<ToolPathSegment> generated =
      loadFlattened(std::string(TOOL_PATH_DIR) + "/generated_tool_path.yaml");
  const ToolPath delivered = assemblePasses(generated, handEditRecipe());

  const std::vector<ToolPaths> nested = nestToolPath(delivered);
  ASSERT_EQ(nested.size(), 1u);
  ASSERT_EQ(nested[0].size(), 1u);
  EXPECT_EQ(flattenToolPaths(nested).size(), delivered.size());
}

TEST(ChainPasses, WalksTheUFromTheBottomAndTurnsTheSideItReachesByItsEnd)
{
  const std::vector<ToolPathSegment> generated = uBands();
  const PathRecipe chained = chainPasses(generated, identityRecipe(3), 1);

  ASSERT_EQ(chained.size(), 1u);
  ASSERT_EQ(chained.front().size(), 3u);
  // Bottom rightward, then the right side upward, then the left side reached by its top: reversed
  EXPECT_EQ(chained.front()[0].segment, 1u);
  EXPECT_FALSE(chained.front()[0].reversed);
  EXPECT_EQ(chained.front()[1].segment, 2u);
  EXPECT_FALSE(chained.front()[1].reversed);
  EXPECT_EQ(chained.front()[2].segment, 0u);
  EXPECT_TRUE(chained.front()[2].reversed);
}

TEST(ChainPasses, StartsWithTheGivenPassInItsOwnDirection)
{
  const PathRecipe chained = chainPasses(uBands(), identityRecipe(3), 0);
  ASSERT_EQ(chained.front().size(), 3u);
  EXPECT_EQ(chained.front()[0].segment, 0u);
  EXPECT_FALSE(chained.front()[0].reversed) << "the starting pass is never turned";
  EXPECT_EQ(chained.front()[1].segment, 1u);
  EXPECT_EQ(chained.front()[2].segment, 2u);
}

TEST(ChainPasses, KeepsTheWeldsAlreadyMade)
{
  PathRecipe recipe;
  recipe.push_back(std::vector<SourceSegment>(1, source(0, false)));
  std::vector<SourceSegment> welded;
  welded.push_back(source(1, false));
  welded.push_back(source(2, false));
  recipe.push_back(welded);

  const PathRecipe chained = chainPasses(uBands(), recipe, 1);
  ASSERT_EQ(chained.size(), 1u);
  ASSERT_EQ(chained.front().size(), 3u);
  EXPECT_EQ(chained.front()[0].segment, 1u);
  EXPECT_EQ(chained.front()[1].segment, 2u);
  EXPECT_EQ(chained.front()[2].segment, 0u);
}

TEST(ChainPasses, DeliversOnePassThatAssembles)
{
  const std::vector<ToolPathSegment> generated = uBands();
  const ToolPath passes = assemblePasses(generated, chainPasses(generated, identityRecipe(3), 1));
  ASSERT_EQ(passes.size(), 1u);
  // Each weld drops one waypoint on either side of the joint: 6 + 11 + 6 minus two joints twice
  EXPECT_EQ(passes.front().size(), 6u + 11u + 6u - 4u);
}

TEST(ChainPasses, RefusesAStartThatIsNotDelivered)
{
  EXPECT_THROW(chainPasses(uBands(), identityRecipe(3), 3), std::out_of_range);
}

TEST(ReverseSources, TurnsTheOrderAndEveryPiece)
{
  std::vector<SourceSegment> sources;
  sources.push_back(source(4, false));
  sources.push_back(source(7, true));
  const std::vector<SourceSegment> reversed = reverseSources(sources);
  ASSERT_EQ(reversed.size(), 2u);
  EXPECT_EQ(reversed[0].segment, 7u);
  EXPECT_FALSE(reversed[0].reversed);
  EXPECT_EQ(reversed[1].segment, 4u);
  EXPECT_TRUE(reversed[1].reversed);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
