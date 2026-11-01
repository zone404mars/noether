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
#include <noether_gui/widgets/edited_path.h>
#include <noether_gui/widgets/trace_tool.h>

#include <gtest/gtest.h>

using namespace noether;

namespace
{
/** @brief Minimum de poses d'une passe livree, approche et retrait compris (spec §11.1) */
constexpr std::size_t kMinPoses = 4;
/** @brief Hauteur d'approche des passes de test, en metres */
constexpr double kHeight = 0.05;

/**
 * @brief Passe droite sur le plan z = 0, le long de x, une pose de contact tous les 10 mm, a la
 * hauteur `y` ; approche et retrait a kHeight au-dessus
 */
EditedPass straightPass(const int contacts, const double y = 0.0)
{
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
  for (int k = 0; k < contacts; ++k)
  {
    points.emplace_back(0.01 * k, y, 0.0);
    normals.push_back(Eigen::Vector3d::UnitZ());
  }
  return EditedPass{ withApproachAndRetract(tracePoses(points, normals), kHeight), nullptr };
}

/** @brief Position x de chaque pose de contact d'une passe */
std::vector<double> contactXs(const EditedPass& pass)
{
  std::vector<double> xs;
  for (std::size_t i = 1; i + 1 < pass.poses.size(); ++i)
  {
    xs.push_back(pass.poses[i].translation().x());
  }
  return xs;
}
}  // namespace

TEST(EditedPath, StartsUnmodifiedWithNothingToUndo)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(5) });
  EXPECT_FALSE(path.modified());
  EXPECT_FALSE(path.canUndo());
  EXPECT_FALSE(path.canRedo());
}

TEST(EditedPath, RemovingAMiddlePoseKeepsTheOthersInPlaceAndOrder)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(5) });
  path.removePoses({ PoseRef{ 0, 3 } });
  const std::vector<double> expected = { 0.0, 0.01, 0.03, 0.04 };
  const std::vector<double> xs = contactXs(path.passes()[0]);
  ASSERT_EQ(xs.size(), expected.size());
  for (std::size_t i = 0; i < xs.size(); ++i)
  {
    EXPECT_NEAR(xs[i], expected[i], 1e-12);
  }
  EXPECT_TRUE(path.modified());
}

TEST(EditedPath, ThePoseBeforeAHoleAimsAtItsNewNext)
{
  // Contacts sur une ligne brisee : la pose 2 vise la pose 3, qui part en y ; une fois la pose 3
  // retiree, elle doit viser la pose 4
  EditedPass pass = straightPass(5);
  pass.poses[3].translation().y() = 0.01;
  EditedPath path(kMinPoses);
  path.reset({ pass });
  const Eigen::Isometry3d before = pass.poses[2];
  path.removePoses({ PoseRef{ 0, 3 } });
  const Eigen::Isometry3d& after = path.passes()[0].poses[2];
  const Eigen::Vector3d to_next = path.passes()[0].poses[3].translation() - after.translation();
  EXPECT_TRUE(after.translation().isApprox(before.translation()));
  EXPECT_TRUE(after.linear().col(2).isApprox(before.linear().col(2)));
  EXPECT_TRUE(after.linear().col(0).isApprox(to_next.normalized()));
}

TEST(EditedPath, RemovingTheLastContactTurnsTheNewLastAlongItsLastStretch)
{
  EditedPass pass = straightPass(5);
  pass.poses[4].translation().y() = 0.01;
  EditedPath path(kMinPoses);
  path.reset({ pass });
  path.removePoses({ PoseRef{ 0, 5 } });
  const ToolPathSegment& poses = path.passes()[0].poses;
  const Eigen::Vector3d last_stretch = poses[4].translation() - poses[3].translation();
  EXPECT_TRUE(poses[4].linear().col(0).isApprox(last_stretch.normalized()));
}

TEST(EditedPath, TheApproachFollowsANewFirstContactAtTheSamePlaceInItsFrame)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(5) });
  const ToolPathSegment before = path.passes()[0].poses;
  path.removePoses({ PoseRef{ 0, 1 } });
  const ToolPathSegment& after = path.passes()[0].poses;
  const Eigen::Isometry3d relative_before = before[1].inverse() * before[0];
  const Eigen::Isometry3d relative_after = after[1].inverse() * after[0];
  EXPECT_TRUE(relative_after.isApprox(relative_before));
  EXPECT_NEAR(after[0].translation().x(), 0.01, 1e-12);
}

TEST(EditedPath, TheApproachStaysWhenItsFirstContactIsKept)
{
  // Passe coudee : la pose 1 visera la pose 3, decalee en y, une fois la pose 2 retiree
  EditedPass pass = straightPass(5);
  pass.poses[3].translation().y() = 0.05;
  EditedPath path(kMinPoses);
  path.reset({ pass });
  path.removePoses({ PoseRef{ 0, 2 } });
  const ToolPathSegment& after = path.passes()[0].poses;
  EXPECT_FALSE(after[1].linear().isApprox(pass.poses[1].linear()));
  EXPECT_TRUE(after.front().isApprox(pass.poses.front()));
  EXPECT_TRUE(after.back().isApprox(pass.poses.back()));
}

TEST(EditedPath, RefusesARemovalThatLeavesAPassTooShortAndNamesIt)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(5), straightPass(3, 0.1) });
  // Passe 1 : 5 poses, en perdre deux la laisse a 3
  const std::vector<PoseRef> poses = { PoseRef{ 0, 2 }, PoseRef{ 1, 1 }, PoseRef{ 1, 2 } };
  ASSERT_TRUE(path.passTooShortAfterRemoval(poses).has_value());
  EXPECT_EQ(*path.passTooShortAfterRemoval(poses), 1u);
  EXPECT_THROW(path.removePoses(poses), std::invalid_argument);
  EXPECT_FALSE(path.modified());
  EXPECT_FALSE(path.passTooShortAfterRemoval({ PoseRef{ 0, 2 }, PoseRef{ 1, 1 } }).has_value());
}

TEST(EditedPath, ApproachAndRetractAreNotRemovable)
{
  EXPECT_THROW(withoutContactPoses(straightPass(5).poses, { 0 }), std::invalid_argument);
  EXPECT_THROW(withoutContactPoses(straightPass(5).poses, { 6 }), std::invalid_argument);
}

TEST(EditedPath, UndoAndRedoWalkOneStackOfCopies)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(6) });
  path.removePoses({ PoseRef{ 0, 2 }, PoseRef{ 0, 3 }, PoseRef{ 0, 4 } });
  EXPECT_EQ(path.passes()[0].poses.size(), 5u);
  path.undo();
  EXPECT_EQ(path.passes()[0].poses.size(), 8u);
  EXPECT_FALSE(path.modified());
  path.redo();
  EXPECT_EQ(path.passes()[0].poses.size(), 5u);
}

TEST(EditedPath, ANewActionAfterUndoForgetsWhatCouldBeRedone)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(5), straightPass(5, 0.1) });
  path.removePoses({ PoseRef{ 0, 2 } });
  path.undo();
  path.movePass(1, 0);
  EXPECT_FALSE(path.canRedo());
  path.redo();
  EXPECT_EQ(path.passes()[0].poses.size(), 7u);
  EXPECT_NEAR(path.passes()[0].poses[1].translation().y(), 0.1, 1e-12);
}

TEST(EditedPath, ADragIsOneActionHoweverManyMovesItMakes)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(5) });
  path.beginAction();
  for (int step = 1; step <= 10; ++step)
  {
    Eigen::Isometry3d pose = path.passes()[0].poses[2];
    pose.translation().y() = 0.001 * step;
    path.setPose(PoseRef{ 0, 2 }, pose);
  }
  path.undo();
  EXPECT_NEAR(path.passes()[0].poses[2].translation().y(), 0.0, 1e-12);
  EXPECT_FALSE(path.canUndo());
}

TEST(EditedPath, InsertedPosesJoinTheirGapsInOneAction)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(4) });
  // Une pose au milieu de chacun des deux premiers ecarts de contact
  ToolPathSegment first(1, path.passes()[0].poses[1]);
  first.front().translation().x() = 0.005;
  ToolPathSegment second(1, path.passes()[0].poses[2]);
  second.front().translation().x() = 0.015;
  path.insertPoses({ GapInsertion{ 0, 1, first }, GapInsertion{ 0, 2, second } });
  const std::vector<double> expected = { 0.0, 0.005, 0.01, 0.015, 0.02, 0.03 };
  const std::vector<double> xs = contactXs(path.passes()[0]);
  ASSERT_EQ(xs.size(), expected.size());
  for (std::size_t i = 0; i < xs.size(); ++i)
  {
    EXPECT_NEAR(xs[i], expected[i], 1e-12);
  }
  path.undo();
  EXPECT_EQ(contactXs(path.passes()[0]).size(), 4u);
}

TEST(EditedPath, AnAddedPoseIsAPoseLikeAnyOther)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(4) });
  ToolPathSegment added(1, path.passes()[0].poses[1]);
  added.front().translation().x() = 0.005;
  path.insertPoses({ GapInsertion{ 0, 1, added } });
  path.removePoses({ PoseRef{ 0, 2 } });
  EXPECT_EQ(contactXs(path.passes()[0]).size(), 4u);
  EXPECT_NEAR(contactXs(path.passes()[0])[1], 0.01, 1e-12);
}

TEST(EditedPath, MovesAndRemovesWholePasses)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(4, 0.0), straightPass(4, 0.1), straightPass(4, 0.2) });
  path.movePass(2, 0);
  EXPECT_NEAR(path.passes()[0].poses[1].translation().y(), 0.2, 1e-12);
  EXPECT_NEAR(path.passes()[1].poses[1].translation().y(), 0.0, 1e-12);
  path.removePass(1);
  ASSERT_EQ(path.passes().size(), 2u);
  EXPECT_NEAR(path.passes()[1].poses[1].translation().y(), 0.1, 1e-12);
  EXPECT_THROW(path.removePass(5), std::out_of_range);
}

TEST(EditedPath, ANewGenerationEmptiesBothStacks)
{
  EditedPath path(kMinPoses);
  path.reset({ straightPass(5) });
  path.removePoses({ PoseRef{ 0, 2 } });
  path.removePoses({ PoseRef{ 0, 2 } });
  path.undo();
  path.reset({ straightPass(5) });
  EXPECT_FALSE(path.canUndo());
  EXPECT_FALSE(path.canRedo());
  EXPECT_FALSE(path.modified());
}
