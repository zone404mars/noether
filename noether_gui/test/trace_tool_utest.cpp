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
#include <noether_gui/widgets/trace_tool.h>

#include <gtest/gtest.h>

using namespace noether;

TEST(TraceTool, RefusesAStepThatMeansNothing)
{
  EXPECT_THROW(sampleTrace({ Eigen::Vector3d::Zero() }, 0.0), std::invalid_argument);
  EXPECT_THROW(sampleTrace({ Eigen::Vector3d::Zero() }, -0.01), std::invalid_argument);
}

TEST(TraceTool, ASingleVertexIsASinglePoint)
{
  EXPECT_TRUE(sampleTrace({}, 0.01).empty());
  const std::vector<Eigen::Vector3d> points = sampleTrace({ Eigen::Vector3d(1.0, 2.0, 3.0) }, 0.01);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.front().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(TraceTool, PlacesAPointEveryStepAlongAStraightLine)
{
  // 0,25 m au pas de 0,1 : 0 ; 0,1 ; 0,2 puis l'arrivee a 0,25
  const std::vector<Eigen::Vector3d> points =
      sampleTrace({ Eigen::Vector3d::Zero(), Eigen::Vector3d(0.25, 0.0, 0.0) }, 0.1);
  ASSERT_EQ(points.size(), 4u);
  EXPECT_NEAR(points[1].x(), 0.1, 1e-9);
  EXPECT_NEAR(points[2].x(), 0.2, 1e-9);
  EXPECT_NEAR(points[3].x(), 0.25, 1e-9);
}

TEST(TraceTool, DoesNotRepeatAnArrivalThatFallsOnAStep)
{
  const std::vector<Eigen::Vector3d> points =
      sampleTrace({ Eigen::Vector3d::Zero(), Eigen::Vector3d(0.3, 0.0, 0.0) }, 0.1);
  ASSERT_EQ(points.size(), 4u);
  EXPECT_NEAR(points.back().x(), 0.3, 1e-9);
}

TEST(TraceTool, KeepsTheSpacingAcrossACorner)
{
  // 0,15 m en x puis 0,15 m en y, pas de 0,1 : les points tombent a 0,1 ; 0,2 (soit 0,05 apres le
  // coin) ; 0,3 d'arc, puis l'arrivee
  const std::vector<Eigen::Vector3d> points = sampleTrace(
      { Eigen::Vector3d::Zero(), Eigen::Vector3d(0.15, 0.0, 0.0), Eigen::Vector3d(0.15, 0.15, 0.0) }, 0.1);
  ASSERT_EQ(points.size(), 4u);
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), 1e-9));
  EXPECT_TRUE(points[2].isApprox(Eigen::Vector3d(0.15, 0.05, 0.0), 1e-9));
  EXPECT_TRUE(points[3].isApprox(Eigen::Vector3d(0.15, 0.15, 0.0), 1e-9));
}

TEST(TraceTool, PosesFollowTheNormalAndTheTravel)
{
  const std::vector<Eigen::Vector3d> points = { Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 0.0) };
  const std::vector<Eigen::Vector3d> normals(2, Eigen::Vector3d::UnitZ());
  const ToolPathSegment poses = tracePoses(points, normals);
  ASSERT_EQ(poses.size(), 2u);
  for (const Eigen::Isometry3d& pose : poses)
  {
    EXPECT_TRUE(pose.linear().col(2).isApprox(Eigen::Vector3d::UnitZ()));
    EXPECT_TRUE(pose.linear().col(0).isApprox(Eigen::Vector3d::UnitX()));
    EXPECT_NEAR(pose.linear().determinant(), 1.0, 1e-9);
  }
  EXPECT_TRUE(poses.back().translation().isApprox(points.back()));
}

TEST(TraceTool, StraightensTheTravelAgainstATiltedNormal)
{
  // Normale inclinee : x doit rester orthogonal a z, et z garder la normale
  const Eigen::Vector3d normal = Eigen::Vector3d(0.3, 0.0, 1.0).normalized();
  const ToolPathSegment poses =
      tracePoses({ Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 0.0) }, { normal, normal });
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_TRUE(poses.front().linear().col(2).isApprox(normal));
  EXPECT_NEAR(poses.front().linear().col(0).dot(normal), 0.0, 1e-9);
}

TEST(TraceTool, NoPoseWithoutADirectionOfTravel)
{
  EXPECT_TRUE(tracePoses({ Eigen::Vector3d::Zero() }, { Eigen::Vector3d::UnitZ() }).empty());
  EXPECT_THROW(tracePoses({ Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX() }, { Eigen::Vector3d::UnitZ() }),
               std::invalid_argument);
}

TEST(TraceTool, ApproachAndRetractStandAboveTheEnds)
{
  const std::vector<Eigen::Vector3d> points = { Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 0.0) };
  const std::vector<Eigen::Vector3d> normals(2, Eigen::Vector3d::UnitZ());
  const ToolPathSegment contact = tracePoses(points, normals);
  const ToolPathSegment pass = withApproachAndRetract(contact, 0.05);
  ASSERT_EQ(pass.size(), 4u);
  EXPECT_TRUE(pass.front().translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.05)));
  EXPECT_TRUE(pass.back().translation().isApprox(Eigen::Vector3d(0.1, 0.0, 0.05)));
  EXPECT_TRUE(pass.front().linear().isApprox(contact.front().linear()));
  EXPECT_TRUE(pass[1].isApprox(contact.front()));
  EXPECT_TRUE(pass[2].isApprox(contact.back()));
}

TEST(TraceTool, ApproachFollowsATiltedToolAxis)
{
  // Normale inclinee : l'approche part le long de cet axe, pas a la verticale
  const Eigen::Vector3d normal = Eigen::Vector3d(1.0, 0.0, 1.0).normalized();
  const ToolPathSegment contact =
      tracePoses({ Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.1, 0.0) }, { normal, normal });
  const ToolPathSegment pass = withApproachAndRetract(contact, 0.1);
  EXPECT_TRUE(pass.front().translation().isApprox(normal * 0.1));
}

TEST(TraceTool, RefusesApproachWithoutContactOrHeight)
{
  EXPECT_THROW(withApproachAndRetract({}, 0.05), std::invalid_argument);
  EXPECT_THROW(withApproachAndRetract({ Eigen::Isometry3d::Identity() }, 0.0), std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
