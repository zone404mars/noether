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
#include <noether_gui/widgets/view_cube.h>

#include <gtest/gtest.h>

using namespace noether;

TEST(ViewCube, TheCentreOfAFaceGivesItsAxis)
{
  // Critere §22 : case centrale de « Dessus (+Z) » -> camera sur +Z, haut +Y comme le bouton Dessus
  const Eigen::Vector3d direction = cubeZoneDirection(Eigen::Vector3d(0.05, -0.1, 0.5));
  EXPECT_TRUE(direction.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(defaultViewUp(direction).isApprox(Eigen::Vector3d::UnitY()));
}

TEST(ViewCube, AnEdgeGivesTheBisectorAndACornerTheDiagonal)
{
  // Critere §22 : case de bord entre « Face (-Y) » et « Droite (+X) » -> (1, -1, 0), haut +Z
  const Eigen::Vector3d edge = cubeZoneDirection(Eigen::Vector3d(0.4, -0.5, 0.0));
  EXPECT_TRUE(edge.isApprox(Eigen::Vector3d(1, -1, 0).normalized()));
  EXPECT_TRUE(defaultViewUp(edge).isApprox(Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d corner = cubeZoneDirection(Eigen::Vector3d(-0.45, 0.4, 0.5));
  EXPECT_TRUE(corner.isApprox(Eigen::Vector3d(-1, 1, 1).normalized()));
  EXPECT_TRUE(defaultViewUp(corner).isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(ViewCube, ARayFindsTheFaceItEnters)
{
  const std::optional<Eigen::Vector3d> hit = cubeHit(Eigen::Vector3d(0.1, -3.0, 0.2), Eigen::Vector3d::UnitY());
  ASSERT_TRUE(hit.has_value());
  EXPECT_TRUE(hit->isApprox(Eigen::Vector3d(0.1, -0.5, 0.2)));
  EXPECT_FALSE(cubeHit(Eigen::Vector3d(2.0, -3.0, 0.0), Eigen::Vector3d::UnitY()).has_value());
  EXPECT_FALSE(cubeHit(Eigen::Vector3d(0.0, -3.0, 0.0), -Eigen::Vector3d::UnitY()).has_value());
}

TEST(ViewCube, ArrowsLeadToTheNeighbourFacesOfTheFrontView)
{
  // Critere §22 : vue « Face (-Y) », fleches droite, gauche, haut, bas -> +X, -X, +Z, -Z
  const Eigen::Vector3d front = -Eigen::Vector3d::UnitY();
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(neighborFace(front, up, CubeArrow::Right).isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(neighborFace(front, up, CubeArrow::Left).isApprox(-Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(neighborFace(front, up, CubeArrow::Up).isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(neighborFace(front, up, CubeArrow::Down).isApprox(-Eigen::Vector3d::UnitZ()));
}

TEST(ViewCube, CurvedArrowsTurnTheViewByAQuarter)
{
  // Vue de face, haut +Z, droite de l'ecran +X : en sens horaire, la gauche (-X) monte en haut
  const Eigen::Vector3d front = -Eigen::Vector3d::UnitY();
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(rolledUp(front, up, true).isApprox(-Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(rolledUp(front, up, false).isApprox(Eigen::Vector3d::UnitX()));
}

TEST(ViewCube, ArrowsShowOnlyWithinOneDegreeOfAFace)
{
  EXPECT_TRUE(alignedFace(Eigen::Vector3d(0.0, -1.0, 0.01)).has_value());
  EXPECT_FALSE(alignedFace(Eigen::Vector3d(0.0, -1.0, 0.03)).has_value());
  EXPECT_FALSE(alignedFace(Eigen::Vector3d(1.0, -1.0, 1.0)).has_value());
}
