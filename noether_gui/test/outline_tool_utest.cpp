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
#include <noether_gui/widgets/outline_tool.h>

#include <gtest/gtest.h>

using namespace noether;

namespace
{
/** @brief Rayon de fermeture des essais, en pixels */
constexpr double kTolerance = 8.0;
}  // namespace

TEST(OutlineBuilder, StartsEmptyAndCannotClose)
{
  const OutlineBuilder outline(kTolerance);
  EXPECT_TRUE(outline.vertices().empty());
  EXPECT_FALSE(outline.canClose());
}

TEST(OutlineBuilder, AClickOnTheFirstVertexClosesOnlyFromThreeVertices)
{
  OutlineBuilder outline(kTolerance);
  EXPECT_FALSE(outline.addVertex(Eigen::Vector2d(100, 100)));
  EXPECT_FALSE(outline.addVertex(Eigen::Vector2d(200, 100)));
  // Deux sommets : revenir sur le premier ne ferme rien, un contour de deux points n'existe pas
  EXPECT_FALSE(outline.addVertex(Eigen::Vector2d(103, 98)));
  EXPECT_EQ(outline.vertices().size(), 3u);

  EXPECT_TRUE(outline.canClose());
  EXPECT_TRUE(outline.addVertex(Eigen::Vector2d(104, 104)));
  EXPECT_EQ(outline.vertices().size(), 3u) << "la fermeture n'ajoute pas de sommet";
}

TEST(OutlineBuilder, AClickJustOutsideTheToleranceAddsAVertex)
{
  OutlineBuilder outline(kTolerance);
  outline.addVertex(Eigen::Vector2d(0, 0));
  outline.addVertex(Eigen::Vector2d(100, 0));
  outline.addVertex(Eigen::Vector2d(100, 100));
  EXPECT_FALSE(outline.addVertex(Eigen::Vector2d(kTolerance + 1.0, 0)));
  EXPECT_EQ(outline.vertices().size(), 4u);
}

TEST(OutlineBuilder, ARepeatedClickIsIgnored)
{
  OutlineBuilder outline(kTolerance);
  outline.addVertex(Eigen::Vector2d(50, 50));
  EXPECT_FALSE(outline.addVertex(Eigen::Vector2d(52, 51)));
  EXPECT_EQ(outline.vertices().size(), 1u);
}

TEST(OutlineBuilder, RemovesTheLastVertexAndClears)
{
  OutlineBuilder outline(kTolerance);
  outline.addVertex(Eigen::Vector2d(0, 0));
  outline.addVertex(Eigen::Vector2d(100, 0));
  outline.removeLastVertex();
  ASSERT_EQ(outline.vertices().size(), 1u);
  EXPECT_DOUBLE_EQ(outline.vertices().front().x(), 0.0);
  outline.removeLastVertex();
  outline.removeLastVertex();
  EXPECT_TRUE(outline.vertices().empty());

  outline.addVertex(Eigen::Vector2d(1, 1));
  outline.clear();
  EXPECT_TRUE(outline.vertices().empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
