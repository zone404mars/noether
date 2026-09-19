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
#include <noether_tpp/tool_path_modifiers/start_end_organization_modifier.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

using namespace noether;

namespace
{
/** @brief Une ligne de raster de x = 0 a x = 1, a la hauteur y donnee, six poses */
ToolPathSegment rasterLine(const double y)
{
  ToolPathSegment line;
  for (int i = 0; i <= 5; ++i)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0.2 * i, y, 0.0);
    line.push_back(pose);
  }
  return line;
}

/** @brief Quatre lignes paralleles a y = 0, 0,1, 0,2, 0,3, dans le desordre et toutes de gauche a droite.
 * Chaque ligne est un ToolPath a un segment, comme les planificateurs de raster les rendent. */
ToolPaths fourLines()
{
  return { ToolPath{ rasterLine(0.2) }, ToolPath{ rasterLine(0.0) }, ToolPath{ rasterLine(0.3) }, ToolPath{ rasterLine(0.1) } };
}

Eigen::Vector3d firstPoint(const ToolPaths& paths) { return paths.front().front().front().translation(); }
/** @brief Hauteur de la ligne de rang i */
double lineY(const ToolPaths& paths, std::size_t i) { return paths[i].front().front().translation().y(); }
/** @brief Abscisse du premier point de la ligne de rang i */
double lineFirstX(const ToolPaths& paths, std::size_t i) { return paths[i].front().front().translation().x(); }
Eigen::Vector3d lastPoint(const ToolPaths& paths) { return paths.back().back().back().translation(); }
}  // namespace

TEST(StartEndOrganization, StartsAtTheRequestedCornerAndSnakesToTheOppositeSide)
{
  // Depart en bas a gauche, arrivee en haut a gauche : quatre lignes en serpent y arrivent exactement
  const StartEndOrganizationModifier modifier(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0.3, 0));
  const ToolPaths out = modifier.modify(fourLines());

  ASSERT_EQ(out.size(), 4u) << "toutes les lignes sont conservees";
  EXPECT_TRUE(firstPoint(out).isApprox(Eigen::Vector3d(0, 0, 0)));
  EXPECT_TRUE(lastPoint(out).isApprox(Eigen::Vector3d(0, 0.3, 0)));
  EXPECT_NEAR(modifier.endGap(out), 0.0, 1e-12);
  // Les hauteurs se suivent : 0, 0,1, 0,2, 0,3, et les sens alternent
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(lineY(out, i), 0.1 * i, 1e-12);
  }
  EXPECT_NEAR(lineFirstX(out, 1), 1.0, 1e-12) << "la deuxieme ligne est retournee";
}

TEST(StartEndOrganization, ReportsTheGapWhenTheParityForbidsTheRequestedEnd)
{
  // Depart en bas a gauche, arrivee en haut a DROITE : impossible avec quatre lignes, le serpent
  // finit a gauche. Le depart est tenu et l'ecart a l'arrivee vaut la longueur d'une ligne
  const StartEndOrganizationModifier modifier(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0.3, 0));
  const ToolPaths out = modifier.modify(fourLines());
  EXPECT_TRUE(firstPoint(out).isApprox(Eigen::Vector3d(0, 0, 0)));
  EXPECT_NEAR(modifier.endGap(out), 1.0, 1e-12);
}

TEST(StartEndOrganization, CanStartFromTheFarCornerByReversingTheFirstLine)
{
  const StartEndOrganizationModifier modifier(Eigen::Vector3d(1, 0.3, 0), Eigen::Vector3d(1, 0, 0));
  const ToolPaths out = modifier.modify(fourLines());
  EXPECT_TRUE(firstPoint(out).isApprox(Eigen::Vector3d(1, 0.3, 0)));
  EXPECT_TRUE(lastPoint(out).isApprox(Eigen::Vector3d(1, 0, 0)));
}

TEST(StartEndOrganization, IsIdempotentAndSkipsEmptyLines)
{
  const StartEndOrganizationModifier modifier(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0.3, 0));
  ToolPaths input = fourLines();
  input.push_back(ToolPath{ ToolPathSegment() });
  const ToolPaths once = modifier.modify(input);
  const ToolPaths twice = modifier.modify(once);
  ASSERT_EQ(once.size(), 4u);
  ASSERT_EQ(twice.size(), 4u);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_TRUE(once[i].front().front().isApprox(twice[i].front().front()));
  }
}

TEST(StartEndOrganization, RoundTripsThroughYaml)
{
  const StartEndOrganizationModifier modifier(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(1, 2, 3));
  const YAML::Node node = YAML::convert<StartEndOrganizationModifier>::encode(modifier);
  StartEndOrganizationModifier back = node.as<StartEndOrganizationModifier>();
  EXPECT_NEAR(back.endGap({ ToolPath{ rasterLine(2.0) } }), (Eigen::Vector3d(1, 2, 0) - Eigen::Vector3d(1, 2, 3)).norm(), 1e-12);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
