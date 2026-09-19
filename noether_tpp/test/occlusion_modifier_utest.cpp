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
#include <noether_tpp/tool_path_modifiers/occlusion_modifier.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtest/gtest.h>

using namespace noether;

namespace
{
/** @brief Longueur de degagement des essais, en metres */
constexpr double kClearance = 0.10;
/** @brief Hauteur a laquelle le toit recouvre la plaque, sous le degagement */
constexpr double kRoofHeight = 0.05;
/** @brief Pas entre deux poses le long de la plaque */
constexpr double kStep = 0.05;
/**
 * @brief Bords du toit, places ENTRE deux poses.
 * @details Une pose posee exactement sous l'arete d'un toit donne une intersection indecidable :
 * le rayon frole le triangle. Les bords sont donc decales d'un quart de pas.
 */
constexpr double kRoofEdgeOffset = kStep / 4.0;

/**
 * @brief Plaque horizontale a z = 0 de x = 0 a 1, recouverte d'un toit a z = kRoofHeight entre
 * x = roof_from et x = roof_to. Les deux sont deux triangles chacun.
 */
pcl::PolygonMesh plateWithRoof(const double roof_from, const double roof_to)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  auto quad = [&cloud](double x0, double x1, double z) {
    const std::uint32_t base = static_cast<std::uint32_t>(cloud.size());
    cloud.push_back(pcl::PointXYZ(x0, 0.0, z));
    cloud.push_back(pcl::PointXYZ(x1, 0.0, z));
    cloud.push_back(pcl::PointXYZ(x1, 0.2, z));
    cloud.push_back(pcl::PointXYZ(x0, 0.2, z));
    return base;
  };
  pcl::PolygonMesh mesh;
  for (const std::uint32_t base : { quad(0.0, 1.0, 0.0), quad(roof_from, roof_to, kRoofHeight) })
  {
    pcl::Vertices first;
    first.vertices = { base, base + 1, base + 2 };
    pcl::Vertices second;
    second.vertices = { base, base + 2, base + 3 };
    mesh.polygons.push_back(first);
    mesh.polygons.push_back(second);
  }
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  return mesh;
}

/**
 * @brief Plaque horizontale a z = 0 de x = 0 a 1, avec une paroi verticale mince (deux triangles,
 * sans epaisseur) plantee a x = wall_x, de z = 0 a z = wall_height. Aucun toit : seul un rayon
 * radial peut la rencontrer.
 */
pcl::PolygonMesh plateWithWall(const double wall_x, const double wall_height)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  cloud.push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
  cloud.push_back(pcl::PointXYZ(1.0, 0.2, 0.0));
  cloud.push_back(pcl::PointXYZ(0.0, 0.2, 0.0));
  cloud.push_back(pcl::PointXYZ(wall_x, 0.0, 0.0));
  cloud.push_back(pcl::PointXYZ(wall_x, 0.2, 0.0));
  cloud.push_back(pcl::PointXYZ(wall_x, 0.2, wall_height));
  cloud.push_back(pcl::PointXYZ(wall_x, 0.0, wall_height));
  pcl::PolygonMesh mesh;
  for (const std::uint32_t base : { 0u, 4u })
  {
    pcl::Vertices first;
    first.vertices = { base, base + 1, base + 2 };
    pcl::Vertices second;
    second.vertices = { base, base + 2, base + 3 };
    mesh.polygons.push_back(first);
    mesh.polygons.push_back(second);
  }
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  return mesh;
}

/** @brief Un segment de poses sur la plaque, z de la pose vers le haut, de x = kStep a x = 1 - kStep */
ToolPaths pathAlongPlate()
{
  ToolPathSegment segment;
  for (double x = kStep; x < 1.0 - kStep / 2.0; x += kStep)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, 0.1, 0.0);
    segment.push_back(pose);
  }
  return { ToolPath{ segment } };
}

/** @brief Abscisses des poses d'un chemin, dans l'ordre, pour lire un resultat d'un coup d'oeil */
std::vector<double> abscissae(const ToolPaths& tool_paths)
{
  std::vector<double> values;
  for (const ToolPath& tool_path : tool_paths)
  {
    for (const ToolPathSegment& segment : tool_path)
    {
      for (const Eigen::Isometry3d& pose : segment)
      {
        values.push_back(pose.translation().x());
      }
    }
  }
  return values;
}
}  // namespace

TEST(OcclusionModifier, RefusesSettingsThatMeanNothing)
{
  const pcl::PolygonMesh mesh = plateWithRoof(0.5, 1.0);
  EXPECT_THROW(OcclusionModifier(mesh, 0.0, 0.0, 8, 1), std::invalid_argument);
  EXPECT_THROW(OcclusionModifier(mesh, kClearance, -0.01, 8, 1), std::invalid_argument);
  EXPECT_THROW(OcclusionModifier(mesh, kClearance, 0.05, 2, 1), std::invalid_argument);
  EXPECT_THROW(OcclusionModifier(mesh, kClearance, 0.0, 8, 0), std::invalid_argument);
  EXPECT_NO_THROW(OcclusionModifier(mesh, kClearance, 0.0, 0, 1));
}

TEST(OcclusionModifier, LeavesAnUnobstructedPathAlone)
{
  // Le toit est plus haut que le degagement : l'outil passe dessous sans le toucher
  const OcclusionModifier modifier(plateWithRoof(0.5, 1.0), kRoofHeight / 2.0, 0.0, 0, 1);
  const ToolPaths input = pathAlongPlate();
  EXPECT_EQ(abscissae(modifier.modify(input)), abscissae(input));
}

TEST(OcclusionModifier, RemovesThePosesUnderTheRoof)
{
  const OcclusionModifier modifier(plateWithRoof(0.5 - kRoofEdgeOffset, 1.0), kClearance, 0.0, 0, 1);
  const ToolPaths output = modifier.modify(pathAlongPlate());

  ASSERT_EQ(output.size(), 1u);
  ASSERT_EQ(output.front().size(), 1u);
  const std::vector<double> xs = abscissae(output);
  ASSERT_EQ(xs.size(), 9u);
  EXPECT_NEAR(xs.front(), 0.05, 1e-9);
  EXPECT_NEAR(xs.back(), 0.45, 1e-9);
  EXPECT_EQ(countWaypoints(output), 9u);
}

TEST(OcclusionModifier, TheDiscEdgeCountsWhenARadiusIsGiven)
{
  // L'axe des poses a x < 0.5 est libre, mais un disque de 0,1 m de rayon centre en x = 0,45
  // avance sous le toit : ces poses tombent aussi
  const OcclusionModifier modifier(plateWithRoof(0.5 - kRoofEdgeOffset, 1.0), kClearance, 0.10, 16, 1);
  const std::vector<double> xs = abscissae(modifier.modify(pathAlongPlate()));
  ASSERT_FALSE(xs.empty());
  EXPECT_NEAR(xs.back(), 0.35, 1e-9);
}

TEST(OcclusionModifier, AThinWallWithinTheRadiusBlocksTheDisc)
{
  // Paroi a x = 0,625 : sans rayon d'outil, toutes les poses restent (aucun toit). Avec un rayon de
  // 0,1 m, les poses a moins de 0,1 m de la paroi tombent : 0,55 ; 0,6 ; 0,65 ; 0,7. Le chemin est
  // coupe en deux suites.
  const double wall_x = 0.625;
  const OcclusionModifier axis_only(plateWithWall(wall_x, 0.05), kClearance, 0.0, 0, 1);
  EXPECT_EQ(abscissae(axis_only.modify(pathAlongPlate())), abscissae(pathAlongPlate()));

  const OcclusionModifier with_disc(plateWithWall(wall_x, 0.05), kClearance, 0.10, 8, 1);
  const ToolPaths output = with_disc.modify(pathAlongPlate());
  ASSERT_EQ(output.size(), 1u);
  ASSERT_EQ(output.front().size(), 2u);
  for (const double x : abscissae(output))
  {
    EXPECT_GT(std::abs(x - wall_x), 0.10 - 1e-9);
  }
  EXPECT_EQ(countWaypoints(output), countWaypoints(pathAlongPlate()) - 4u);
}

TEST(OcclusionModifier, AWallLowerThanTheSpokeHeightIsNotSeen)
{
  // Les rayons radiaux passent a 10 % du rayon au-dessus de la pose : une paroi plus basse ne
  // les coupe pas. Ce test fixe cette limite pour qu'elle ne bouge pas sans qu'on le sache.
  const OcclusionModifier modifier(plateWithWall(0.625, 0.005), kClearance, 0.10, 8, 1);
  EXPECT_EQ(abscissae(modifier.modify(pathAlongPlate())), abscissae(pathAlongPlate()));
}

TEST(OcclusionModifier, SplitsASegmentAroundAnObstacleInTheMiddle)
{
  const OcclusionModifier modifier(plateWithRoof(0.3 + kRoofEdgeOffset, 0.6 + kRoofEdgeOffset), kClearance, 0.0, 0, 1);
  const ToolPaths output = modifier.modify(pathAlongPlate());

  ASSERT_EQ(output.size(), 1u);
  ASSERT_EQ(output.front().size(), 2u) << "deux suites accessibles, une de chaque cote du toit";
  EXPECT_NEAR(output.front()[0].back().translation().x(), 0.30, 1e-9);
  EXPECT_NEAR(output.front()[1].front().translation().x(), 0.65, 1e-9);
}

TEST(OcclusionModifier, DropsRunsShorterThanTheMinimum)
{
  // Le toit laisse deux poses libres a gauche (x = 0,05 et 0,10) : moins que les trois demandees
  const OcclusionModifier modifier(plateWithRoof(0.125, 0.6 + kRoofEdgeOffset), kClearance, 0.0, 0, 3);
  const ToolPaths output = modifier.modify(pathAlongPlate());

  ASSERT_EQ(output.size(), 1u);
  ASSERT_EQ(output.front().size(), 1u);
  EXPECT_NEAR(output.front().front().front().translation().x(), 0.65, 1e-9);
}

TEST(OcclusionModifier, DropsAPathWithNothingLeft)
{
  const OcclusionModifier modifier(plateWithRoof(0.0, 1.0), kClearance, 0.0, 0, 1);
  EXPECT_TRUE(modifier.modify(pathAlongPlate()).empty());
  EXPECT_TRUE(modifier.modify(ToolPaths()).empty());
}

TEST(OcclusionModifier, ReadsTheToolAxisFromThePose)
{
  // Meme plaque et meme toit, mais des poses dont l'axe outil pointe vers -z : le toit est alors
  // derriere l'outil, pas devant, et rien ne doit etre retire
  const OcclusionModifier modifier(plateWithRoof(0.5, 1.0), kClearance, 0.0, 0, 1);
  ToolPaths flipped = pathAlongPlate();
  for (Eigen::Isometry3d& pose : flipped.front().front())
  {
    pose.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  EXPECT_EQ(abscissae(modifier.modify(flipped)), abscissae(flipped));
}

TEST(OcclusionModifier, IsIdempotent)
{
  const OcclusionModifier modifier(plateWithRoof(0.3 + kRoofEdgeOffset, 0.6 + kRoofEdgeOffset), kClearance, 0.05, 8, 2);
  const ToolPaths once = modifier.modify(pathAlongPlate());
  EXPECT_EQ(abscissae(modifier.modify(once)), abscissae(once));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
