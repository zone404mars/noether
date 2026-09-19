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
#include <noether_tpp/core/face_coverage.h>

#include <gtest/gtest.h>

using namespace noether;

namespace
{
/** @brief Rayon du disque des essais, en metres : le Mirka de 125 mm */
constexpr double kRadius = 0.0625;

/**
 * @brief Centroides d'une plaque : `rows` rangees de `columns` faces, au pas de 50 mm, la
 * rangee r a y = 0,05 r. La face (r, c) porte l'indice r * columns + c.
 */
std::vector<Eigen::Vector3d> plateCentroids(std::size_t rows, std::size_t columns)
{
  std::vector<Eigen::Vector3d> centroids;
  for (std::size_t r = 0; r < rows; ++r)
  {
    for (std::size_t c = 0; c < columns; ++c)
    {
      centroids.emplace_back(0.05 * static_cast<double>(c), 0.05 * static_cast<double>(r), 0.0);
    }
  }
  return centroids;
}

/** @brief Toutes les faces d'une plaque */
std::vector<int> allFaces(std::size_t count)
{
  std::vector<int> faces;
  for (std::size_t i = 0; i < count; ++i)
  {
    faces.push_back(static_cast<int>(i));
  }
  return faces;
}

/** @brief Une passe droite a y constant, de x = 0 a x = 0,45, avec une approche et un retrait */
ToolPathSegment passAlongX(const double y)
{
  ToolPathSegment pass;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.0, y, 0.10);
  pass.push_back(pose);
  for (double x = 0.0; x <= 0.451; x += 0.05)
  {
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    pass.push_back(pose);
  }
  pose.translation() = Eigen::Vector3d(0.45, y, 0.10);
  pass.push_back(pose);
  return pass;
}
}  // namespace

TEST(FaceCoverage, OnePassCoversTheRowsWithinTheRadius)
{
  // Plaque de 5 rangees a y = 0, 50, 100, 150, 200 mm ; passe a y = 100 mm, rayon 62,5 mm :
  // les rangees 50, 100 et 150 sont couvertes, 0 et 200 (a 100 mm) ne le sont pas
  const std::vector<Eigen::Vector3d> centroids = plateCentroids(5, 10);
  const CoverageResult result = computeCoverage({ passAlongX(0.10) }, centroids, allFaces(50), kRadius, 1);

  EXPECT_EQ(result.selected_count, 50u);
  EXPECT_EQ(result.uncovered_faces.size(), 20u);
  EXPECT_NEAR(result.ratio, 0.6, 1e-9);
  EXPECT_EQ(result.uncovered_faces.front(), 0);
  EXPECT_EQ(result.uncovered_faces.back(), 49);
}

TEST(FaceCoverage, TwoPassesAtTheDiscDiameterCoverEverything)
{
  const std::vector<Eigen::Vector3d> centroids = plateCentroids(5, 10);
  const CoverageResult result =
      computeCoverage({ passAlongX(0.05), passAlongX(0.15) }, centroids, allFaces(50), kRadius, 1);
  EXPECT_TRUE(result.uncovered_faces.empty());
  EXPECT_DOUBLE_EQ(result.ratio, 1.0);
}

TEST(FaceCoverage, TheApproachAndDepartureDoNotCount)
{
  // Une seule face, sous la pose d'approche mais a 100 mm au-dessus : avec les bouts ecartes elle
  // n'est pas couverte, sans les ecarter le troncon d'approche la frole et la compte
  std::vector<Eigen::Vector3d> centroids;
  centroids.emplace_back(0.0, 0.10, 0.08);
  const std::vector<int> face(1, 0);
  EXPECT_EQ(computeCoverage({ passAlongX(0.10) }, centroids, face, kRadius, 1).uncovered_faces.size(), 1u);
  EXPECT_TRUE(computeCoverage({ passAlongX(0.10) }, centroids, face, kRadius, 0).uncovered_faces.empty());
}

TEST(FaceCoverage, AnEmptySelectionIsFullyCovered)
{
  const CoverageResult result = computeCoverage({ passAlongX(0.10) }, plateCentroids(1, 1), {}, kRadius, 1);
  EXPECT_EQ(result.selected_count, 0u);
  EXPECT_DOUBLE_EQ(result.ratio, 1.0);
}

TEST(FaceCoverage, NoPassCoversNothing)
{
  const CoverageResult result = computeCoverage(ToolPath(), plateCentroids(2, 2), allFaces(4), kRadius, 1);
  EXPECT_EQ(result.uncovered_faces.size(), 4u);
  EXPECT_DOUBLE_EQ(result.ratio, 0.0);
}

TEST(FaceCoverage, RefusesWhatItCannotMeasure)
{
  const std::vector<Eigen::Vector3d> centroids = plateCentroids(1, 1);
  EXPECT_THROW(computeCoverage({ passAlongX(0.0) }, centroids, allFaces(1), 0.0, 1), std::invalid_argument);
  EXPECT_THROW(computeCoverage({ passAlongX(0.0) }, centroids, std::vector<int>(1, 7), kRadius, 1),
               std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
