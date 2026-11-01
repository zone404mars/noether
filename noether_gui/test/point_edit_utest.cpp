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
#include <noether_gui/widgets/point_edit.h>
#include <noether_gui/widgets/point_picking.h>
#include <noether_gui/widgets/trace_tool.h>

#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <vtkCellArray.h>
#include <vtkCellLocator.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

using namespace noether;

namespace
{
/** @brief Hauteur d'approche des passes de test, en metres */
constexpr double kHeight = 0.05;

/** @brief Carre de 1 m de cote sur le plan z = `z`, normale `+z` ou `-z`, en deux triangles */
pcl::PolygonMesh square(const double z, const bool upward)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto& corner : { std::make_pair(-0.5, -0.5), std::make_pair(0.5, -0.5), std::make_pair(0.5, 0.5),
                              std::make_pair(-0.5, 0.5) })
  {
    cloud.push_back(pcl::PointXYZ(static_cast<float>(corner.first), static_cast<float>(corner.second),
                                  static_cast<float>(z)));
  }
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  pcl::Vertices first;
  pcl::Vertices second;
  first.vertices = upward ? pcl::Indices{ 0, 1, 2 } : pcl::Indices{ 0, 2, 1 };
  second.vertices = upward ? pcl::Indices{ 0, 2, 3 } : pcl::Indices{ 0, 3, 2 };
  mesh.polygons = { first, second };
  return mesh;
}

/** @brief Passe droite sur le plan z = 0, poses de contact aux abscisses donnees, surface = le plan */
EditedPass planePass(const std::vector<double>& xs, const bool upward_surface = true)
{
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
  for (const double x : xs)
  {
    points.emplace_back(x, 0.0, 0.0);
    normals.push_back(Eigen::Vector3d::UnitZ());
  }
  const ToolPathSegment contact = tracePoses(points, normals);
  return EditedPass{ withApproachAndRetract(contact, kHeight),
                     std::make_shared<const PassSurface>(square(0.0, upward_surface), std::vector<int>{ 0, 1 }) };
}

/** @brief Abscisses des poses de contact d'une passe */
std::vector<double> contactXs(const ToolPathSegment& poses)
{
  std::vector<double> xs;
  for (std::size_t i = 1; i + 1 < poses.size(); ++i)
  {
    xs.push_back(poses[i].translation().x());
  }
  return xs;
}

/** @brief Compare deux listes d'abscisses au micrometre */
void expectXs(const std::vector<double>& actual, const std::vector<double>& expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i)
  {
    EXPECT_NEAR(actual[i], expected[i], 1e-6);
  }
}

/** @brief Reglages de Densifier en mode Nombre */
DensifySettings byCount(const int count) { return DensifySettings{ DensifyMode::Count, 0.01, count }; }
}  // namespace

TEST(PointSelection, AClickSelectsOnlyThePointAndAClickInTheVoidClears)
{
  PointSelection selection;
  selection.click(PoseRef{ 0, 2 }, false);
  selection.click(PoseRef{ 0, 3 }, false);
  ASSERT_EQ(selection.points().size(), 1u);
  EXPECT_TRUE(selection.contains(PoseRef{ 0, 3 }));
  selection.click(std::nullopt, false);
  EXPECT_TRUE(selection.empty());
}

TEST(PointSelection, CtrlClickTogglesAndCtrlClickInTheVoidKeeps)
{
  PointSelection selection;
  selection.click(PoseRef{ 0, 2 }, false);
  selection.click(PoseRef{ 1, 4 }, true);
  EXPECT_EQ(selection.points().size(), 2u);
  selection.click(PoseRef{ 0, 2 }, true);
  ASSERT_EQ(selection.points().size(), 1u);
  EXPECT_TRUE(selection.contains(PoseRef{ 1, 4 }));
  selection.click(std::nullopt, true);
  EXPECT_EQ(selection.points().size(), 1u);
}

TEST(PointSelection, DraggingAnUnselectedPointMakesItTheSelection)
{
  PointSelection selection;
  selection.set({ PoseRef{ 0, 2 }, PoseRef{ 0, 3 } });
  selection.beginDrag(PoseRef{ 0, 3 });
  EXPECT_EQ(selection.points().size(), 2u);
  selection.beginDrag(PoseRef{ 0, 5 });
  ASSERT_EQ(selection.points().size(), 1u);
  EXPECT_TRUE(selection.contains(PoseRef{ 0, 5 }));
}

TEST(PointSelection, DensifyNeedsExactlyTwoPosesOfOnePassInPassOrder)
{
  PointSelection selection;
  selection.set({ PoseRef{ 0, 5 }, PoseRef{ 0, 2 } });
  ASSERT_TRUE(selection.densifyEnds().has_value());
  EXPECT_EQ(selection.densifyEnds()->first.index, 2u);
  EXPECT_EQ(selection.densifyEnds()->second.index, 5u);
  selection.set({ PoseRef{ 0, 2 }, PoseRef{ 1, 3 } });
  EXPECT_FALSE(selection.densifyEnds().has_value());
  selection.set({ PoseRef{ 0, 2 }, PoseRef{ 0, 3 }, PoseRef{ 0, 4 } });
  EXPECT_FALSE(selection.densifyEnds().has_value());
}

TEST(Densify, CountModeSpreadsNPointsEvenly)
{
  // Critere §25 : N = 3, ecart droit de 40 mm sur un plan -> 3 points espaces de 10 mm
  const std::vector<EditedPass> passes = { planePass({ 0.0, 0.04 }) };
  const DensifyPlan plan = planDensify(passes, PoseRef{ 0, 1 }, PoseRef{ 0, 2 }, byCount(3));
  ASSERT_EQ(plan.insertions.size(), 1u);
  expectXs({ plan.insertions[0].poses[0].translation().x(), plan.insertions[0].poses[1].translation().x(),
             plan.insertions[0].poses[2].translation().x() },
           { 0.01, 0.02, 0.03 });
}

TEST(Densify, CountModeFillsEveryGapFromAToB)
{
  // Critere §25 : N = 2, trois ecarts de A a B -> 6 points
  EditedPath path(4);
  path.reset({ planePass({ 0.0, 0.03, 0.06, 0.09 }) });
  const DensifyPlan plan = planDensify(path.passes(), PoseRef{ 0, 1 }, PoseRef{ 0, 4 }, byCount(2));
  path.insertPoses(plan.insertions);
  EXPECT_EQ(contactXs(path.passes()[0].poses).size(), 10u);
}

TEST(Densify, StepModeKeepsTheV1Rule)
{
  // Critere §25 : ecart de 20 mm, pas de 7 mm -> 2 points
  const std::vector<EditedPass> passes = { planePass({ 0.0, 0.02 }) };
  const DensifySettings settings{ DensifyMode::Step, 0.007, 1 };
  const DensifyPlan plan = planDensify(passes, PoseRef{ 0, 1 }, PoseRef{ 0, 2 }, settings);
  ASSERT_EQ(plan.insertions.size(), 1u);
  EXPECT_EQ(plan.insertions[0].poses.size(), 2u);
  EXPECT_EQ(gapPointCount(0.02, DensifySettings{ DensifyMode::Step, 0.03, 1 }), 0);
}

TEST(Densify, DensifyingTwiceCutsTheGapsAsTheyHaveBecome)
{
  // Critere §25 : deux fois N = 1 sur un ecart -> 3 points dans l'ecart d'origine
  EditedPath path(4);
  path.reset({ planePass({ 0.0, 0.04 }) });
  PoseRef a{ 0, 1 };
  PoseRef b{ 0, 2 };
  for (int round = 0; round < 2; ++round)
  {
    const DensifyPlan plan = planDensify(path.passes(), a, b, byCount(1));
    path.insertPoses(plan.insertions);
    // A et B restent choisies : B a recule du nombre de poses inserees avant elle
    b = shiftedByInsertions(b, plan.insertions);
  }
  expectXs(contactXs(path.passes()[0].poses), { 0.0, 0.01, 0.02, 0.03, 0.04 });
  EXPECT_EQ(b.index, 5u);
}

TEST(Densify, RefusesAPassWithoutSurface)
{
  std::vector<EditedPass> passes = { planePass({ 0.0, 0.04 }) };
  passes[0].surface = nullptr;
  EXPECT_THROW(planDensify(passes, PoseRef{ 0, 1 }, PoseRef{ 0, 2 }, byCount(1)), std::invalid_argument);
}

TEST(Drag, EverySelectedPoseMovesByTheSameVectorOnTheSurface)
{
  const EditedPass pass = planePass({ 0.0, 0.01, 0.02, 0.03, 0.04 });
  const Eigen::Vector3d delta(0.0, 0.02, 0.0);
  const ToolPathSegment moved = draggedPass(pass, { 2, 3, 4 }, delta);
  for (const std::size_t i : { 2u, 3u, 4u })
  {
    EXPECT_TRUE(moved[i].translation().isApprox(pass.poses[i].translation() + delta));
    EXPECT_TRUE(moved[i].linear().col(2).isApprox(Eigen::Vector3d::UnitZ()));
  }
  EXPECT_TRUE(moved[1].isApprox(pass.poses[1]));
}

TEST(Drag, TheApproachFollowsTheFirstContactAtTheSamePlaceInItsFrame)
{
  const EditedPass pass = planePass({ 0.0, 0.01, 0.02 });
  const ToolPathSegment moved = draggedPass(pass, { 1 }, Eigen::Vector3d(0.0, 0.03, 0.0));
  EXPECT_TRUE((moved[1].inverse() * moved[0]).isApprox(pass.poses[1].inverse() * pass.poses[0]));
  EXPECT_NEAR(moved[0].translation().y(), 0.03, 1e-9);
  EXPECT_TRUE(moved.back().isApprox(pass.poses.back()));
}

TEST(Drag, APoseWithoutAValidTriangleStaysWhereItWas)
{
  // Surface tournee vers -z, poses tournees vers +z : aucun triangle du bon cote
  const EditedPass pass = planePass({ 0.0, 0.01, 0.02 }, false);
  const ToolPathSegment moved = draggedPass(pass, { 2 }, Eigen::Vector3d(0.0, 0.03, 0.0));
  EXPECT_TRUE(moved[2].isApprox(pass.poses[2]));
}

TEST(Drag, APassWithoutSurfaceIsRefused)
{
  EditedPass pass = planePass({ 0.0, 0.01, 0.02 });
  pass.surface = nullptr;
  EXPECT_THROW(draggedPass(pass, { 1 }, Eigen::Vector3d::Zero()), std::invalid_argument);
  EXPECT_THROW(draggedPass(planePass({ 0.0, 0.01 }), { 0 }, Eigen::Vector3d::Zero()), std::invalid_argument);
}

TEST(Picking, TakesTheNearestVisiblePointWithinTheRadius)
{
  const std::vector<Eigen::Vector2d> screen = { Eigen::Vector2d(100, 100), Eigen::Vector2d(108, 100),
                                                Eigen::Vector2d(103, 100) };
  // Critere §25 : a 5 pixels -> saisi ; a 15 pixels de tout point -> rien
  EXPECT_EQ(nearestOnScreen({ screen[0] }, { true }, Eigen::Vector2d(105, 100), 10.0), std::optional<std::size_t>(0));
  EXPECT_FALSE(nearestOnScreen({ screen[0] }, { true }, Eigen::Vector2d(115, 100), 10.0).has_value());
  // Deux points a moins de 10 pixels : le plus proche a l'ecran
  EXPECT_EQ(nearestOnScreen(screen, { true, true, true }, Eigen::Vector2d(104.5, 100), 10.0),
            std::optional<std::size_t>(2));
  // Un point cache ne se saisit pas : le suivant le plus proche l'est
  EXPECT_EQ(nearestOnScreen(screen, { true, true, false }, Eigen::Vector2d(104.5, 100), 10.0),
            std::optional<std::size_t>(1));
}

TEST(Picking, APointHiddenByThePartIsNotVisible)
{
  // Critere §25 : le segment camera -> pose rencontre la piece 5 mm avant elle -> cache
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto cells = vtkSmartPointer<vtkCellArray>::New();
  const vtkIdType a = points->InsertNextPoint(-1.0, -1.0, 0.005);
  const vtkIdType b = points->InsertNextPoint(1.0, -1.0, 0.005);
  const vtkIdType c = points->InsertNextPoint(0.0, 1.0, 0.005);
  const vtkIdType triangle[3] = { a, b, c };
  cells->InsertNextCell(3, triangle);
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetPolys(cells);
  auto locator = vtkSmartPointer<vtkCellLocator>::New();
  locator->SetDataSet(poly);
  locator->BuildLocator();
  const Eigen::Vector3d eye(0.0, 0.0, 1.0);
  EXPECT_FALSE(visibleFrom(*locator, eye, Eigen::Vector3d::Zero(), 0.001));
  // Le point pose sur la piece elle-meme : son triangle est dans la marge
  EXPECT_TRUE(visibleFrom(*locator, eye, Eigen::Vector3d(0.0, 0.0, 0.005), 0.001));
}
