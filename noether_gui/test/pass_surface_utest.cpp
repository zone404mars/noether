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
#include <noether_gui/widgets/pass_surface.h>
#include <noether_gui/widgets/trace_tool.h>

#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

using namespace noether;

namespace
{
/** @brief Rayon interieur du cintrage, en metres (spec §25) */
constexpr double kInnerRadius = 0.020;
/** @brief Epaisseur de la tole, en metres */
constexpr double kThickness = 0.003;
/** @brief Rayon de la peau exterieure */
constexpr double kOuterRadius = kInnerRadius + kThickness;
/** @brief Largeur de la tole le long de l'axe du cintrage (y) */
constexpr double kWidth = 0.040;
/** @brief Nombre de facettes sur le quart de cercle */
constexpr int kSteps = 45;

/**
 * @brief Tole cintree d'un quart de tour autour de l'axe y, fermee : peau exterieure, peau
 * interieure, et la tranche aux deux bouts du quart de tour.
 * @details Faces 0 a 2 * kSteps - 1 : peau exterieure ; puis autant pour la peau interieure ;
 * puis 4 faces de tranche. Les normales pointent hors de la matiere.
 */
struct BentSheet
{
  pcl::PolygonMesh mesh;
  std::vector<int> outer;
  std::vector<int> inner;
  std::vector<int> all;
};

/** @brief Point d'une peau de rayon `radius`, a l'angle `angle` et a la position `y` */
Eigen::Vector3d onSkin(const double radius, const double angle, const double y)
{
  return Eigen::Vector3d(radius * std::cos(angle), y, radius * std::sin(angle));
}

/** @brief Ajoute un quadrilatere en deux triangles, sommets dans le sens qui donne la normale voulue */
void addQuad(pcl::PolygonMesh& mesh, const int a, const int b, const int c, const int d)
{
  pcl::Vertices first;
  first.vertices = { a, b, c };
  pcl::Vertices second;
  second.vertices = { a, c, d };
  mesh.polygons.push_back(first);
  mesh.polygons.push_back(second);
}

/** @brief Sommets de la tole : pour chaque angle, exterieur y-, exterieur y+, interieur y-, interieur y+ */
pcl::PointCloud<pcl::PointXYZ> sheetVertices()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int k = 0; k <= kSteps; ++k)
  {
    const double angle = (M_PI / 2.0) * k / kSteps;
    for (const double radius : { kOuterRadius, kInnerRadius })
    {
      for (const double y : { -kWidth / 2.0, kWidth / 2.0 })
      {
        const Eigen::Vector3d p = onSkin(radius, angle, y);
        cloud.push_back(pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
      }
    }
  }
  return cloud;
}

/** @brief La tole cintree complete, avec ses listes de faces par peau */
/** @brief Range les faces : peau exterieure, peau interieure, et toutes */
void sortSheetFaces(BentSheet& sheet)
{
  for (int f = 0; f < static_cast<int>(sheet.mesh.polygons.size()); ++f)
  {
    if (f < 2 * kSteps)
    {
      sheet.outer.push_back(f);
    }
    else if (f < 4 * kSteps)
    {
      sheet.inner.push_back(f);
    }
    sheet.all.push_back(f);
  }
}

BentSheet makeBentSheet()
{
  BentSheet sheet;
  pcl::toPCLPointCloud2(sheetVertices(), sheet.mesh.cloud);
  const auto id = [](int k, int corner) { return 4 * k + corner; };
  for (int k = 0; k < kSteps; ++k)
  {
    // Exterieur : normale radiale vers l'exterieur
    addQuad(sheet.mesh, id(k, 0), id(k, 1), id(k + 1, 1), id(k + 1, 0));
  }
  for (int k = 0; k < kSteps; ++k)
  {
    // Interieur : normale vers l'axe
    addQuad(sheet.mesh, id(k, 2), id(k + 1, 2), id(k + 1, 3), id(k, 3));
  }
  // Tranche aux deux bouts du quart de tour
  addQuad(sheet.mesh, id(0, 0), id(0, 2), id(0, 3), id(0, 1));
  addQuad(sheet.mesh, id(kSteps, 0), id(kSteps, 1), id(kSteps, 3), id(kSteps, 2));
  sortSheetFaces(sheet);
  return sheet;
}

/** @brief Pose de contact sur la peau exterieure, z vers l'exterieur, x dans le sens des angles croissants */
Eigen::Isometry3d outerPose(const double angle)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d radial(std::cos(angle), 0.0, std::sin(angle));
  pose.linear() = traceFrame(Eigen::Vector3d(-std::sin(angle), 0.0, std::cos(angle)), radial);
  pose.translation() = onSkin(kOuterRadius, angle, 0.0);
  return pose;
}

/** @brief Distance d'un point a l'axe du cintrage */
double radiusOf(const Eigen::Vector3d& point) { return std::hypot(point.x(), point.z()); }

/** @brief Angle qui separe deux poses de la peau exterieure distantes de `chord` en ligne droite */
double angleForChord(const double chord) { return 2.0 * std::asin(chord / (2.0 * kOuterRadius)); }
}  // namespace

TEST(PassSurface, RefusesAnEmptySurfaceOrAMissingFace)
{
  const BentSheet sheet = makeBentSheet();
  EXPECT_THROW(PassSurface(sheet.mesh, {}), std::invalid_argument);
  EXPECT_THROW(PassSurface(sheet.mesh, { static_cast<int>(sheet.mesh.polygons.size()) }), std::invalid_argument);
}

TEST(PassSurface, DensifiesOnTheOuterSkinOfAThinBentSheet)
{
  // Critere §25 : 20 mm entre A et B, la droite passe a 2,3 mm sous la peau exterieure, plus
  // loin que la demi-epaisseur ; le maillage entier aurait donne la peau interieure
  const BentSheet sheet = makeBentSheet();
  const double start = 0.3;
  const double end = start + angleForChord(0.020);
  for (const std::vector<int>* faces : { &sheet.outer, &sheet.all })
  {
    const PassSurface surface(sheet.mesh, *faces);
    const std::optional<ToolPathSegment> added = densifyGap(surface, outerPose(start), outerPose(end), 3);
    ASSERT_TRUE(added.has_value());
    ASSERT_EQ(added->size(), 3u);
    for (const Eigen::Isometry3d& pose : *added)
    {
      EXPECT_NEAR(radiusOf(pose.translation()), kOuterRadius, 1e-4);
      // Normale du cote de A et B : vers l'exterieur du cintrage
      const Eigen::Vector3d radial = Eigen::Vector3d(pose.translation().x(), 0.0, pose.translation().z()).normalized();
      EXPECT_GT(pose.linear().col(2).dot(radial), 0.0);
    }
  }
}

TEST(PassSurface, TheSideTestIsWhatKeepsTheOuterSkin)
{
  // Meme point de depart, normale attendue retournee : la peau interieure est retenue. Le test de
  // cote decide donc bien de la peau quand la surface porte les deux
  const BentSheet sheet = makeBentSheet();
  const PassSurface surface(sheet.mesh, sheet.all);
  const Eigen::Vector3d below_outer_skin = onSkin(kOuterRadius - 0.0023, 0.5, 0.0);
  const Eigen::Vector3d radial(std::cos(0.5), 0.0, std::sin(0.5));
  const std::optional<SurfacePoint> outward = surface.place(below_outer_skin, radial);
  const std::optional<SurfacePoint> inward = surface.place(below_outer_skin, -radial);
  ASSERT_TRUE(outward.has_value());
  ASSERT_TRUE(inward.has_value());
  EXPECT_NEAR(radiusOf(outward->position), kOuterRadius, 1e-4);
  EXPECT_NEAR(radiusOf(inward->position), kInnerRadius, 1e-4);
}

TEST(PassSurface, AnAddedPoseTakesTheTriangleNormalAndFollowsTheGap)
{
  const BentSheet sheet = makeBentSheet();
  const PassSurface surface(sheet.mesh, sheet.outer);
  const Eigen::Isometry3d from = outerPose(0.4);
  const Eigen::Isometry3d to = outerPose(0.4 + angleForChord(0.020));
  const std::optional<ToolPathSegment> added = densifyGap(surface, from, to, 1);
  ASSERT_TRUE(added.has_value());
  ASSERT_EQ(added->size(), 1u);
  const Eigen::Matrix3d frame = added->front().linear();
  const Eigen::Vector3d z = frame.col(2);
  const Eigen::Vector3d x = frame.col(0);
  // Normale plane d'une facette de la peau exterieure : radiale a la moitie d'un pas d'angle pres
  const Eigen::Vector3d point = added->front().translation();
  const Eigen::Vector3d radial = Eigen::Vector3d(point.x(), 0.0, point.z()).normalized();
  EXPECT_GT(z.dot(radial), std::cos(M_PI / 2.0 / kSteps));
  EXPECT_NEAR(x.dot(z), 0.0, 1e-9);
  EXPECT_GT(x.dot(to.translation() - from.translation()), 0.0);
}

TEST(PassSurface, SkipsAGapWithoutATriangleOnTheRightSide)
{
  // Surface = peau interieure seule, poses tournees vers l'exterieur : aucun triangle ne convient
  const BentSheet sheet = makeBentSheet();
  const PassSurface surface(sheet.mesh, sheet.inner);
  EXPECT_FALSE(densifyGap(surface, outerPose(0.3), outerPose(0.9), 2).has_value());
}

TEST(PassSurface, SkipsAGapBetweenOpposedPoses)
{
  const BentSheet sheet = makeBentSheet();
  const PassSurface surface(sheet.mesh, sheet.all);
  // La seconde pose est la premiere retournee d'un demi-tour autour de x : son z est oppose
  const Eigen::Isometry3d first = outerPose(0.9);
  Eigen::Isometry3d flipped = first;
  flipped.linear() = first.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_FALSE(densifyGap(surface, first, flipped, 1).has_value());
}

TEST(PassSurface, NoCountGivesNoPose)
{
  const BentSheet sheet = makeBentSheet();
  const PassSurface surface(sheet.mesh, sheet.outer);
  const std::optional<ToolPathSegment> added = densifyGap(surface, outerPose(0.3), outerPose(0.9), 0);
  ASSERT_TRUE(added.has_value());
  EXPECT_TRUE(added->empty());
}

TEST(PassSurface, InterpolatesNormalsOnTheSphere)
{
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(interpolateNormal(x, z, 0.0)->isApprox(x));
  EXPECT_TRUE(interpolateNormal(x, z, 1.0)->isApprox(z));
  EXPECT_TRUE(interpolateNormal(x, z, 0.5)->isApprox(Eigen::Vector3d(1.0, 0.0, 1.0).normalized()));
  EXPECT_TRUE(interpolateNormal(z, z, 0.3)->isApprox(z));
  EXPECT_FALSE(interpolateNormal(z, -z, 0.5).has_value());
}

namespace
{
/** @brief Maillage d'un seul polygone, sommets donnes dans l'ordre */
pcl::PolygonMesh polygonMesh(const std::vector<Eigen::Vector3d>& corners)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::Vertices polygon;
  for (std::size_t k = 0; k < corners.size(); ++k)
  {
    cloud.push_back(pcl::PointXYZ(static_cast<float>(corners[k].x()), static_cast<float>(corners[k].y()),
                                  static_cast<float>(corners[k].z())));
    polygon.vertices.push_back(static_cast<int>(k));
  }
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  mesh.polygons.push_back(polygon);
  return mesh;
}
}  // namespace

TEST(PassSurface, AQuadGivesTwoTriangles)
{
  const pcl::PolygonMesh quad = polygonMesh({ Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),
                                              Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(0, 1, 0) });
  EXPECT_EQ(PassSurface(quad, { 0 }).size(), 2u);
}

TEST(PassSurface, ANonFiniteStartFindsNothing)
{
  const BentSheet sheet = makeBentSheet();
  const PassSurface surface(sheet.mesh, sheet.all);
  const Eigen::Vector3d nan_start(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  EXPECT_FALSE(surface.place(nan_start, -Eigen::Vector3d::UnitX()).has_value());
}

TEST(PassSurface, OneUnplaceablePointSkipsTheWholeGap)
{
  // Plan de normale +x ; la seconde pose tourne son z de 170 deg : les points de la seconde moitie
  // attendent une normale tournee vers -x, aucun triangle ne convient, tout l'ecart saute
  const pcl::PolygonMesh plane = polygonMesh({ Eigen::Vector3d(0, -1, -1), Eigen::Vector3d(0, 1, -1),
                                               Eigen::Vector3d(0, 1, 1), Eigen::Vector3d(0, -1, 1) });
  const PassSurface surface(plane, { 0 });
  Eigen::Isometry3d from = Eigen::Isometry3d::Identity();
  from.linear() = traceFrame(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
  Eigen::Isometry3d to = from;
  to.translation() = Eigen::Vector3d(0.0, 0.0, 0.1);
  to.linear() = Eigen::AngleAxisd(170.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix() * from.linear();
  EXPECT_FALSE(densifyGap(surface, from, to, 3).has_value());
}

namespace
{
/** @brief Sommets du polygone `face` d'un maillage, en double */
std::array<Eigen::Vector3d, 3> triangleOf(const pcl::PolygonMesh& mesh, const std::size_t face)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  std::array<Eigen::Vector3d, 3> triangle;
  for (int c = 0; c < 3; ++c)
  {
    const auto vertex = static_cast<std::size_t>(mesh.polygons[face].vertices[c]);
    triangle[c] = cloud.points[vertex].getVector3fMap().cast<double>();
  }
  return triangle;
}

/** @brief `count` triangles de 0,1 m au plus, poses au hasard dans le cube [-1, 1]^3 */
pcl::PolygonMesh randomTriangles(std::mt19937& random, const int count)
{
  std::uniform_real_distribution<double> coordinate(-1.0, 1.0);
  const auto draw = [&]() { return Eigen::Vector3d(coordinate(random), coordinate(random), coordinate(random)); };
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PolygonMesh mesh;
  for (int f = 0; f < count; ++f)
  {
    const Eigen::Vector3d center = draw();
    pcl::Vertices polygon;
    const Eigen::Vector3d second = center + 0.1 * draw();
    const Eigen::Vector3d third = center + 0.1 * draw();
    for (const Eigen::Vector3d& corner : { center, second, third })
    {
      polygon.vertices.push_back(static_cast<int>(cloud.size()));
      cloud.push_back(pcl::PointXYZ(static_cast<float>(corner.x()), static_cast<float>(corner.y()),
                                    static_cast<float>(corner.z())));
    }
    mesh.polygons.push_back(polygon);
  }
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  return mesh;
}

/** @brief Distance au triangle du bon cote le plus proche, en examinant chaque triangle seul */
double fullScanDistance(const pcl::PolygonMesh& mesh, const Eigen::Vector3d& start, const Eigen::Vector3d& expected)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t f = 0; f < mesh.polygons.size(); ++f)
  {
    const std::array<Eigen::Vector3d, 3> t = triangleOf(mesh, f);
    if ((t[1] - t[0]).cross(t[2] - t[0]).dot(expected) <= 0.0)
    {
      continue;
    }
    const PassSurface single(polygonMesh({ t[0], t[1], t[2] }), { 0 });
    best = std::min(best, (single.place(start, expected)->position - start).norm());
  }
  return best;
}
}  // namespace

TEST(PassSurface, TheGrowingBoxFindsWhatAFullScanFinds)
{
  // La recherche par boite doublee doit rendre la meme distance que l'examen de chaque triangle
  std::mt19937 random(7);
  const pcl::PolygonMesh mesh = randomTriangles(random, 200);
  std::vector<int> faces(mesh.polygons.size());
  std::iota(faces.begin(), faces.end(), 0);
  const PassSurface surface(mesh, faces);
  std::uniform_real_distribution<double> coordinate(-1.0, 1.0);
  for (int q = 0; q < 300; ++q)
  {
    const Eigen::Vector3d start(coordinate(random), coordinate(random), coordinate(random));
    const Eigen::Vector3d expected =
        Eigen::Vector3d(coordinate(random), coordinate(random), coordinate(random)).normalized();
    const std::optional<SurfacePoint> found = surface.place(start, expected);
    const double best = fullScanDistance(mesh, start, expected);
    ASSERT_EQ(found.has_value(), std::isfinite(best));
    if (found)
    {
      EXPECT_NEAR((found->position - start).norm(), best, 1e-6);
    }
  }
}
