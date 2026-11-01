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

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <vtkCellArray.h>
#include <vtkCellLocator.h>
#include <vtkIdList.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
/** @brief Seuil sous lequel une longueur (en metres) ou un sinus est tenu pour nul ; au carre pour une aire */
constexpr double kDegenerate = 1e-9;

/** @brief Point du segment [a, b] le plus proche de `p` */
Eigen::Vector3d closestOnSegment(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double squared = ab.squaredNorm();
  if (squared < kDegenerate * kDegenerate)
  {
    return a;
  }
  return a + ab * std::clamp((p - a).dot(ab) / squared, 0.0, 1.0);
}

/**
 * @brief Point d'un triangle le plus proche de `p`.
 * @details Le projete de `p` sur le plan du triangle s'il tombe dedans, sinon le plus proche des
 * points des trois cotes.
 */
Eigen::Vector3d closestOnTriangle(const Eigen::Vector3d& p, const std::array<Eigen::Vector3d, 3>& t)
{
  const Eigen::Vector3d normal = (t[1] - t[0]).cross(t[2] - t[0]);
  if (normal.squaredNorm() > kDegenerate * kDegenerate)
  {
    const Eigen::Vector3d projected = p - normal * ((p - t[0]).dot(normal) / normal.squaredNorm());
    // Dedans quand le projete est du meme cote des trois aretes
    bool inside = true;
    for (int c = 0; c < 3; ++c)
    {
      inside = inside && (t[(c + 1) % 3] - t[c]).cross(projected - t[c]).dot(normal) >= 0.0;
    }
    if (inside)
    {
      return projected;
    }
  }
  Eigen::Vector3d best = closestOnSegment(p, t[0], t[1]);
  for (const Eigen::Vector3d& candidate : { closestOnSegment(p, t[1], t[2]), closestOnSegment(p, t[2], t[0]) })
  {
    best = ((candidate - p).squaredNorm() < (best - p).squaredNorm()) ? candidate : best;
  }
  return best;
}

/**
 * @brief Positions des sommets d'un polygone du maillage
 * @throws std::invalid_argument si un sommet est absent du nuage
 */
std::vector<Eigen::Vector3d> polygonCorners(const pcl::PointCloud<pcl::PointXYZ>& cloud, const pcl::Indices& indices)
{
  std::vector<Eigen::Vector3d> corners;
  for (const auto index : indices)
  {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud.size())
    {
      throw std::invalid_argument("Sommet " + std::to_string(index) + " absent du maillage");
    }
    corners.push_back(cloud.points[static_cast<std::size_t>(index)].getVector3fMap().cast<double>());
  }
  return corners;
}
}  // namespace

namespace noether
{
PassSurface::PassSurface(const pcl::PolygonMesh& mesh, const std::vector<int>& faces)
{
  if (faces.empty())
  {
    throw std::invalid_argument("Une surface de passe a au moins une face");
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  for (const int face : faces)
  {
    if (face < 0 || static_cast<std::size_t>(face) >= mesh.polygons.size())
    {
      throw std::invalid_argument("Face " + std::to_string(face) + " absente du maillage");
    }
    addPolygon(polygonCorners(cloud, mesh.polygons[static_cast<std::size_t>(face)].vertices));
  }
  if (triangles_.empty())
  {
    throw std::invalid_argument("Une surface de passe a au moins un triangle");
  }
  buildLocator();
}

void PassSurface::addPolygon(const std::vector<Eigen::Vector3d>& corners)
{
  // Un polygone de n sommets donne n - 2 triangles, tous issus du premier sommet
  for (std::size_t k = 1; k + 1 < corners.size(); ++k)
  {
    addTriangle({ corners[0], corners[k], corners[k + 1] });
  }
}

void PassSurface::addTriangle(const std::array<Eigen::Vector3d, 3>& triangle)
{
  const Eigen::Vector3d cross = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);
  // Un triangle sans aire n'a pas de normale : la normale nulle ne passe jamais le test de cote
  normals_.push_back(cross.squaredNorm() > kDegenerate * kDegenerate ? cross.normalized() : Eigen::Vector3d::Zero());
  triangles_.push_back(triangle);
  for (const Eigen::Vector3d& corner : triangle)
  {
    bounds_.extend(corner);
  }
}

void PassSurface::buildLocator()
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto cells = vtkSmartPointer<vtkCellArray>::New();
  for (const std::array<Eigen::Vector3d, 3>& triangle : triangles_)
  {
    vtkIdType cell[3];
    for (int c = 0; c < 3; ++c)
    {
      cell[c] = points->InsertNextPoint(triangle[c].data());
    }
    cells->InsertNextCell(3, cell);
  }
  poly_ = vtkSmartPointer<vtkPolyData>::New();
  poly_->SetPoints(points);
  poly_->SetPolys(cells);
  locator_ = vtkSmartPointer<vtkCellLocator>::New();
  locator_->SetDataSet(poly_);
  locator_->BuildLocator();
}

std::optional<SurfacePoint> PassSurface::place(const Eigen::Vector3d& start,
                                               const Eigen::Vector3d& expected_normal) const
{
  // Une coordonnee non finie ne se compare a rien : la recherche par boite ne finirait pas
  if (!start.allFinite() || !expected_normal.allFinite())
  {
    return std::nullopt;
  }
  // Cas courant : le triangle le plus proche est deja du bon cote
  double closest[3] = { 0.0, 0.0, 0.0 };
  vtkIdType cell = -1;
  int sub_id = 0;
  double squared_distance = 0.0;
  locator_->FindClosestPoint(start.data(), closest, cell, sub_id, squared_distance);
  if (cell < 0)
  {
    return std::nullopt;
  }
  const Eigen::Vector3d& normal = normals_[static_cast<std::size_t>(cell)];
  if (normal.dot(expected_normal) > 0.0)
  {
    return SurfacePoint{ Eigen::Vector3d(closest[0], closest[1], closest[2]), normal };
  }
  return closestFacing(start, expected_normal, std::sqrt(squared_distance));
}

std::optional<SurfacePoint> PassSurface::closest(const Eigen::Vector3d& start) const
{
  if (!start.allFinite())
  {
    return std::nullopt;
  }
  double point[3] = { 0.0, 0.0, 0.0 };
  vtkIdType cell = -1;
  int sub_id = 0;
  double squared_distance = 0.0;
  locator_->FindClosestPoint(start.data(), point, cell, sub_id, squared_distance);
  if (cell < 0)
  {
    return std::nullopt;
  }
  return SurfacePoint{ Eigen::Vector3d(point[0], point[1], point[2]), normals_[static_cast<std::size_t>(cell)] };
}

std::optional<SurfacePoint> PassSurface::closestFacing(const Eigen::Vector3d& start,
                                                       const Eigen::Vector3d& expected_normal,
                                                       const double nearest) const
{
  // Boite doublee a chaque tour. Tout triangle a moins de `radius` coupe la boite : un meilleur
  // trouve a moins de `radius` est donc le plus proche de toute la surface
  const Eigen::Vector3d reach = (bounds_.min() - start).cwiseAbs().cwiseMax((bounds_.max() - start).cwiseAbs());
  double radius = std::max(nearest, kDegenerate);
  while (true)
  {
    radius *= 2.0;
    double best_squared = 0.0;
    const std::optional<SurfacePoint> best = closestFacingWithin(start, expected_normal, radius, best_squared);
    // Une boite qui contient toute la surface a tout vu : son resultat est definitif, meme vide
    if ((best && best_squared <= radius * radius) || radius >= reach.maxCoeff())
    {
      return best;
    }
  }
}

std::optional<SurfacePoint> PassSurface::closestFacingWithin(const Eigen::Vector3d& start,
                                                             const Eigen::Vector3d& expected_normal,
                                                             const double radius,
                                                             double& squared_distance) const
{
  double box[6] = { start.x() - radius, start.x() + radius, start.y() - radius,
                    start.y() + radius, start.z() - radius, start.z() + radius };
  auto candidates = vtkSmartPointer<vtkIdList>::New();
  locator_->FindCellsWithinBounds(box, candidates);
  std::optional<SurfacePoint> best;
  for (vtkIdType k = 0; k < candidates->GetNumberOfIds(); ++k)
  {
    const auto cell = static_cast<std::size_t>(candidates->GetId(k));
    if (normals_[cell].dot(expected_normal) <= 0.0)
    {
      continue;
    }
    const Eigen::Vector3d point = closestOnTriangle(start, triangles_[cell]);
    const double squared = (point - start).squaredNorm();
    if (!best || squared < squared_distance)
    {
      best = SurfacePoint{ point, normals_[cell] };
      squared_distance = squared;
    }
  }
  return best;
}

std::optional<Eigen::Vector3d> interpolateNormal(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                                                 const double fraction)
{
  const double cosine = std::clamp(from.dot(to), -1.0, 1.0);
  const double angle = std::acos(cosine);
  // Deux normales opposees : une infinite de chemins entre elles, aucun ne s'impose
  if (std::sin(angle) < kDegenerate)
  {
    return cosine > 0.0 ? std::optional<Eigen::Vector3d>(from) : std::nullopt;
  }
  const Eigen::Vector3d blended =
      (std::sin((1.0 - fraction) * angle) * from + std::sin(fraction * angle) * to) / std::sin(angle);
  return blended.normalized();
}

std::optional<ToolPathSegment> densifyGap(const PassSurface& surface,
                                          const Eigen::Isometry3d& from,
                                          const Eigen::Isometry3d& to,
                                          const int count)
{
  ToolPathSegment poses;
  const Eigen::Vector3d travel = to.translation() - from.translation();
  for (int i = 1; i <= count; ++i)
  {
    const double fraction = static_cast<double>(i) / (count + 1);
    const std::optional<Eigen::Vector3d> expected =
        interpolateNormal(from.linear().col(2), to.linear().col(2), fraction);
    if (!expected)
    {
      return std::nullopt;
    }
    const std::optional<SurfacePoint> placed = surface.place(from.translation() + travel * fraction, *expected);
    // Un seul point sans place fait sauter l'ecart : pas d'ecart a moitie densifie
    if (!placed)
    {
      return std::nullopt;
    }
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = traceFrame(travel, placed->normal);
    pose.translation() = placed->position;
    poses.push_back(pose);
  }
  return poses;
}

}  // namespace noether
