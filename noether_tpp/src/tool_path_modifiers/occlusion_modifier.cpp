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

#include <pcl/io/vtk_lib_io.h>

#include <cmath>
#include <stdexcept>

namespace
{
/**
 * @brief Part du degagement, au ras de la surface, ou une rencontre n'est pas comptee.
 * @details La pose est SUR la surface : un rayon parti exactement de la touche aussitot. Cette
 * bande l'evite ; ce qui est touche au-dela est un vrai recouvrement. Le bord du disque, lui,
 * est deja decolle de kSpokeLiftFraction du rayon (voir rayOrigins).
 */
constexpr double kLiftOffFraction = 0.02;

/** @brief Tolerance passee au localisateur VTK pour l'intersection rayon / triangle, en metres */
constexpr double kIntersectionTolerance = 1e-9;

/**
 * @brief Hauteur des rayons du bord et des rayons radiaux au-dessus de la pose, en fraction du
 * rayon d'outil.
 * @details Sur une surface courbe, le bord du disque n'est pas a la hauteur de son centre : un
 * rayon parti du bord a la hauteur du centre traverserait la surface elle-meme. Le decollement
 * suit le rayon d'outil, pour qu'une courbure de quelques dixiemes de metre passe, tout en
 * laissant voir une marche de la hauteur d'une tole.
 */
constexpr double kSpokeLiftFraction = 0.1;
}  // namespace

namespace noether
{
OcclusionModifier::OcclusionModifier(const pcl::PolygonMesh& mesh,
                                     const double clearance,
                                     const double tool_radius,
                                     const unsigned ring_samples,
                                     const std::size_t min_segment_poses)
  : poly_(vtkSmartPointer<vtkPolyData>::New())
  , locator_(vtkSmartPointer<vtkCellLocator>::New())
  , clearance_(clearance)
  , tool_radius_(tool_radius)
  , ring_samples_(ring_samples)
  , min_segment_poses_(min_segment_poses)
{
  if (!(clearance > 0.0))
  {
    throw std::invalid_argument("Le degagement de l'outil doit etre strictement positif");
  }
  if (tool_radius < 0.0)
  {
    throw std::invalid_argument("Le rayon de l'outil ne peut pas etre negatif");
  }
  if (tool_radius > 0.0 && ring_samples < 3)
  {
    throw std::invalid_argument("Il faut au moins trois rayons sur le bord du disque");
  }
  if (min_segment_poses == 0)
  {
    throw std::invalid_argument("Une suite de poses conservee compte au moins une pose");
  }

  pcl::io::mesh2vtk(mesh, poly_);
  locator_->SetDataSet(poly_);
  locator_->BuildLocator();
}

bool OcclusionModifier::segmentBlocked(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const
{
  double t = 0.0;
  double hit[3] = { 0.0, 0.0, 0.0 };
  double pcoords[3] = { 0.0, 0.0, 0.0 };
  int sub_id = 0;
  vtkIdType cell_id = -1;
  Eigen::Vector3d p1 = from;
  Eigen::Vector3d p2 = to;
  const int hit_count =
      locator_->IntersectWithLine(p1.data(), p2.data(), kIntersectionTolerance, t, hit, pcoords, sub_id, cell_id);
  return hit_count != 0;
}

bool OcclusionModifier::rayBlocked(const Eigen::Vector3d& start, const Eigen::Vector3d& direction) const
{
  // Le depart est decolle de la surface pour que le rayon ne rencontre pas la face qui porte la pose
  return segmentBlocked(start + direction * (clearance_ * kLiftOffFraction), start + direction * clearance_);
}

bool OcclusionModifier::spokesBlocked(const Eigen::Isometry3d& pose, const std::vector<Eigen::Vector3d>& origins) const
{
  // origins[0] est le centre, les suivants le bord du disque deja decolle : un rayon radial vers
  // chacun, a la meme hauteur
  const Eigen::Vector3d centre = origins.front() + pose.linear().col(2) * (tool_radius_ * kSpokeLiftFraction);
  for (std::size_t k = 1; k < origins.size(); ++k)
  {
    if (segmentBlocked(centre, origins[k]))
    {
      return true;
    }
  }
  return false;
}

std::vector<Eigen::Vector3d> OcclusionModifier::rayOrigins(const Eigen::Isometry3d& pose) const
{
  std::vector<Eigen::Vector3d> origins;
  origins.push_back(pose.translation());
  if (tool_radius_ <= 0.0)
  {
    return origins;
  }

  // Le bord du disque, dans le plan (x, y) de la pose, decolle le long de l'axe outil
  const Eigen::Vector3d x_axis = pose.linear().col(0);
  const Eigen::Vector3d y_axis = pose.linear().col(1);
  const Eigen::Vector3d lift = pose.linear().col(2) * (tool_radius_ * kSpokeLiftFraction);
  for (unsigned k = 0; k < ring_samples_; ++k)
  {
    const double angle = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(ring_samples_);
    origins.push_back(pose.translation() + lift + tool_radius_ * (std::cos(angle) * x_axis + std::sin(angle) * y_axis));
  }
  return origins;
}

bool OcclusionModifier::isReachable(const Eigen::Isometry3d& pose) const
{
  const Eigen::Vector3d tool_axis = pose.linear().col(2);
  const std::vector<Eigen::Vector3d> origins = rayOrigins(pose);
  for (const Eigen::Vector3d& origin : origins)
  {
    if (rayBlocked(origin, tool_axis))
    {
      return false;
    }
  }
  return !spokesBlocked(pose, origins);
}

std::vector<ToolPathSegment> OcclusionModifier::splitReachable(const ToolPathSegment& segment) const
{
  std::vector<ToolPathSegment> runs;
  ToolPathSegment current;
  for (const Eigen::Isometry3d& pose : segment)
  {
    if (isReachable(pose))
    {
      current.push_back(pose);
      continue;
    }
    // Une pose inaccessible clot la suite en cours : l'outil ne peut pas la traverser au contact
    if (current.size() >= min_segment_poses_)
    {
      runs.push_back(current);
    }
    current.clear();
  }
  if (current.size() >= min_segment_poses_)
  {
    runs.push_back(current);
  }
  return runs;
}

ToolPaths OcclusionModifier::modify(ToolPaths tool_paths) const
{
  ToolPaths output;
  for (const ToolPath& tool_path : tool_paths)
  {
    ToolPath kept;
    for (const ToolPathSegment& segment : tool_path)
    {
      for (const ToolPathSegment& run : splitReachable(segment))
      {
        kept.push_back(run);
      }
    }
    // Un chemin dont plus rien n'est accessible disparait plutot que de rester vide
    if (!kept.empty())
    {
      output.push_back(kept);
    }
  }
  return output;
}

std::size_t countWaypoints(const ToolPaths& tool_paths)
{
  std::size_t count = 0;
  for (const ToolPath& tool_path : tool_paths)
  {
    for (const ToolPathSegment& segment : tool_path)
    {
      count += segment.size();
    }
  }
  return count;
}

}  // namespace noether
