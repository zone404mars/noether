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

#include <algorithm>
#include <stdexcept>

namespace
{
/// @brief Un troncon de passe au contact, entre deux poses consecutives.
struct Stroke
{
  Eigen::Vector3d from;
  Eigen::Vector3d to;
};

/// @brief Distance d'un point au segment [from, to].
double distanceToStroke(const Eigen::Vector3d& point, const Stroke& stroke)
{
  const Eigen::Vector3d axis = stroke.to - stroke.from;
  const double length_squared = axis.squaredNorm();
  if (length_squared <= 0.0)
  {
    return (point - stroke.from).norm();
  }
  const double t = std::min(1.0, std::max(0.0, (point - stroke.from).dot(axis) / length_squared));
  return (point - (stroke.from + t * axis)).norm();
}

/// @brief Les troncons au contact de toutes les passes, les bouts d'approche et de retrait ecartes.
std::vector<Stroke> contactStrokes(const noether::ToolPath& passes, const std::size_t skirt_poses)
{
  std::vector<Stroke> strokes;
  for (const noether::ToolPathSegment& pass : passes)
  {
    if (pass.size() < 2 * skirt_poses + 2)
    {
      continue;
    }
    for (std::size_t i = skirt_poses; i + 1 < pass.size() - skirt_poses; ++i)
    {
      strokes.push_back({ pass[i].translation(), pass[i + 1].translation() });
    }
  }
  return strokes;
}
}  // namespace

namespace noether
{
CoverageResult computeCoverage(const ToolPath& passes,
                               const std::vector<Eigen::Vector3d>& centroids,
                               const std::vector<int>& selected_faces,
                               const double tool_radius,
                               const std::size_t skirt_poses)
{
  if (!(tool_radius > 0.0))
  {
    throw std::invalid_argument("Le rayon d'outil doit etre strictement positif pour mesurer une couverture");
  }
  const std::vector<Stroke> strokes = contactStrokes(passes, skirt_poses);

  CoverageResult result;
  result.selected_count = selected_faces.size();
  for (const int face : selected_faces)
  {
    if (face < 0 || static_cast<std::size_t>(face) >= centroids.size())
    {
      throw std::invalid_argument("Face selectionnee sans centroide : " + std::to_string(face));
    }
    const Eigen::Vector3d& centroid = centroids[static_cast<std::size_t>(face)];
    const bool covered = std::any_of(strokes.begin(), strokes.end(), [&](const Stroke& stroke) {
      return distanceToStroke(centroid, stroke) <= tool_radius;
    });
    if (!covered)
    {
      result.uncovered_faces.push_back(face);
    }
  }
  if (result.selected_count > 0)
  {
    result.ratio = 1.0 - static_cast<double>(result.uncovered_faces.size()) / static_cast<double>(result.selected_count);
  }
  return result;
}

}  // namespace noether
