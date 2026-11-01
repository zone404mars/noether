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
#include <noether_gui/widgets/point_picking.h>

#include <vtkAbstractCellLocator.h>

namespace noether
{
std::optional<std::size_t> nearestOnScreen(const std::vector<Eigen::Vector2d>& screen,
                                           const std::vector<bool>& visible,
                                           const Eigen::Vector2d& cursor,
                                           const double radius)
{
  std::optional<std::size_t> best;
  double best_distance = radius;
  for (std::size_t i = 0; i < screen.size() && i < visible.size(); ++i)
  {
    const double distance = (screen[i] - cursor).norm();
    if (visible[i] && distance <= best_distance)
    {
      best = i;
      best_distance = distance;
    }
  }
  return best;
}

bool visibleFrom(vtkAbstractCellLocator& locator, const Eigen::Vector3d& eye, const Eigen::Vector3d& point,
                 const double margin)
{
  const Eigen::Vector3d ray = point - eye;
  const double length = ray.norm();
  // Un point a moins de la marge de l'oeil n'a rien devant lui
  if (length <= margin)
  {
    return true;
  }
  const Eigen::Vector3d stop = point - ray * (margin / length);
  double parametric = 0.0;
  double hit[3] = { 0.0, 0.0, 0.0 };
  double parametric_coords[3] = { 0.0, 0.0, 0.0 };
  int sub_id = 0;
  // Le premier obstacle suffit : la tolerance nulle ne grossit pas les triangles
  return locator.IntersectWithLine(eye.data(), stop.data(), 0.0, parametric, hit, parametric_coords, sub_id) == 0;
}

}  // namespace noether
