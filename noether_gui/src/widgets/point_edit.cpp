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
#include <noether_gui/widgets/trace_tool.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace noether
{
int gapPointCount(const double length, const DensifySettings& settings)
{
  if (settings.mode == DensifyMode::Count)
  {
    return std::max(settings.count, 0);
  }
  if (!(settings.step > 0.0))
  {
    return 0;
  }
  return std::max(static_cast<int>(std::ceil(length / settings.step)) - 1, 0);
}

DensifyPlan planDensify(const std::vector<EditedPass>& passes, const PoseRef& a, const PoseRef& b,
                        const DensifySettings& settings)
{
  if (a.pass != b.pass || a.index >= b.index)
  {
    throw std::invalid_argument("Densifier demande deux poses d'une meme passe, dans le sens de la passe");
  }
  const EditedPass& pass = passes.at(a.pass);
  if (!pass.surface || b.index >= pass.poses.size())
  {
    throw std::invalid_argument("Densifier demande une passe dont la surface est connue");
  }
  DensifyPlan plan;
  for (std::size_t k = a.index; k < b.index; ++k)
  {
    const double length = (pass.poses[k + 1].translation() - pass.poses[k].translation()).norm();
    const std::optional<ToolPathSegment> poses =
        densifyGap(*pass.surface, pass.poses[k], pass.poses[k + 1], gapPointCount(length, settings));
    if (!poses)
    {
      ++plan.skipped;
    }
    else if (!poses->empty())
    {
      plan.insertions.push_back(GapInsertion{ a.pass, k, *poses });
    }
  }
  return plan;
}

PoseRef shiftedByInsertions(const PoseRef& ref, const std::vector<GapInsertion>& insertions)
{
  PoseRef shifted = ref;
  for (const GapInsertion& insertion : insertions)
  {
    // Les poses inserees apres une pose qui precede `ref` la font reculer d'autant
    if (insertion.pass == ref.pass && insertion.after < ref.index)
    {
      shifted.index += insertion.poses.size();
    }
  }
  return shifted;
}

void PointSelection::click(const std::optional<PoseRef>& point, const bool ctrl)
{
  if (!point)
  {
    // Ctrl + clic dans le vide ne change rien (convention SolidWorks)
    if (!ctrl)
    {
      points_.clear();
    }
    return;
  }
  if (!ctrl)
  {
    points_ = { *point };
    return;
  }
  const auto found = std::find(points_.begin(), points_.end(), *point);
  if (found != points_.end())
  {
    points_.erase(found);
  }
  else
  {
    points_.push_back(*point);
  }
}

void PointSelection::beginDrag(const PoseRef& pressed)
{
  if (!contains(pressed))
  {
    points_ = { pressed };
  }
}

bool PointSelection::contains(const PoseRef& point) const
{
  return std::find(points_.begin(), points_.end(), point) != points_.end();
}

std::optional<std::pair<PoseRef, PoseRef>> PointSelection::densifyEnds() const
{
  if (points_.size() != 2 || points_[0].pass != points_[1].pass || points_[0].index == points_[1].index)
  {
    return std::nullopt;
  }
  // A est la premiere des deux dans le sens de la passe
  return points_[0].index < points_[1].index ? std::make_pair(points_[0], points_[1]) :
                                               std::make_pair(points_[1], points_[0]);
}

ToolPathSegment draggedPass(const EditedPass& before, const std::vector<std::size_t>& indices,
                            const Eigen::Vector3d& delta)
{
  if (!before.surface)
  {
    throw std::invalid_argument("Une passe sans surface ne se glisse pas");
  }
  const ToolPathSegment& start = before.poses;
  ToolPathSegment moved = start;
  for (const std::size_t i : indices)
  {
    if (i == 0 || i + 1 >= start.size())
    {
      throw std::invalid_argument("Seules les poses de contact se glissent");
    }
    const std::optional<SurfacePoint> placed =
        before.surface->place(start[i].translation() + delta, start[i].linear().col(2));
    // Sans triangle valable, la pose reste a sa place d'avant le glisser
    if (placed)
    {
      moved[i].linear() = traceFrame(start[i].linear().col(0), placed->normal);
      moved[i].translation() = placed->position;
    }
  }
  // L'approche et le retrait gardent leur place dans le repere de la pose de contact qu'ils bordent
  moved.front() = moved[1] * (start[1].inverse() * start.front());
  moved.back() = moved[moved.size() - 2] * (start[start.size() - 2].inverse() * start.back());
  return moved;
}

}  // namespace noether
