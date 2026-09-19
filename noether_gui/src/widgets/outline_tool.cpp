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
#include <noether_gui/widgets/outline_tool.h>

namespace noether
{
OutlineBuilder::OutlineBuilder(const double close_tolerance_px) : close_tolerance_px_(close_tolerance_px) {}

bool OutlineBuilder::addVertex(const Eigen::Vector2d& point)
{
  if (canClose() && (point - vertices_.front()).norm() <= close_tolerance_px_)
  {
    return true;
  }
  // Deux clics au meme endroit ne font pas deux sommets : le second est un double-clic ou une
  // hesitation, pas une intention
  if (!vertices_.empty() && (point - vertices_.back()).norm() <= close_tolerance_px_)
  {
    return false;
  }
  vertices_.push_back(point);
  return false;
}

void OutlineBuilder::removeLastVertex()
{
  if (!vertices_.empty())
  {
    vertices_.pop_back();
  }
}

void OutlineBuilder::clear() { vertices_.clear(); }

bool OutlineBuilder::canClose() const { return vertices_.size() >= 3; }

const std::vector<Eigen::Vector2d>& OutlineBuilder::vertices() const { return vertices_; }

}  // namespace noether
