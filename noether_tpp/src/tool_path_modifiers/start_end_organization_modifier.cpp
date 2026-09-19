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
#include <noether_tpp/tool_path_modifiers/start_end_organization_modifier.h>
#include <noether_tpp/serialization.h>

#include <algorithm>
#include <limits>

namespace
{
/// @brief Premiere pose d'une ligne de raster, faite de segments consecutifs.
Eigen::Vector3d lineStart(const noether::ToolPath& line) { return line.front().front().translation(); }

/// @brief Derniere pose d'une ligne de raster.
Eigen::Vector3d lineEnd(const noether::ToolPath& line) { return line.back().back().translation(); }

/// @brief La ligne parcourue a l'envers : segments dans l'ordre inverse, chacun retourne.
noether::ToolPath reversedLine(const noether::ToolPath& line)
{
  noether::ToolPath reversed;
  for (auto segment = line.rbegin(); segment != line.rend(); ++segment)
  {
    reversed.push_back(noether::ToolPathSegment(segment->rbegin(), segment->rend()));
  }
  return reversed;
}

/// @brief La ligne dans le sens demande.
noether::ToolPath oriented(const noether::ToolPath& line, const bool reversed)
{
  return reversed ? reversedLine(line) : line;
}

/// @brief Parmi les lignes restantes, celle dont une extremite est la plus proche de `end`.
/// @return Son rang, et vrai si c'est par sa fin qu'on l'aborde
std::pair<std::size_t, bool> nearestLine(const noether::ToolPaths& lines,
                                         const std::vector<std::size_t>& remaining,
                                         const Eigen::Vector3d& end)
{
  std::pair<std::size_t, bool> best(remaining.front(), false);
  double best_distance = std::numeric_limits<double>::infinity();
  for (const std::size_t index : remaining)
  {
    const double to_start = (lineStart(lines[index]) - end).norm();
    const double to_end = (lineEnd(lines[index]) - end).norm();
    if (to_start < best_distance)
    {
      best_distance = to_start;
      best = std::make_pair(index, false);
    }
    if (to_end < best_distance)
    {
      best_distance = to_end;
      best = std::make_pair(index, true);
    }
  }
  return best;
}

/// @brief Enchaine les lignes du plus proche au plus proche a partir de `first`, dans le sens donne.
noether::ToolPaths greedyChain(const noether::ToolPaths& lines, const std::size_t first, const bool first_reversed)
{
  noether::ToolPaths chain;
  chain.push_back(oriented(lines[first], first_reversed));
  std::vector<std::size_t> remaining;
  for (std::size_t i = 0; i < lines.size(); ++i)
  {
    if (i != first)
    {
      remaining.push_back(i);
    }
  }
  while (!remaining.empty())
  {
    const std::pair<std::size_t, bool> next = nearestLine(lines, remaining, lineEnd(chain.back()));
    chain.push_back(oriented(lines[next.first], next.second));
    remaining.erase(std::find(remaining.begin(), remaining.end(), next.first));
  }
  return chain;
}

/// @brief Ecart aux deux points demandes : depart a la premiere pose, arrivee a la derniere.
double endpointsCost(const noether::ToolPaths& chain, const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
  return (lineStart(chain.front()) - start).norm() + (lineEnd(chain.back()) - end).norm();
}

/// @brief Longueur des sauts entre lignes consecutives, pour departager deux organisations.
double travelBetweenLines(const noether::ToolPaths& chain)
{
  double travel = 0.0;
  for (std::size_t i = 1; i < chain.size(); ++i)
  {
    travel += (lineStart(chain[i]) - lineEnd(chain[i - 1])).norm();
  }
  return travel;
}

/// @brief Les lignes qui portent au moins une pose ; les autres n'ont ni debut ni fin.
noether::ToolPaths nonEmptyLines(const noether::ToolPaths& tool_paths)
{
  noether::ToolPaths lines;
  for (const noether::ToolPath& line : tool_paths)
  {
    noether::ToolPath kept;
    for (const noether::ToolPathSegment& segment : line)
    {
      if (!segment.empty())
      {
        kept.push_back(segment);
      }
    }
    if (!kept.empty())
    {
      lines.push_back(kept);
    }
  }
  return lines;
}
}  // namespace

namespace noether
{
StartEndOrganizationModifier::StartEndOrganizationModifier(const Eigen::Vector3d& start, const Eigen::Vector3d& end)
  : start_(start), end_(end)
{
}

ToolPaths StartEndOrganizationModifier::modify(ToolPaths tool_paths) const
{
  const ToolPaths lines = nonEmptyLines(tool_paths);
  if (lines.empty())
  {
    return lines;
  }
  // Chaque ligne, dans chaque sens, est essayee comme premiere ; l'organisation retenue colle le
  // mieux aux deux points, puis saute le moins entre les lignes
  ToolPaths best;
  double best_cost = std::numeric_limits<double>::infinity();
  double best_travel = std::numeric_limits<double>::infinity();
  for (std::size_t first = 0; first < lines.size(); ++first)
  {
    for (const bool reversed : { false, true })
    {
      const ToolPaths chain = greedyChain(lines, first, reversed);
      const double cost = endpointsCost(chain, start_, end_);
      const double travel = travelBetweenLines(chain);
      if (cost < best_cost - 1e-12 || (std::abs(cost - best_cost) <= 1e-12 && travel < best_travel))
      {
        best = chain;
        best_cost = cost;
        best_travel = travel;
      }
    }
  }
  return best;
}

double StartEndOrganizationModifier::endGap(const ToolPaths& tool_paths) const
{
  if (tool_paths.empty() || tool_paths.back().empty() || tool_paths.back().back().empty())
  {
    return std::numeric_limits<double>::infinity();
  }
  return (lineEnd(tool_paths.back()) - end_).norm();
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::StartEndOrganizationModifier>::encode(const noether::StartEndOrganizationModifier& val)
{
  Node node;
  node["start"] = val.start_;
  node["end"] = val.end_;
  return node;
}

bool convert<noether::StartEndOrganizationModifier>::decode(const Node& node, noether::StartEndOrganizationModifier& val)
{
  val.start_ = getMember<Eigen::Vector3d>(node, "start");
  val.end_ = getMember<Eigen::Vector3d>(node, "end");
  return true;
}
/** @endcond */

}  // namespace YAML
