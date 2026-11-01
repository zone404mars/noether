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
#include <noether_gui/widgets/edited_path.h>
#include <noether_gui/widgets/trace_tool.h>

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>

namespace
{
/** @brief Rangs a retirer, groupes par passe */
std::map<std::size_t, std::vector<std::size_t>> byPass(const std::vector<noether::PoseRef>& poses)
{
  std::map<std::size_t, std::vector<std::size_t>> grouped;
  for (const noether::PoseRef& ref : poses)
  {
    grouped[ref.pass].push_back(ref.index);
  }
  return grouped;
}

/** @brief Nombre de rangs distincts dans une liste qui peut porter des doublons */
std::size_t distinctCount(const std::vector<std::size_t>& indices)
{
  return std::set<std::size_t>(indices.begin(), indices.end()).size();
}

/**
 * @brief Poses de contact d'une passe privees des rangs `gone`
 * @param reaim Sortie : rangs, dans le resultat, des poses qui precedaient un trou
 */
noether::ToolPathSegment keptContacts(const noether::ToolPathSegment& pass,
                                      const std::set<std::size_t>& gone,
                                      std::set<std::size_t>& reaim)
{
  noether::ToolPathSegment kept;
  for (std::size_t i = 1; i + 1 < pass.size(); ++i)
  {
    if (gone.count(i) == 0)
    {
      kept.push_back(pass[i]);
    }
    else if (!kept.empty())
    {
      reaim.insert(kept.size() - 1);
    }
  }
  return kept;
}

/**
 * @brief Axe x des poses `reaim` dirige vers leur nouvelle suivante, z et position inchanges
 * @details La derniere pose n'a plus de suivante : elle garde le sens de son dernier troncon.
 */
void reaimBeforeGaps(noether::ToolPathSegment& kept, const std::set<std::size_t>& reaim)
{
  for (const std::size_t k : reaim)
  {
    const Eigen::Vector3d travel = (k + 1 < kept.size()) ? kept[k + 1].translation() - kept[k].translation() :
                                                           kept[k].translation() - kept[k - 1].translation();
    kept[k].linear() = noether::traceFrame(travel, kept[k].linear().col(2));
  }
}
}  // namespace

namespace noether
{
EditedPath::EditedPath(const std::size_t min_poses) : min_poses_(min_poses) {}

void EditedPath::reset(const std::vector<EditedPass>& passes)
{
  passes_ = passes;
  undo_.clear();
  redo_.clear();
}

void EditedPath::undo()
{
  if (undo_.empty())
  {
    return;
  }
  redo_.push_back(passes_);
  passes_ = undo_.back();
  undo_.pop_back();
}

void EditedPath::redo()
{
  if (redo_.empty())
  {
    return;
  }
  undo_.push_back(passes_);
  passes_ = redo_.back();
  redo_.pop_back();
}

void EditedPath::beginAction()
{
  undo_.push_back(passes_);
  // Une nouvelle action apres un Annuler rend la suite annulee inaccessible (spec §18)
  redo_.clear();
}

void EditedPath::setPose(const PoseRef& ref, const Eigen::Isometry3d& pose)
{
  passes_.at(ref.pass).poses.at(ref.index) = pose;
}

void EditedPath::movePass(const std::size_t from, const std::size_t to)
{
  if (from >= passes_.size() || to >= passes_.size())
  {
    throw std::out_of_range("Rang de passe absent du chemin livre");
  }
  beginAction();
  const EditedPass moved = passes_[from];
  passes_.erase(passes_.begin() + static_cast<std::ptrdiff_t>(from));
  passes_.insert(passes_.begin() + static_cast<std::ptrdiff_t>(to), moved);
}

void EditedPath::removePass(const std::size_t index)
{
  if (index >= passes_.size())
  {
    throw std::out_of_range("Rang de passe absent du chemin livre");
  }
  beginAction();
  passes_.erase(passes_.begin() + static_cast<std::ptrdiff_t>(index));
}

void EditedPath::insertPoses(const std::vector<GapInsertion>& insertions)
{
  for (const GapInsertion& insertion : insertions)
  {
    if (insertion.after + 1 >= passes_.at(insertion.pass).poses.size())
    {
      throw std::out_of_range("Ecart absent de la passe");
    }
  }
  beginAction();
  // Du dernier ecart au premier : une insertion ne decale pas les rangs de celles qui restent
  std::vector<GapInsertion> ordered = insertions;
  std::sort(ordered.begin(), ordered.end(), [](const GapInsertion& a, const GapInsertion& b) {
    return a.pass != b.pass ? a.pass > b.pass : a.after > b.after;
  });
  for (const GapInsertion& insertion : ordered)
  {
    ToolPathSegment& poses = passes_[insertion.pass].poses;
    poses.insert(poses.begin() + static_cast<std::ptrdiff_t>(insertion.after) + 1,
                 insertion.poses.begin(),
                 insertion.poses.end());
  }
}

std::optional<std::size_t> EditedPath::passTooShortAfterRemoval(const std::vector<PoseRef>& poses) const
{
  for (const auto& entry : byPass(poses))
  {
    const std::size_t size = passes_.at(entry.first).poses.size();
    if (size - std::min(size, distinctCount(entry.second)) < min_poses_)
    {
      return entry.first;
    }
  }
  return std::nullopt;
}

void EditedPath::removePoses(const std::vector<PoseRef>& poses)
{
  if (passTooShortAfterRemoval(poses))
  {
    throw std::invalid_argument("La suppression ferait passer une passe sous le minimum de poses");
  }
  // Toutes les passes sont calculees avant de toucher a la liste : une erreur ne laisse rien a moitie fait
  std::vector<EditedPass> edited = passes_;
  for (const auto& entry : byPass(poses))
  {
    edited[entry.first].poses = withoutContactPoses(passes_[entry.first].poses, entry.second);
  }
  beginAction();
  passes_ = edited;
}

ToolPathSegment withoutContactPoses(const ToolPathSegment& pass, const std::vector<std::size_t>& removed)
{
  const std::set<std::size_t> gone(removed.begin(), removed.end());
  if (pass.size() < 4 || gone.count(0) > 0 || gone.count(pass.size() - 1) > 0)
  {
    throw std::invalid_argument("Seules les poses de contact se suppriment");
  }
  std::set<std::size_t> reaim;
  ToolPathSegment kept = keptContacts(pass, gone, reaim);
  if (kept.size() < 2)
  {
    throw std::invalid_argument("Une passe garde au moins deux poses de contact");
  }
  reaimBeforeGaps(kept, reaim);
  // L'approche n'est replacee que si la premiere pose de contact disparait, le retrait de meme (§20.1)
  const bool first_gone = gone.count(1) > 0;
  const bool last_gone = gone.count(pass.size() - 2) > 0;
  ToolPathSegment result;
  result.push_back(first_gone ? Eigen::Isometry3d(kept.front() * (pass[1].inverse() * pass.front())) : pass.front());
  result.insert(result.end(), kept.begin(), kept.end());
  result.push_back(last_gone ? Eigen::Isometry3d(kept.back() * (pass[pass.size() - 2].inverse() * pass.back())) :
                               pass.back());
  return result;
}

}  // namespace noether
