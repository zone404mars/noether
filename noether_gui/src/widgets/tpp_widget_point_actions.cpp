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

/**
 * @file tpp_widget_point_actions.cpp
 * @brief Actions de l'outil Editer les points (spec §19.5, §20, §21) : supprimer, densifier,
 * annuler et retablir, et les deplacements des sommets du trace avant generation.
 */

#include <noether_gui/widgets/tpp_widget.h>
#include "tpp_widget_defaults.h"
#include <noether_gui/widgets/path_edit_widget.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QStatusBar>
#include <QTreeWidgetItem>
#include <vtkPolyData.h>

namespace noether
{
std::optional<QString> TPPWidget::deleteBlockReason() const
{
  if (point_selection_.empty())
  {
    return say("Choisissez au moins un point.", "Choose at least one point.");
  }
  if (editingTrace())
  {
    const auto found = surface_traces_.find(selectedSurfaceId());
    const std::size_t count = (found == surface_traces_.end()) ? 0 : found->second.size();
    // Generer demande au moins deux sommets (§20.2)
    if (count < point_selection_.points().size() + 2)
    {
      return say("Le tracé garderait moins de 2 sommets.", "The trace would keep fewer than 2 vertices.");
    }
    return std::nullopt;
  }
  const std::optional<std::size_t> short_pass = path_edit_->passTooShortAfterRemoval(point_selection_.points());
  if (short_pass)
  {
    return say("La passe %1 garderait moins de %2 poses.", "Pass %1 would keep fewer than %2 poses.")
        .arg(*short_pass + 1)
        .arg(kMinPassPoses);
  }
  return std::nullopt;
}

void TPPWidget::deleteSelectedPoints()
{
  const std::optional<QString> reason = deleteBlockReason();
  if (reason)
  {
    statusBar()->showMessage(*reason);
    return;
  }
  const std::size_t count = point_selection_.points().size();
  if (editingTrace())
  {
    deleteTraceVertices();
  }
  else
  {
    path_edit_->removePoses(point_selection_.points());
  }
  // Une suppression vide la selection (§20.1)
  clearPointSelection();
  statusBar()->showMessage(say("%1 point(s) supprimé(s).", "%1 point(s) removed.").arg(count));
  render();
}

void TPPWidget::deleteTraceVertices()
{
  const int surface_id = selectedSurfaceId();
  pushTraceUndo();
  std::vector<Eigen::Vector3d>& vertices = surface_traces_[surface_id];
  std::vector<Eigen::Vector3d> kept;
  for (std::size_t i = 0; i < vertices.size(); ++i)
  {
    if (!point_selection_.contains(PoseRef{ static_cast<std::size_t>(surface_id), i }))
    {
      kept.push_back(vertices[i]);
    }
  }
  // La polyligne relie le sommet d'avant au sommet d'apres (§20.2)
  vertices = kept;
  relabelCurrentSurface();
  updateTraceMarkers();
}

std::optional<QString> TPPWidget::densifyBlockReason() const
{
  if (editingTrace())
  {
    return say("Générez d'abord le chemin.", "Generate the path first.");
  }
  if (!selection_poly_ || selection_poly_->GetNumberOfCells() == 0)
  {
    return say("Ouvrez la pièce du chemin.", "Open the part of the path.");
  }
  const std::optional<std::pair<PoseRef, PoseRef>> ends = point_selection_.densifyEnds();
  if (!ends)
  {
    return say("Choisissez exactement deux points d'une même passe.", "Choose exactly two points of one pass.");
  }
  if (!path_edit_->editedPasses().at(ends->first.pass).surface)
  {
    return say("Surface inconnue pour cette passe (chemin repris).", "Unknown surface for this pass (loaded path).");
  }
  const std::optional<DensifyPlan> plan = currentDensifyPlan();
  if (!plan || plan->insertions.empty())
  {
    return say("Aucun écart à densifier avec ces réglages.", "No gap to densify with these settings.");
  }
  return std::nullopt;
}

std::optional<DensifyPlan> TPPWidget::currentDensifyPlan() const
{
  const std::optional<std::pair<PoseRef, PoseRef>> ends = point_selection_.densifyEnds();
  if (editingTrace() || !ends || !path_edit_->editedPasses().at(ends->first.pass).surface)
  {
    return std::nullopt;
  }
  return planDensify(path_edit_->editedPasses(), ends->first, ends->second, densifySettings());
}

void TPPWidget::densifySelectedPoints()
{
  const std::optional<QString> reason = densifyBlockReason();
  if (reason)
  {
    statusBar()->showMessage(*reason);
    return;
  }
  const DensifyPlan plan = *currentDensifyPlan();
  const std::pair<PoseRef, PoseRef> ends = *point_selection_.densifyEnds();
  path_edit_->insertPoses(plan.insertions);
  // A et B restent choisies ; B a recule du nombre de poses inserees avant elle (§21)
  point_selection_.set({ ends.first, shiftedByInsertions(ends.second, plan.insertions) });
  onPointSelectionChanged();
  statusBar()->showMessage(densifyReport(plan));
  render();
}

QString TPPWidget::densifyReport(const DensifyPlan& plan) const
{
  std::size_t added = 0;
  for (const GapInsertion& insertion : plan.insertions)
  {
    added += insertion.poses.size();
  }
  QString message = say("%1 point(s) ajouté(s).", "%1 point(s) added.").arg(added);
  if (plan.skipped > 0)
  {
    message += say(" %1 écart(s) sauté(s) : pas de face de la surface du bon côté.",
                   " %1 gap(s) skipped: no face of the surface on the right side.")
                   .arg(plan.skipped);
  }
  return message;
}

void TPPWidget::pushTraceUndo()
{
  const int surface_id = selectedSurfaceId();
  trace_undo_[surface_id].push_back(surface_traces_[surface_id]);
  trace_redo_[surface_id].clear();
}

void TPPWidget::forgetTraceHistory(const int surface_id)
{
  trace_undo_.erase(surface_id);
  trace_redo_.erase(surface_id);
}

void TPPWidget::undoPointEdit()
{
  if (!editingTrace())
  {
    path_edit_->undo();
  }
  else if (!trace_undo_[selectedSurfaceId()].empty())
  {
    const int surface_id = selectedSurfaceId();
    trace_redo_[surface_id].push_back(surface_traces_[surface_id]);
    surface_traces_[surface_id] = trace_undo_[surface_id].back();
    trace_undo_[surface_id].pop_back();
    relabelCurrentSurface();
    updateTraceMarkers();
  }
  // Annuler vide la selection, comme dans SolidWorks (§18)
  clearPointSelection();
  render();
}

void TPPWidget::redoPointEdit()
{
  if (!editingTrace())
  {
    path_edit_->redo();
  }
  else if (!trace_redo_[selectedSurfaceId()].empty())
  {
    const int surface_id = selectedSurfaceId();
    trace_undo_[surface_id].push_back(surface_traces_[surface_id]);
    surface_traces_[surface_id] = trace_redo_[surface_id].back();
    trace_redo_[surface_id].pop_back();
    relabelCurrentSurface();
    updateTraceMarkers();
  }
  clearPointSelection();
  render();
}

void TPPWidget::dragTraceVerticesBy(const Eigen::Vector3d& delta)
{
  const int surface_id = selectedSurfaceId();
  if (!point_press_.trace_surface)
  {
    // Construite une fois par glisser : refaire le sous-maillage a chaque mouvement serait lent
    point_press_.trace_surface = std::make_shared<const PassSurface>(mesh_, facesOfSurface(surface_id));
  }
  std::vector<Eigen::Vector3d>& vertices = surface_traces_[surface_id];
  for (const PoseRef& ref : point_selection_.points())
  {
    const std::optional<SurfacePoint> placed =
        point_press_.trace_surface->closest(point_press_.trace_before.at(ref.index) + delta);
    if (placed)
    {
      vertices.at(ref.index) = placed->position;
    }
  }
  updateTraceMarkers();
}

void TPPWidget::relabelCurrentSurface()
{
  if (QTreeWidgetItem* item = currentSurfaceItem())
  {
    relabelSurface(item);
  }
}

DensifySettings TPPWidget::densifySettings() const
{
  // Les controles sont crees avec la fenetre : leurs valeurs sont toujours la
  const DensifyMode mode = densify_mode_combo_->currentIndex() == 1 ? DensifyMode::Count : DensifyMode::Step;
  return DensifySettings{ mode, densify_step_spin_box_->value() / 1000.0, densify_count_spin_box_->value() };
}

}  // namespace noether
