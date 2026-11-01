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
 * @file tpp_widget_edit_points.cpp
 * @brief Outil Editer les points (spec §19) : saisie, selection, glisser et touches. Les actions
 * sur la selection (supprimer, densifier) et leur affichage sont dans tpp_widget_point_actions.cpp.
 */

#include <noether_gui/widgets/tpp_widget.h>
#include "tpp_widget_defaults.h"
#include <noether_gui/widgets/path_edit_widget.h>
#include <noether_gui/widgets/point_picking.h>

#include <QApplication>
#include <QStatusBar>
#include <QToolButton>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkCellLocator.h>
#include <vtkPolyData.h>
#include <vtkRenderer.h>

namespace noether
{
namespace
{
/** @brief Distance a l'ecran, en pixels logiques, en dessous de laquelle un point se saisit (N21) */
constexpr double kPickRadiusPx = 10.0;
/** @brief Marge, en metres, pres d'un point ou la piece ne le cache pas (§19.2) */
constexpr double kVisibilityMarginM = 0.001;
}  // namespace

bool TPPWidget::editingPoints() const { return edit_points_button_ != nullptr && edit_points_button_->isChecked(); }

bool TPPWidget::editingTrace() const { return path_edit_ == nullptr || path_edit_->editedPasses().empty(); }

void TPPWidget::onEditPointsToggled(const bool checked)
{
  // Quitter l'outil vide la selection, la barre et l'apercu (§19)
  clearPointSelection();
  hovered_point_.reset();
  point_pressed_ = false;
  if (checked)
  {
    outline_.clear();
    updateOutlineOverlay();
    showOptions("edit_points", { "Éditer les points", "Edit points" },
                { "Clic : choisit un point. Ctrl + clic : ajoute ou retire. Glisser : déplace les points choisis. "
                  "Suppr : supprime. Ctrl+Z, Ctrl+Y : annuler, rétablir.",
                  "Click: picks a point. Ctrl + click: adds or removes. Drag: moves the chosen points. "
                  "Delete: removes. Ctrl+Z, Ctrl+Y: undo, redo." });
  }
  updatePointMarkers();
  statusBar()->showMessage(checked ? say("Éditer les points : cliquez un point du chemin ou du tracé.",
                                         "Edit points: click a point of the path or of the trace.") :
                                     say("Éditer les points : terminé.", "Edit points: done."));
  render();
}

std::vector<std::pair<PoseRef, Eigen::Vector3d>> TPPWidget::editablePoints() const
{
  std::vector<std::pair<PoseRef, Eigen::Vector3d>> points;
  if (!editingTrace())
  {
    // Apres generation : les poses de contact ; l'approche et le retrait ne s'editent pas
    const std::vector<EditedPass>& passes = path_edit_->editedPasses();
    for (std::size_t p = 0; p < passes.size(); ++p)
    {
      for (std::size_t i = 1; i + 1 < passes[p].poses.size(); ++i)
      {
        points.emplace_back(PoseRef{ p, i }, passes[p].poses[i].translation());
      }
    }
    return points;
  }
  // Avant generation : les sommets du trace de la surface courante ; `pass` porte la surface
  const int surface_id = selectedSurfaceId();
  const auto found = surface_traces_.find(surface_id);
  if (surface_id >= 0 && found != surface_traces_.end())
  {
    for (std::size_t i = 0; i < found->second.size(); ++i)
    {
      points.emplace_back(PoseRef{ static_cast<std::size_t>(surface_id), i }, found->second[i]);
    }
  }
  return points;
}

std::optional<PoseRef> TPPWidget::pointUnderCursor(const int x, const int y)
{
  const std::vector<std::pair<PoseRef, Eigen::Vector3d>> points = editablePoints();
  // Les evenements VTK sont en pixels physiques : le rayon logique est mis a l'echelle de l'ecran
  const double radius = kPickRadiusPx * render_widget_->devicePixelRatioF();
  const Eigen::Vector2d cursor(x, y);
  std::vector<Eigen::Vector2d> screen;
  std::vector<bool> visible;
  for (const auto& point : points)
  {
    screen.push_back(toDisplay(point.second));
    // Un lancer de rayon par point coute cher : seuls les points proches du curseur sont testes
    visible.push_back((screen.back() - cursor).norm() <= radius && pointVisible(point.second));
  }
  const std::optional<std::size_t> nearest = nearestOnScreen(screen, visible, cursor, radius);
  return nearest ? std::optional<PoseRef>(points[*nearest].first) : std::nullopt;
}

Eigen::Vector2d TPPWidget::toDisplay(const Eigen::Vector3d& point) const
{
  renderer_->SetWorldPoint(point.x(), point.y(), point.z(), 1.0);
  renderer_->WorldToDisplay();
  const double* display = renderer_->GetDisplayPoint();
  return Eigen::Vector2d(display[0], display[1]);
}

bool TPPWidget::pointVisible(const Eigen::Vector3d& point)
{
  if (!selection_poly_ || selection_poly_->GetNumberOfCells() == 0)
  {
    return true;
  }
  if (!mesh_locator_ || mesh_locator_->GetDataSet() != selection_poly_)
  {
    mesh_locator_ = vtkSmartPointer<vtkCellLocator>::New();
    mesh_locator_->SetDataSet(selection_poly_);
    mesh_locator_->BuildLocator();
  }
  vtkCamera* camera = renderer_->GetActiveCamera();
  Eigen::Vector3d eye;
  camera->GetPosition(eye.data());
  if (camera->GetParallelProjection())
  {
    // En projection parallele, l'oeil est loin en arriere sur l'axe de vue
    Eigen::Vector3d direction;
    camera->GetDirectionOfProjection(direction.data());
    eye = point - direction * (2.0 * camera->GetDistance() + 10.0);
  }
  return visibleFrom(*mesh_locator_, eye, point, kVisibilityMarginM);
}

void TPPWidget::onEditHover(const int x, const int y)
{
  if (!editingPoints() || point_pressed_)
  {
    return;
  }
  const std::optional<PoseRef> hovered = pointUnderCursor(x, y);
  if (hovered.has_value() == hovered_point_.has_value() && (!hovered || *hovered == *hovered_point_))
  {
    return;
  }
  hovered_point_ = hovered;
  updatePointMarkers();
  render();
}

void TPPWidget::onEditPress(const int x, const int y)
{
  point_pressed_ = true;
  point_press_ = PointPress();
  point_press_.position = Eigen::Vector2d(x, y);
  // Ctrl ne compte qu'a l'appui (§19.3)
  point_press_.ctrl = (QApplication::keyboardModifiers() & Qt::ControlModifier) != 0;
  point_press_.point = pointUnderCursor(x, y);
}

void TPPWidget::onEditMove(const int x, const int y)
{
  if (!point_pressed_ || !pressBecameDrag(x, y))
  {
    return;
  }
  Eigen::Vector3d cursor;
  // Curseur hors de la piece : les points restent a leur derniere place
  if (pickPoint(x, y, cursor) < 0)
  {
    return;
  }
  // Le point d'annulation n'est pose qu'au premier deplacement : un glisser sans effet n'est pas une action
  if (!point_press_.moved)
  {
    rememberBeforeDrag();
    point_press_.moved = true;
  }
  dragPointsBy(cursor - point_press_.piece_start);
  updatePointMarkers();
  render();
}

bool TPPWidget::pressBecameDrag(const int x, const int y)
{
  if (point_press_.dragging)
  {
    return true;
  }
  const double threshold = QApplication::startDragDistance() * render_widget_->devicePixelRatioF();
  if ((Eigen::Vector2d(x, y) - point_press_.position).norm() < threshold)
  {
    return false;
  }
  // Au-dela du seuil, le relachement n'est plus un clic (§19.2)
  point_press_.beyond_threshold = true;
  return point_press_.point && startPointDrag();
}

void TPPWidget::onEditRelease()
{
  if (!point_pressed_)
  {
    return;
  }
  point_pressed_ = false;
  if (point_press_.dragging)
  {
    finishPointDrag();
    return;
  }
  // Un glisser parti du vide, ou refuse, n'est pas un clic ; Ctrl + clic dans le vide ne fait rien (§19.3)
  if (point_press_.beyond_threshold || (!point_press_.point && point_press_.ctrl))
  {
    return;
  }
  point_selection_.click(point_press_.point, point_press_.ctrl);
  onPointSelectionChanged();
  // La barre apparait apres un clic qui laisse une selection ; tout autre clic dans la vue la ferme
  hideContextBar();
  if (!point_selection_.empty())
  {
    showContextBar(point_press_.position);
  }
  render();
}

bool TPPWidget::startPointDrag()
{
  const std::optional<QString> reason = dragBlockReason();
  Eigen::Vector3d start;
  const int x = static_cast<int>(point_press_.position.x());
  const int y = static_cast<int>(point_press_.position.y());
  if (reason || pickPoint(x, y, start) < 0)
  {
    const QString off_part = say("Commencez le glisser sur la pièce.", "Start the drag on the part.");
    statusBar()->showMessage(reason.value_or(off_part));
    // L'appui est abandonne : ni glisser, ni clic au relachement
    point_pressed_ = false;
    return false;
  }
  point_selection_.beginDrag(*point_press_.point);
  hideContextBar();
  point_press_.dragging = true;
  point_press_.piece_start = start;
  onPointSelectionChanged();
  return true;
}

std::optional<QString> TPPWidget::dragBlockReason() const
{
  if (!selection_poly_ || selection_poly_->GetNumberOfCells() == 0)
  {
    return say("Ouvrez la pièce du chemin.", "Open the part of the path.");
  }
  // Points que le glisser emporterait : la selection si le point saisi en fait partie, sinon lui seul
  const PoseRef pressed = *point_press_.point;
  const std::vector<PoseRef> dragged =
      point_selection_.contains(pressed) ? point_selection_.points() : std::vector<PoseRef>{ pressed };
  if (!editingTrace() && !passesHaveSurface(dragged))
  {
    return say("Surface inconnue pour cette passe (chemin repris).", "Unknown surface for this pass (loaded path).");
  }
  return std::nullopt;
}

bool TPPWidget::passesHaveSurface(const std::vector<PoseRef>& points) const
{
  for (const PoseRef& ref : points)
  {
    if (!path_edit_->editedPasses().at(ref.pass).surface)
    {
      return false;
    }
  }
  return true;
}

void TPPWidget::rememberBeforeDrag()
{
  if (editingTrace())
  {
    point_press_.trace_before = surface_traces_[selectedSurfaceId()];
    pushTraceUndo();
    return;
  }
  point_press_.passes_before = path_edit_->editedPasses();
  path_edit_->beginDrag();
}

void TPPWidget::dragPointsBy(const Eigen::Vector3d& delta)
{
  if (editingTrace())
  {
    dragTraceVerticesBy(delta);
    return;
  }
  // Les poses de chaque passe touchee, recalculees depuis leur place au debut du glisser
  std::map<std::size_t, std::vector<std::size_t>> by_pass;
  for (const PoseRef& ref : point_selection_.points())
  {
    by_pass[ref.pass].push_back(ref.index);
  }
  for (const auto& entry : by_pass)
  {
    const ToolPathSegment moved = draggedPass(point_press_.passes_before.at(entry.first), entry.second, delta);
    for (std::size_t i = 0; i < moved.size(); ++i)
    {
      path_edit_->setPose(PoseRef{ entry.first, i }, moved[i]);
    }
  }
}

void TPPWidget::finishPointDrag()
{
  if (!point_press_.moved)
  {
    // Rien n'a bouge, mais la selection a pu changer a l'appui
    render();
    return;
  }
  if (editingTrace())
  {
    relabelCurrentSurface();
    updateTraceMarkers();
  }
  else
  {
    // Un seul recalcul du chemin, de la couverture et des cercles, au relachement
    path_edit_->endDrag();
  }
  // Panneau, apercu et lettres A et B suivent les points poses
  onPointSelectionChanged();
  statusBar()->showMessage(
      say("Points posés. Annuler défait tout le glisser.", "Points dropped. Undo reverts the whole drag."));
  render();
}

void TPPWidget::onEditKey(const std::string& key)
{
  // Pendant un glisser, une touche agirait sur un chemin en cours de deplacement
  if (point_pressed_)
  {
    return;
  }
  const bool ctrl = (QApplication::keyboardModifiers() & Qt::ControlModifier) != 0;
  if (key == "Delete")
  {
    deleteSelectedPoints();
  }
  else if (key == "Escape")
  {
    clearPointSelection();
    render();
  }
  else if (ctrl && (key == "z" || key == "Z"))
  {
    undoPointEdit();
  }
  else if (ctrl && (key == "y" || key == "Y"))
  {
    redoPointEdit();
  }
}

void TPPWidget::clearPointSelection()
{
  point_selection_.clear();
  onPointSelectionChanged();
}

}  // namespace noether
