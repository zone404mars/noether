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
 * @file tpp_widget_densify.cpp
 * @brief Outil Densifier : deux poses de contact d'une meme passe generee, puis des poses
 * ajoutees entre elles a un pas cible, posees sur la piece et figees dans la retouche.
 */

#include <noether_gui/widgets/tpp_widget.h>
#include "tpp_widget_defaults.h"
#include <noether_gui/widgets/path_edit_widget.h>
#include <noether_gui/widgets/trace_tool.h>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QStatusBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellLocator.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace noether
{
bool TPPWidget::densifying() const { return densify_button_ != nullptr && densify_button_->isChecked(); }

void TPPWidget::onDensifyToggled(const bool checked)
{
  clearDensifyEnds();
  if (checked)
  {
    showOptions("densify",
                { "Densifier", "Densify" },
                { "Cliquez deux points de contact d'une même passe : chaque écart plus grand que le pas cible "
                  "reçoit des points posés sur la pièce. Les points existants restent en place.",
                  "Click two contact points of one pass: every gap longer than the target step gets points "
                  "placed on the part. The existing points stay where they are." });
  }
  statusBar()->showMessage(checked ? say("Densifier : cliquez le premier point.", "Densify: click the first point.") :
                                     say("Densifier : terminé.", "Densify: done."));
  render();
}

QWidget* TPPWidget::buildDensifyPage()
{
  densify_step_spin_box_ = new QDoubleSpinBox(this);
  densify_step_spin_box_->setRange(1.0, 1000.0);
  densify_step_spin_box_->setDecimals(1);
  densify_step_spin_box_->setSingleStep(1.0);
  densify_step_spin_box_->setValue(kDefaultDensifyStepMm);
  densify_step_spin_box_->setSuffix(" mm");
  densify_ends_label_ = new QLabel(this);
  densify_ends_label_->setWordWrap(true);
  translator_.bind([this]() { relabelDensifyEnds(); });
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(labelledRow({ "Pas cible", "Target step" }, densify_step_spin_box_));
  layout->addWidget(densify_ends_label_);
  addDensifyButtons(layout);
  return page;
}

void TPPWidget::addDensifyButtons(QVBoxLayout* layout)
{
  auto* apply = new QPushButton(this);
  translator_.bindText(apply, { "Densifier", "Densify" });
  connect(apply, &QPushButton::clicked, this, [this](bool) { applyDensify(); });
  auto* clear = new QPushButton(this);
  translator_.bindText(clear, { "Effacer les deux points", "Clear both points" });
  connect(clear, &QPushButton::clicked, this, [this](bool) {
    clearDensifyEnds();
    render();
  });
  layout->addWidget(apply);
  layout->addWidget(clear);
}

void TPPWidget::onDensifyClick(const int x, const int y)
{
  Eigen::Vector3d point;
  PoseMove found;
  if (path_edit_ == nullptr || !pickAnyPoint(x, y, point) || !locateContactPose(point, found))
  {
    statusBar()->showMessage(say("Aucun point de contact du chemin à moins de 50 mm du clic.",
                                 "No contact point of the path within 50 mm of the click."));
    return;
  }
  if (addDensifyEnd(found))
  {
    relabelDensifyEnds();
    updateDensifyMarkers();
    render();
  }
}

bool TPPWidget::addDensifyEnd(const PoseMove& found)
{
  // Un troisieme clic recommence le choix
  if (densify_ends_.size() == 2)
  {
    densify_ends_.clear();
  }
  // Les rangs ne se comparent que dans une meme passe de la generation
  if (!densify_ends_.empty() && densify_ends_.front().segment != found.segment)
  {
    statusBar()->showMessage(say("Le second point doit être sur la même passe générée que le premier.",
                                 "The second point must be on the same generated pass as the first."));
    return false;
  }
  densify_ends_.push_back(found);
  statusBar()->showMessage(densify_ends_.size() == 1 ? say("Premier point choisi : cliquez le second.",
                                                           "First point chosen: click the second.") :
                                                       say("Deux points choisis : réglez le pas, puis Densifier.",
                                                           "Two points chosen: set the step, then Densify."));
  return true;
}

void TPPWidget::relabelDensifyEnds()
{
  if (densify_ends_label_ == nullptr)
  {
    return;
  }
  QString text;
  const char* names[2] = { "A", "B" };
  for (std::size_t e = 0; e < densify_ends_.size(); ++e)
  {
    text += say("%1 : passe générée %2, point %3\n", "%1: generated pass %2, point %3\n")
                .arg(names[e])
                .arg(densify_ends_[e].segment + 1)
                .arg(densify_ends_[e].index);
  }
  densify_ends_label_->setText(text.isEmpty() ? say("Aucun point choisi.", "No point chosen.") : text.trimmed());
}

void TPPWidget::clearDensifyEnds()
{
  densify_ends_.clear();
  relabelDensifyEnds();
  updateDensifyMarkers();
}

void TPPWidget::updateDensifyMarkers()
{
  if (!renderer_)
  {
    return;
  }
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto vertices = vtkSmartPointer<vtkCellArray>::New();
  for (const PoseMove& end : densify_ends_)
  {
    const Eigen::Vector3d& t = end.pose.translation();
    const vtkIdType id = points->InsertNextPoint(t.x(), t.y(), t.z());
    vertices->InsertNextCell(1, &id);
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(vertices);
  densify_actor_ = replaceActor(*renderer_, densify_actor_, poly);
  densify_actor_->GetProperty()->SetColor(kGrabRgb[0] / 255.0, kGrabRgb[1] / 255.0, kGrabRgb[2] / 255.0);
  densify_actor_->GetProperty()->SetPointSize(22.0f);
  densify_actor_->GetProperty()->SetRenderPointsAsSpheres(true);
}

void TPPWidget::applyDensify()
{
  if (densify_ends_.size() != 2 || densify_ends_[0].index == densify_ends_[1].index)
  {
    statusBar()->showMessage(say("Choisissez deux points différents d'une même passe.",
                                 "Choose two different points of one pass."));
    return;
  }
  // Sans piece chargee, rien sur quoi poser les points : un chemin repris seul ne se densifie pas
  if (!selection_poly_ || selection_poly_->GetNumberOfCells() == 0)
  {
    statusBar()->showMessage(say("Ouvrez la pièce du chemin avant de densifier.", "Open the part of the path before densifying."));
    return;
  }
  QApplication::setOverrideCursor(Qt::WaitCursor);
  const std::vector<PoseInsertion> insertions =
      densifyInsertions(densify_ends_[0], densify_ends_[1], densify_step_spin_box_->value() / 1000.0);
  QApplication::restoreOverrideCursor();
  const std::size_t added = std::accumulate(
      insertions.begin(), insertions.end(), std::size_t(0),
      [](std::size_t sum, const PoseInsertion& insertion) { return sum + insertion.poses.size(); });
  // L'annulation de la page Ordre defait toute la densification d'un coup
  path_edit_->insertPoses(insertions);
  clearDensifyEnds();
  statusBar()->showMessage(say("%1 point(s) ajouté(s). Annuler les retire.", "%1 point(s) added. Undo removes them.").arg(added));
  render();
}

std::vector<PoseInsertion> TPPWidget::densifyInsertions(const PoseMove& a, const PoseMove& b, const double step) const
{
  // Les poses voisines sont lues deplacees : c'est entre leurs places actuelles qu'on densifie
  // movedGenerated renvoie une copie : elle est gardee le temps du calcul
  const std::vector<ToolPathSegment> moved = path_edit_->movedGenerated();
  const ToolPathSegment& pass = moved.at(a.segment);
  const std::size_t first = std::min(a.index, b.index);
  const std::size_t last = std::max(a.index, b.index);
  // Un seul localisateur pour toute la densification : le construire coute autant que le maillage
  auto locator = vtkSmartPointer<vtkCellLocator>::New();
  locator->SetDataSet(selection_poly_);
  locator->BuildLocator();
  std::vector<PoseInsertion> insertions;
  for (std::size_t k = first; k < last && k + 1 < pass.size(); ++k)
  {
    // Un ecart deja sous le pas donne une insertion vide : elle efface ce qui y avait ete ajoute
    insertions.push_back(PoseInsertion{ a.segment, k, gapPoses(*locator, pass[k], pass[k + 1], step) });
  }
  return insertions;
}

ToolPathSegment TPPWidget::gapPoses(vtkCellLocator& locator, const Eigen::Isometry3d& from, const Eigen::Isometry3d& to,
                                    const double step) const
{
  const Eigen::Vector3d start = from.translation();
  const Eigen::Vector3d end = to.translation();
  const int count = static_cast<int>(std::ceil((end - start).norm() / step)) - 1;
  if (count < 1)
  {
    return ToolPathSegment();
  }
  std::vector<Eigen::Vector3d> points;
  for (int i = 1; i <= count; ++i)
  {
    points.push_back(start + (end - start) * (static_cast<double>(i) / (count + 1)));
  }
  // Posees sur la piece, pas sur la corde ; l'orientation se calcule comme celle du trace
  std::vector<Eigen::Vector3d> normals;
  if (!projectOnMesh(locator, points, normals))
  {
    return ToolPathSegment();
  }
  points.insert(points.begin(), start);
  points.push_back(end);
  normals.insert(normals.begin(), from.linear().col(2));
  normals.push_back(to.linear().col(2));
  const ToolPathSegment poses = tracePoses(points, normals);
  return ToolPathSegment(poses.begin() + 1, poses.end() - 1);
}

bool TPPWidget::projectOnMesh(vtkCellLocator& locator, std::vector<Eigen::Vector3d>& points,
                              std::vector<Eigen::Vector3d>& normals) const
{
  // La cellule i de selection_poly_ est la face i du maillage : sa normale est face_normals_[i]
  normals.clear();
  for (Eigen::Vector3d& point : points)
  {
    double closest[3] = { 0.0, 0.0, 0.0 };
    vtkIdType cell = -1;
    int sub_id = 0;
    double squared_distance = 0.0;
    locator.FindClosestPoint(point.data(), closest, cell, sub_id, squared_distance);
    if (cell < 0 || static_cast<std::size_t>(cell) >= face_normals_.size())
    {
      return false;
    }
    point = Eigen::Vector3d(closest[0], closest[1], closest[2]);
    normals.push_back(face_normals_[static_cast<std::size_t>(cell)]);
  }
  return true;
}

}  // namespace noether
