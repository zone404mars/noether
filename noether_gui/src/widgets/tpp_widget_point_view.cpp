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
 * @file tpp_widget_point_view.cpp
 * @brief Ce que l'outil Editer les points montre (spec §19.4, §19.5, §21) : marques de selection
 * et de pre-selection, apercu de Densifier, barre contextuelle et page d'options.
 */

#include <noether_gui/widgets/tpp_widget.h>
#include "tpp_widget_defaults.h"
#include <noether_gui/widgets/path_edit_widget.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkCoordinate.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

namespace noether
{
namespace
{
/** @brief Couleur de pre-selection (§19.4) */
constexpr unsigned char kHoverRgb[3] = { 255, 255, 255 };
/** @brief Opacite des points de l'apercu de Densifier : ils ne sont pas encore dans le chemin */
constexpr double kPreviewOpacity = 0.6;
/** @brief Decalage, en pixels logiques, entre le curseur et le coin de la barre contextuelle */
constexpr int kBarOffsetPx = 12;

/** @brief Nuage de points sans aretes, une cellule par point */
vtkSmartPointer<vtkPolyData> pointCloud(const std::vector<Eigen::Vector3d>& positions)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto vertices = vtkSmartPointer<vtkCellArray>::New();
  for (const Eigen::Vector3d& position : positions)
  {
    const vtkIdType id = points->InsertNextPoint(position.data());
    vertices->InsertNextCell(1, &id);
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(vertices);
  return poly;
}

/** @brief Polyligne qui relie des positions dans l'ordre */
vtkSmartPointer<vtkPolyData> polyline(const std::vector<Eigen::Vector3d>& positions)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(static_cast<vtkIdType>(positions.size()));
  for (const Eigen::Vector3d& position : positions)
  {
    lines->InsertCellPoint(points->InsertNextPoint(position.data()));
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetLines(lines);
  return poly;
}

/** @brief Points dessines en spheres, d'une couleur et d'une taille a l'ecran */
void stylePoints(vtkActor& actor, const unsigned char rgb[3], const float size, const double opacity)
{
  actor.GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0);
  actor.GetProperty()->SetPointSize(size);
  actor.GetProperty()->SetRenderPointsAsSpheres(true);
  actor.GetProperty()->SetOpacity(opacity);
}
}  // namespace

QWidget* TPPWidget::buildEditPointsPage()
{
  point_info_label_ = new QLabel(this);
  point_info_label_->setWordWrap(true);
  delete_points_button_ = new QPushButton(this);
  translator_.bindText(delete_points_button_, { "Supprimer", "Delete" });
  connect(delete_points_button_, &QPushButton::clicked, this, [this](bool) { deleteSelectedPoints(); });
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(point_info_label_);
  layout->addWidget(delete_points_button_);
  layout->addWidget(buildDensifyControls());
  // La barre d'abord : la mise a jour du panneau regle aussi ses boutons
  buildContextBar();
  translator_.bind([this]() { updatePointPanel(); });
  return page;
}

QWidget* TPPWidget::buildDensifyControls()
{
  densify_mode_combo_ = new QComboBox(this);
  densify_mode_combo_->addItems({ QString(), QString() });
  translator_.bind([this]() {
    densify_mode_combo_->setItemText(0, say("Pas", "Step"));
    densify_mode_combo_->setItemText(1, say("Nombre", "Count"));
  });
  buildDensifySpinBoxes();
  densify_points_button_ = new QPushButton(this);
  translator_.bindText(densify_points_button_, { "Densifier", "Densify" });
  connect(densify_points_button_, &QPushButton::clicked, this, [this](bool) { densifySelectedPoints(); });
  auto* box = new QWidget(this);
  auto* layout = new QVBoxLayout(box);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(labelledRow({ "Mode", "Mode" }, densify_mode_combo_));
  densify_step_row_ = labelledRow({ "Pas cible", "Target step" }, densify_step_spin_box_);
  densify_count_row_ = labelledRow({ "Points par écart", "Points per gap" }, densify_count_spin_box_);
  layout->addWidget(densify_step_row_);
  layout->addWidget(densify_count_row_);
  layout->addWidget(densify_points_button_);
  connectDensifyControls();
  return box;
}

void TPPWidget::buildDensifySpinBoxes()
{
  // Bornes et valeurs d'ouverture de la spec §21 et §24
  densify_step_spin_box_ = new QDoubleSpinBox(this);
  densify_step_spin_box_->setRange(1.0, 1000.0);
  densify_step_spin_box_->setDecimals(1);
  densify_step_spin_box_->setValue(kDefaultDensifyStepMm);
  densify_step_spin_box_->setSuffix(" mm");
  densify_count_spin_box_ = new QSpinBox(this);
  densify_count_spin_box_->setRange(1, 100);
  densify_count_spin_box_->setValue(kDefaultDensifyCount);
}

void TPPWidget::connectDensifyControls()
{
  // Chaque reglage recalcule l'apercu ; le chemin livre ne change pas (§21)
  const auto changed = [this]() { onPointSelectionChanged(); render(); };
  connect(densify_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, changed);
  connect(densify_step_spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, changed);
  connect(densify_count_spin_box_, QOverload<int>::of(&QSpinBox::valueChanged), this, changed);
}

void TPPWidget::buildContextBar()
{
  // Enfant de la vue : elle se place pres du curseur, au-dessus de la scene
  context_bar_ = new QFrame(render_widget_);
  context_bar_->setFrameShape(QFrame::StyledPanel);
  context_bar_->setAutoFillBackground(true);
  auto* layout = new QHBoxLayout(context_bar_);
  layout->setContentsMargins(4, 4, 4, 4);
  context_delete_button_ = new QPushButton(context_bar_);
  translator_.bindText(context_delete_button_, { "Supprimer", "Delete" });
  connect(context_delete_button_, &QPushButton::clicked, this, [this](bool) { deleteSelectedPoints(); });
  context_densify_button_ = new QPushButton(context_bar_);
  translator_.bindText(context_densify_button_, { "Densifier", "Densify" });
  connect(context_densify_button_, &QPushButton::clicked, this, [this](bool) { densifySelectedPoints(); });
  layout->addWidget(context_delete_button_);
  layout->addWidget(context_densify_button_);
  context_bar_->hide();
}

void TPPWidget::showContextBar(const Eigen::Vector2d& display)
{
  context_bar_->adjustSize();
  // Les positions VTK sont en pixels physiques, origine en bas ; Qt en pixels logiques, origine en haut
  const double ratio = render_widget_->devicePixelRatioF();
  // Au-dessus et a droite du curseur, comme dans SolidWorks : en dessous, la barre couvrirait
  // le point suivant d'une passe qui descend, et un Ctrl + clic tomberait sur Supprimer
  int x = static_cast<int>(display.x() / ratio) + kBarOffsetPx;
  int y = render_widget_->height() - static_cast<int>(display.y() / ratio) - kBarOffsetPx - context_bar_->height();
  // Toujours entiere dans la vue (§19.5)
  x = std::max(0, std::min(x, render_widget_->width() - context_bar_->width()));
  y = std::max(0, std::min(y, render_widget_->height() - context_bar_->height()));
  context_bar_->move(x, y);
  context_bar_->show();
  context_bar_->raise();
}

void TPPWidget::hideContextBar()
{
  if (context_bar_ != nullptr)
  {
    context_bar_->hide();
  }
}

void TPPWidget::onPointSelectionChanged()
{
  if (point_selection_.empty())
  {
    hideContextBar();
  }
  updatePointPanel();
  updatePointMarkers();
  updateDensifyPreview();
}

void TPPWidget::updatePointPanel()
{
  if (point_info_label_ == nullptr)
  {
    return;
  }
  point_info_label_->setText(describePointSelection());
  const std::optional<QString> delete_reason = deleteBlockReason();
  const std::optional<QString> densify_reason = densifyBlockReason();
  // Une action indisponible est grisee, avec une bulle qui dit pourquoi (§19.5)
  for (QPushButton* button : { delete_points_button_, context_delete_button_ })
  {
    button->setEnabled(!delete_reason);
    button->setToolTip(delete_reason.value_or(QString()));
  }
  for (QPushButton* button : { densify_points_button_, context_densify_button_ })
  {
    button->setEnabled(!densify_reason);
    button->setToolTip(densify_reason.value_or(QString()));
  }
  const bool by_count = densify_mode_combo_->currentIndex() == 1;
  densify_step_row_->setVisible(!by_count);
  densify_count_row_->setVisible(by_count);
}

QString TPPWidget::describePointSelection() const
{
  const std::vector<PoseRef>& points = point_selection_.points();
  if (points.size() != 1)
  {
    return say("%1 points", "%1 points").arg(points.size());
  }
  const std::vector<std::pair<PoseRef, Eigen::Vector3d>> editable = editablePoints();
  for (const auto& point : editable)
  {
    if (point.first == points.front())
    {
      const Eigen::Vector3d mm = point.second * 1000.0;
      const QString where = editingTrace() ? say("Sommet %1", "Vertex %1").arg(points.front().index + 1) :
                                             say("Passe %1, point %2", "Pass %1, point %2")
                                                 .arg(points.front().pass + 1)
                                                 .arg(points.front().index);
      return where + QString("\n(%1, %2, %3) mm").arg(mm.x(), 0, 'f', 1).arg(mm.y(), 0, 'f', 1).arg(mm.z(), 0, 'f', 1);
    }
  }
  return QString();
}

void TPPWidget::updatePointMarkers()
{
  if (!renderer_)
  {
    return;
  }
  std::vector<Eigen::Vector3d> selected;
  std::vector<Eigen::Vector3d> hovered;
  if (editingPoints())
  {
    for (const auto& point : editablePoints())
    {
      if (point_selection_.contains(point.first))
      {
        selected.push_back(point.second);
      }
      else if (hovered_point_ && point.first == *hovered_point_)
      {
        hovered.push_back(point.second);
      }
    }
  }
  selected_points_actor_ = replaceActor(*renderer_, selected_points_actor_, pointCloud(selected));
  stylePoints(*selected_points_actor_, kGrabRgb, 16.0f, 1.0);
  hovered_point_actor_ = replaceActor(*renderer_, hovered_point_actor_, pointCloud(hovered));
  stylePoints(*hovered_point_actor_, kHoverRgb, 16.0f, 1.0);
}

void TPPWidget::updateDensifyPreview()
{
  if (!renderer_)
  {
    return;
  }
  std::vector<Eigen::Vector3d> preview;
  std::vector<Eigen::Vector3d> span;
  const std::optional<DensifyPlan> plan = editingPoints() ? currentDensifyPlan() : std::nullopt;
  if (plan)
  {
    for (const GapInsertion& insertion : plan->insertions)
    {
      for (const Eigen::Isometry3d& pose : insertion.poses)
      {
        preview.push_back(pose.translation());
      }
    }
    span = densifySpan();
  }
  densify_preview_actor_ = replaceActor(*renderer_, densify_preview_actor_, pointCloud(preview));
  stylePoints(*densify_preview_actor_, kGrabRgb, 10.0f, kPreviewOpacity);
  densify_span_actor_ = replaceActor(*renderer_, densify_span_actor_, polyline(span));
  densify_span_actor_->GetProperty()->SetColor(kGrabRgb[0] / 255.0, kGrabRgb[1] / 255.0, kGrabRgb[2] / 255.0);
  densify_span_actor_->GetProperty()->SetLineWidth(4.0f);
  updateDensifyLabels(span);
}

std::vector<Eigen::Vector3d> TPPWidget::densifySpan() const
{
  // Le troncon de A a B, surligne pendant l'apercu (§21)
  std::vector<Eigen::Vector3d> span;
  const std::optional<std::pair<PoseRef, PoseRef>> ends = point_selection_.densifyEnds();
  if (ends)
  {
    const ToolPathSegment& poses = path_edit_->editedPasses().at(ends->first.pass).poses;
    for (std::size_t i = ends->first.index; i <= ends->second.index; ++i)
    {
      span.push_back(poses[i].translation());
    }
  }
  return span;
}

vtkSmartPointer<vtkTextActor> TPPWidget::densifyLabel(const std::string& letter, const Eigen::Vector3d& pose)
{
  auto label = vtkSmartPointer<vtkTextActor>::New();
  label->SetInput(letter.c_str());
  // Position en pixels, relative au point projete de la pose ; a gauche, la barre contextuelle s'ouvrant a droite
  auto anchor = vtkSmartPointer<vtkCoordinate>::New();
  anchor->SetCoordinateSystemToWorld();
  anchor->SetValue(pose.x(), pose.y(), pose.z());
  label->GetPositionCoordinate()->SetCoordinateSystemToDisplay();
  label->GetPositionCoordinate()->SetReferenceCoordinate(anchor);
  label->GetPositionCoordinate()->SetValue(-34.0, -12.0);
  vtkTextProperty* text = label->GetTextProperty();
  text->SetFontSize(20);
  text->SetBold(true);
  text->SetColor(1.0, 1.0, 1.0);
  text->SetBackgroundColor(0.1, 0.1, 0.12);
  text->SetBackgroundOpacity(0.8);
  text->SetFrame(true);
  text->SetFrameColor(kGrabRgb[0] / 255.0, kGrabRgb[1] / 255.0, kGrabRgb[2] / 255.0);
  return label;
}

void TPPWidget::updateDensifyLabels(const std::vector<Eigen::Vector3d>& span)
{
  for (const vtkSmartPointer<vtkTextActor>& label : densify_labels_)
  {
    renderer_->RemoveActor2D(label);
  }
  densify_labels_.clear();
  if (span.size() < 2)
  {
    return;
  }
  // Les lettres A et B sont ecrites a cote des deux poses (§21), en surimpression 2D : dessinees
  // dans la scene, elles passeraient sous les disques transparents
  const std::pair<std::string, Eigen::Vector3d> ends[] = { { "A", span.front() }, { "B", span.back() } };
  for (const auto& end : ends)
  {
    auto label = densifyLabel(end.first, end.second);
    renderer_->AddActor2D(label);
    densify_labels_.push_back(label);
  }
}

}  // namespace noether
