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
 * @file tpp_widget_coverage.cpp
 * @brief Dessin des disques de l'outil (pleins ou cercles, sureleves) et analyse de couverture :
 * les faces d'une surface que le disque n'atteint pas, pendant le trace et sur le chemin livre.
 */

#include <noether_gui/widgets/tpp_widget.h>
#include "tpp_widget_defaults.h"
#include <noether_gui/widgets/path_edit_widget.h>
#include <noether_gui/widgets/trace_tool.h>
#include <noether_tpp/core/face_coverage.h>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkRenderer.h>

namespace
{
/**
 * @brief Ajoute le bord d'un disque dans le plan (x, y) d'une pose, decale le long de son axe z
 * @param filled Vrai : un polygone plein ; faux : une ligne fermee
 * @param cells Sortie : polygones ou lignes, selon `filled`
 */
void appendDisc(const Eigen::Isometry3d& pose, const double radius, const double lift, const bool filled,
                vtkPoints& points, vtkCellArray& cells)
{
  // Une ligne fermee reprend son premier point ; un polygone se ferme de lui-meme
  const int count = filled ? noether::kCircleSegments : noether::kCircleSegments + 1;
  vtkSmartPointer<vtkCell> cell = filled ? vtkSmartPointer<vtkCell>(vtkSmartPointer<vtkPolygon>::New()) :
                                           vtkSmartPointer<vtkCell>(vtkSmartPointer<vtkPolyLine>::New());
  cell->GetPointIds()->SetNumberOfIds(count);
  for (int i = 0; i < count; ++i)
  {
    const double angle = 2.0 * M_PI * static_cast<double>(i % noether::kCircleSegments) / noether::kCircleSegments;
    const Eigen::Vector3d rim = pose * Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), lift);
    cell->GetPointIds()->SetId(i, points.InsertNextPoint(rim.x(), rim.y(), rim.z()));
  }
  cells.InsertNextCell(cell);
}

/** @brief Triangles d'une mesure de couverture : sommets, centroide et aire de chacun */
struct CoverageCells
{
  /** @brief Sommets de chaque triangle, deja decales au-dessus de la piece pour l'affichage */
  std::vector<std::array<Eigen::Vector3d, 3>> corners;
  /** @brief Centroide de chaque triangle, sur la piece : c'est lui qui est mesure */
  std::vector<Eigen::Vector3d> centroids;
  /** @brief Aire de chaque triangle, en metres carres */
  std::vector<double> areas;
};

/** @brief Decalage des zones non couvertes au-dessus de la piece, en metres, pour qu'elles ne s'y melent pas */
const double kUncoveredLiftM = 0.0005;

/** @brief Cote maximal des triangles mesures, en fraction du rayon d'outil */
const double kCoverageCellsPerRadius = 6.0;

/** @brief Profondeur maximale du decoupage d'un triangle : 4^8 morceaux au plus */
const int kMaxSubdivisionDepth = 8;

/**
 * @brief Decoupe un triangle en quatre, recursivement, jusqu'a ce que son plus grand cote soit sous `max_edge`
 * @param lift Decalage des sommets affiches, le long de la normale de la face
 */
void subdivideTriangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                       const double max_edge, const Eigen::Vector3d& lift, const int depth, CoverageCells& cells)
{
  const double longest = std::max({ (b - a).norm(), (c - b).norm(), (a - c).norm() });
  if (longest > max_edge && depth < kMaxSubdivisionDepth)
  {
    const Eigen::Vector3d ab = 0.5 * (a + b);
    const Eigen::Vector3d bc = 0.5 * (b + c);
    const Eigen::Vector3d ca = 0.5 * (c + a);
    subdivideTriangle(a, ab, ca, max_edge, lift, depth + 1, cells);
    subdivideTriangle(ab, b, bc, max_edge, lift, depth + 1, cells);
    subdivideTriangle(ca, bc, c, max_edge, lift, depth + 1, cells);
    subdivideTriangle(ab, bc, ca, max_edge, lift, depth + 1, cells);
    return;
  }
  cells.corners.push_back({ a + lift, b + lift, c + lift });
  cells.centroids.push_back((a + b + c) / 3.0);
  cells.areas.push_back(0.5 * (b - a).cross(c - a).norm());
}

/**
 * @brief Triangles de mesure couvrant des faces du maillage
 * @details Une face a plus de trois sommets est decoupee en eventail depuis son premier sommet.
 * @param poly Maillage affiche, dont la cellule i est la face i
 * @param normals Normale de chaque face, pour decaler les triangles affiches
 */
CoverageCells subdivideFaces(vtkPolyData& poly, const std::vector<int>& faces,
                             const std::vector<Eigen::Vector3d>& normals, const double max_edge)
{
  CoverageCells cells;
  vtkNew<vtkIdList> ids;
  for (const int face : faces)
  {
    poly.GetCellPoints(face, ids);
    const Eigen::Vector3d lift = kUncoveredLiftM * normals[static_cast<std::size_t>(face)];
    Eigen::Vector3d first;
    poly.GetPoint(ids->GetId(0), first.data());
    for (vtkIdType k = 1; k + 1 < ids->GetNumberOfIds(); ++k)
    {
      Eigen::Vector3d second;
      Eigen::Vector3d third;
      poly.GetPoint(ids->GetId(k), second.data());
      poly.GetPoint(ids->GetId(k + 1), third.data());
      subdivideTriangle(first, second, third, max_edge, lift, 0, cells);
    }
  }
  return cells;
}

/** @brief Nombre maximal de triangles de mesure : au-dela, ils sont agrandis */
const double kMaxCoverageCells = 200000.0;

/**
 * @brief Cote des triangles de mesure : `wanted`, agrandi si leur nombre depasserait kMaxCoverageCells
 * @details Le nombre est estime par l'aire des faces divisee par l'aire d'un triangle de ce cote.
 */
double coverageCellSize(vtkPolyData& poly, const std::vector<int>& faces, const double wanted)
{
  vtkNew<vtkIdList> ids;
  double area = 0.0;
  for (const int face : faces)
  {
    poly.GetCellPoints(face, ids);
    Eigen::Vector3d first;
    poly.GetPoint(ids->GetId(0), first.data());
    for (vtkIdType k = 1; k + 1 < ids->GetNumberOfIds(); ++k)
    {
      Eigen::Vector3d second;
      Eigen::Vector3d third;
      poly.GetPoint(ids->GetId(k), second.data());
      poly.GetPoint(ids->GetId(k + 1), third.data());
      area += 0.5 * (second - first).cross(third - first).norm();
    }
  }
  const double estimated = area / (0.5 * wanted * wanted);
  return estimated > kMaxCoverageCells ? wanted * std::sqrt(estimated / kMaxCoverageCells) : wanted;
}

/** @brief Triangles non couverts, prets a dessiner */
vtkSmartPointer<vtkPolyData> uncoveredCellsPoly(const CoverageCells& cells, const std::vector<int>& uncovered)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto polys = vtkSmartPointer<vtkCellArray>::New();
  for (const int cell : uncovered)
  {
    vtkIdType ids[3];
    for (int corner = 0; corner < 3; ++corner)
    {
      const Eigen::Vector3d& p = cells.corners[static_cast<std::size_t>(cell)][static_cast<std::size_t>(corner)];
      ids[corner] = points->InsertNextPoint(p.x(), p.y(), p.z());
    }
    polys->InsertNextCell(3, ids);
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetPolys(polys);
  return poly;
}

/** @brief Part couverte de la surface et aire non couverte, dans les deux langues */
noether::Bilingual coverageText(const CoverageCells& cells, const std::vector<int>& uncovered)
{
  const double total = std::accumulate(cells.areas.begin(), cells.areas.end(), 0.0);
  double missing = 0.0;
  for (const int cell : uncovered)
  {
    missing += cells.areas[static_cast<std::size_t>(cell)];
  }
  const double ratio = total > 0.0 ? 1.0 - missing / total : 1.0;
  const QString percent = QString::number(100.0 * ratio, 'f', 1);
  const QString area = QString::number(missing * 1.0e4, 'f', 0);
  return { QString::fromUtf8("Couverture : %1 % de la surface, %2 cm² non couverts").arg(percent).arg(area),
           QString::fromUtf8("Coverage: %1 % of the surface, %2 cm² uncovered").arg(percent).arg(area) };
}
}  // namespace

namespace noether
{
// ---------------------------------------------------------------------------------------------
// Disques

double TPPWidget::discRadius() const
{
  const bool enabled = tool_radius_enabled_check_box_ != nullptr && tool_radius_enabled_check_box_->isChecked();
  return enabled ? tool_radius_spin_box_->value() / 1000.0 : 0.0;
}

vtkSmartPointer<vtkPolyData> TPPWidget::discFootprints(const std::vector<Eigen::Isometry3d>& poses) const
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto cells = vtkSmartPointer<vtkCellArray>::New();
  const bool filled = filled_discs_check_box_ != nullptr && filled_discs_check_box_->isChecked();
  const double lift = disc_lift_spin_box_ != nullptr ? disc_lift_spin_box_->value() / 1000.0 : 0.0;
  const double radius = discRadius();
  if (radius > 0.0)
  {
    for (const Eigen::Isometry3d& pose : poses)
    {
      appendDisc(pose, radius, lift, filled, *points, *cells);
    }
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  if (filled)
  {
    poly->SetPolys(cells);
  }
  else
  {
    poly->SetLines(cells);
  }
  return poly;
}

void TPPWidget::styleDiscActor(vtkActor& actor, const unsigned char rgb[3], const float line_width) const
{
  const bool filled = filled_discs_check_box_ != nullptr && filled_discs_check_box_->isChecked();
  actor.GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0);
  actor.GetProperty()->SetLineWidth(line_width);
  actor.GetProperty()->SetOpacity(filled ? kFilledDiscOpacity : 1.0);
  // Un disque plein, au-dessus de la piece, ne doit pas arreter les clics destines a la surface
  actor.PickableOff();
}

QWidget* TPPWidget::buildDiscGroup()
{
  auto* group = new QGroupBox(this);
  translator_.bind([this, group]() { group->setTitle(say("Disques de l'outil", "Tool discs")); });
  filled_discs_check_box_ = new QCheckBox(this);
  filled_discs_check_box_->setChecked(true);
  translator_.bindText(filled_discs_check_box_, { "Disques pleins", "Filled discs" });
  translator_.bindToolTip(filled_discs_check_box_, { "Décoché : seul le bord de chaque disque est dessiné.",
                                                     "Unchecked: only the rim of each disc is drawn." });
  disc_lift_spin_box_ = new QDoubleSpinBox(this);
  disc_lift_spin_box_->setRange(0.0, 50.0);
  disc_lift_spin_box_->setDecimals(1);
  disc_lift_spin_box_->setSingleStep(0.5);
  disc_lift_spin_box_->setValue(kDefaultDiscLiftMm);
  disc_lift_spin_box_->setSuffix(" mm");
  translator_.bindToolTip(disc_lift_spin_box_, { "Décalage des disques le long de l'axe de l'outil, pour l'affichage seulement.",
                                                 "Offset of the discs along the tool axis, for display only." });
  show_uncovered_check_box_ = new QCheckBox(this);
  show_uncovered_check_box_->setChecked(true);
  translator_.bindText(show_uncovered_check_box_, { "Zones non couvertes", "Uncovered areas" });
  auto* layout = new QVBoxLayout(group);
  layout->addWidget(filled_discs_check_box_);
  layout->addWidget(labelledRow({ "Surélévation", "Lift" }, disc_lift_spin_box_));
  layout->addWidget(show_uncovered_check_box_);
  connectDiscGroup();
  return group;
}

void TPPWidget::connectDiscGroup()
{
  connect(filled_discs_check_box_, &QCheckBox::toggled, this, [this](bool) { refreshToolRadiusViews(); });
  connect(disc_lift_spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
    refreshToolRadiusViews();
  });
  connect(show_uncovered_check_box_, &QCheckBox::toggled, this, [this](bool shown) {
    if (uncovered_actor_)
    {
      uncovered_actor_->SetVisibility(shown);
    }
    render();
  });
}

// ---------------------------------------------------------------------------------------------
// Couverture

bool TPPWidget::showsUncovered() const
{
  return show_uncovered_check_box_ != nullptr && show_uncovered_check_box_->isChecked();
}

std::vector<int> TPPWidget::facesOfSurface(const int surface_id) const
{
  std::vector<int> faces;
  for (std::size_t f = 0; f < face_region_.size(); ++f)
  {
    if (face_region_[f] == surface_id)
    {
      faces.push_back(static_cast<int>(f));
    }
  }
  return faces;
}

bool TPPWidget::traceCoverageInput(ToolPath& passes, std::vector<int>& faces) const
{
  // Pendant le trace : le trace de la surface courante, contre les faces de cette surface
  const int surface_id = selectedSurfaceId();
  std::vector<Eigen::Vector3d> normals;
  const std::vector<Eigen::Vector3d> samples = projectedTraceSamples(surface_id, normals);
  if (samples.empty())
  {
    return false;
  }
  ToolPathSegment pass;
  for (const Eigen::Vector3d& sample : samples)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = sample;
    pass.push_back(pose);
  }
  passes = { pass };
  faces = facesOfSurface(surface_id);
  return true;
}

bool TPPWidget::pathCoverageInput(ToolPath& passes, std::vector<int>& faces) const
{
  // Apres generation : le chemin livre, contre les faces de toutes les surfaces tracees
  if (path_edit_ == nullptr || path_edit_->passes().empty())
  {
    return false;
  }
  passes = path_edit_->passes();
  faces.clear();
  for (std::size_t f = 0; f < face_region_.size(); ++f)
  {
    if (face_region_[f] >= 0 && surfaceHasTrace(face_region_[f]))
    {
      faces.push_back(static_cast<int>(f));
    }
  }
  return true;
}

void TPPWidget::updateCoverage()
{
  ToolPath passes;
  std::vector<int> faces;
  std::size_t skirt = 0;
  const double radius = discRadius();
  if (radius <= 0.0 || !selection_poly_ || !coverageInput(passes, faces, skirt) || faces.empty())
  {
    passes.clear();
    faces.clear();
  }
  // Ni le style ni la surelevation des disques ne changent la mesure : elle n'est refaite que si elle le doit
  if (!rememberCoverageInput(passes, faces, radius, skirt))
  {
    return;
  }
  if (faces.empty())
  {
    showUncoveredCells(vtkSmartPointer<vtkPolyData>::New());
    setCoverageInfo(Bilingual());
    return;
  }
  measureCoverage(passes, faces, radius, skirt);
}

void TPPWidget::measureCoverage(const ToolPath& passes, const std::vector<int>& faces, const double radius,
                                const std::size_t skirt)
{
  // Mesure sur des triangles assez petits pour que le resultat ne depende pas du maillage
  const double cell_size = coverageCellSize(*selection_poly_, faces, radius / kCoverageCellsPerRadius);
  const CoverageCells cells = subdivideFaces(*selection_poly_, faces, face_normals_, cell_size);
  std::vector<int> all(cells.centroids.size());
  std::iota(all.begin(), all.end(), 0);
  const CoverageResult result = computeCoverage(passes, cells.centroids, all, radius, skirt);
  showUncoveredCells(uncoveredCellsPoly(cells, result.uncovered_faces));
  setCoverageInfo(coverageText(cells, result.uncovered_faces));
}

bool TPPWidget::coverageInput(ToolPath& passes, std::vector<int>& faces, std::size_t& skirt) const
{
  if (traceCoverageInput(passes, faces))
  {
    skirt = 0;
    return true;
  }
  // L'approche et le retrait du chemin livre ne touchent pas la piece
  skirt = kSkirtPoses;
  return pathCoverageInput(passes, faces);
}

bool TPPWidget::rememberCoverageInput(const ToolPath& passes, const std::vector<int>& faces, const double radius,
                                      const std::size_t skirt)
{
  bool same = coverage_valid_ && faces == coverage_faces_ && radius == coverage_radius_ &&
              skirt == coverage_skirt_ && passes.size() == coverage_passes_.size();
  for (std::size_t p = 0; same && p < passes.size(); ++p)
  {
    same = passes[p].size() == coverage_passes_[p].size();
    for (std::size_t k = 0; same && k < passes[p].size(); ++k)
    {
      // Seules les positions comptent pour la couverture
      same = passes[p][k].translation() == coverage_passes_[p][k].translation();
    }
  }
  coverage_valid_ = true;
  coverage_passes_ = passes;
  coverage_faces_ = faces;
  coverage_radius_ = radius;
  coverage_skirt_ = skirt;
  return !same;
}

void TPPWidget::showUncoveredCells(vtkPolyData* poly)
{
  if (!renderer_)
  {
    return;
  }
  uncovered_actor_ = replaceActor(*renderer_, uncovered_actor_, poly);
  uncovered_actor_->GetProperty()->SetColor(kUncoveredRgb[0] / 255.0, kUncoveredRgb[1] / 255.0, kUncoveredRgb[2] / 255.0);
  uncovered_actor_->GetProperty()->LightingOff();
  // Couche d'affichage : les clics la traversent jusqu'a la piece
  uncovered_actor_->PickableOff();
  uncovered_actor_->SetVisibility(showsUncovered());
}

void TPPWidget::setCoverageInfo(const Bilingual& text)
{
  coverage_text_ = text;
  // L'etiquette est creee apres les outils, qui mesurent deja en s'armant
  if (coverage_label_ != nullptr)
  {
    coverage_label_->setText(translator_.text(text));
  }
}

}  // namespace noether
