/**
 * @file path_edit_widget.cpp
 * @copyright Copyright (c) 2026, Zone 404
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <noether_gui/widgets/path_edit_widget.h>

#include <QGridLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace noether
{
namespace
{
/** @brief Metres to millimetres, for the readings shown to the operator */
const double kMetresToMillimetres = 1000.0;

/** @brief Radians to degrees, for the readings shown to the operator */
const double kRadiansToDegrees = 180.0 / M_PI;

/// @brief Builds a button, adds it to a grid, and returns it.
/// @param layout Grid the button joins
/// @param text Label shown on the button
/// @param tip Explanation shown on hover
/// @param row Grid row
/// @param column Grid column
QPushButton* addButton(QGridLayout* layout, const QString& text, const QString& tip, int row, int column)
{
  QPushButton* button = new QPushButton(text, layout->parentWidget());
  button->setToolTip(tip);
  layout->addWidget(button, row, column);
  return button;
}

}  // namespace

PathEditWidget::PathEditWidget(std::size_t min_poses, QWidget* parent)
  : QWidget(parent), min_poses_(min_poses)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("Passes livrees, dans l'ordre d'execution :", this));

  pass_list_ = new QListWidget(this);
  pass_list_->setToolTip("Chaque ligne donne son rang, les passes generees qu'elle enchaine\n"
                         "([3<] = passe 3 parcourue a l'envers), puis son nombre de poses.");
  layout->addWidget(pass_list_);

  buildControls();

  weld_label_ = new QLabel(this);
  weld_label_->setWordWrap(true);
  layout->addWidget(weld_label_);

  fault_label_ = new QLabel(this);
  fault_label_->setWordWrap(true);
  layout->addWidget(fault_label_);

  connect(pass_list_, &QListWidget::currentRowChanged, this, [this](int) {
    updateReadings();
    if (on_selection_changed)
    {
      on_selection_changed();
    }
  });
  refresh(-1);
}

void PathEditWidget::buildControls()
{
  QGridLayout* grid = new QGridLayout();
  static_cast<QVBoxLayout*>(this->layout())->addLayout(grid);

  up_button_ = addButton(grid, "Monter", "Avance la passe dans l'ordre d'execution.", 0, 0);
  down_button_ = addButton(grid, "Descendre", "Recule la passe dans l'ordre d'execution.", 0, 1);
  reverse_button_ = addButton(grid, "Inverser", "Parcourt la passe dans l'autre sens.", 1, 0);
  weld_button_ = addButton(grid,
                           "Souder a la precedente",
                           "Enchaine la passe a celle du dessus, sans decoller entre les deux.\n"
                           "L'ecart et l'angle de la soudure sont affiches sous les boutons.",
                           1,
                           1);
  split_button_ = addButton(grid, "Degrouper", "Rend a chaque passe generee sa ligne.", 2, 0);
  remove_button_ = addButton(grid, "Supprimer", "Retire la passe de la livraison.", 2, 1);
  chain_button_ = addButton(grid,
                            "Enchainer depuis celle-ci",
                            "Soude toutes les passes en une seule, en partant de la selectionnee :\n"
                            "a chaque etape la plus proche, retournee si on l'aborde par sa fin.\n"
                            "L'outil ne decolle plus ; verifier les ecarts avec Degrouper si besoin.",
                            3,
                            0);
  undo_button_ = addButton(grid, "Annuler", "Revient a l'etat precedent la derniere action.", 4, 0);
  reset_button_ = addButton(grid, "Tout reinitialiser", "Revient a la generation, sans aucune retouche.", 4, 1);

  connectControls();
}

void PathEditWidget::hideAdvancedControls()
{
  for (QPushButton* button : { reverse_button_, weld_button_, split_button_, chain_button_, reset_button_ })
  {
    button->hide();
  }
  weld_label_->hide();
}

void PathEditWidget::connectControls()
{
  connect(up_button_, &QPushButton::clicked, this, [this](bool) { moveSelected(-1); });
  connect(down_button_, &QPushButton::clicked, this, [this](bool) { moveSelected(1); });
  connect(reverse_button_, &QPushButton::clicked, this, [this](bool) { reverseSelected(); });
  connect(weld_button_, &QPushButton::clicked, this, [this](bool) { weldSelected(); });
  connect(chain_button_, &QPushButton::clicked, this, [this](bool) { chainFromSelected(); });
  connect(split_button_, &QPushButton::clicked, this, [this](bool) { splitSelected(); });
  connect(remove_button_, &QPushButton::clicked, this, [this](bool) { removeSelected(); });
  connect(undo_button_, &QPushButton::clicked, this, [this](bool) { undo(); });
  connect(reset_button_, &QPushButton::clicked, this, [this](bool) { reset(); });
}

void PathEditWidget::setGenerated(const std::vector<ToolPathSegment>& generated)
{
  generated_ = generated;
  recipe_ = identityRecipe(generated_.size());
  moves_.clear();
  // Une nouvelle generation renumerote les passes : l'historique ne designerait plus rien.
  history_.clear();
  refresh(generated_.empty() ? -1 : 0);
}

void PathEditWidget::applyRecipeFile(const RecipeFile& content, double tolerance)
{
  const std::vector<std::size_t> stale =
      staleSegments(content.source, fingerprintSegments(generated_), tolerance);
  if (!stale.empty())
  {
    QString passes;
    for (std::size_t i = 0; i < stale.size(); ++i)
    {
      passes += (i == 0 ? "" : ", ") + QString::number(stale[i]);
    }
    throw std::runtime_error(
        QString("La recette a ete ecrite sur une autre generation : les passes %1 n'y correspondent "
                "plus. Ses indices designeraient d'autres passes, la rejouer souderait les mauvaises.")
            .arg(passes)
            .toStdString());
  }
  remember();
  recipe_ = content.recipe;
  moves_ = content.moves;
  refresh(0);
}

int PathEditWidget::selectedPass() const
{
  const int row = pass_list_->currentRow();
  return row < static_cast<int>(passes_.size()) ? row : -1;
}

std::vector<std::string> PathEditWidget::faults() const
{
  if (generated_.empty())
  {
    return std::vector<std::string>();
  }
  return validateRecipe(generated_, recipe_, min_poses_);
}

bool PathEditWidget::isRetouched() const
{
  const PathRecipe untouched = identityRecipe(generated_.size());
  if (recipe_.size() != untouched.size() || !moves_.empty())
  {
    return true;
  }
  for (std::size_t p = 0; p < recipe_.size(); ++p)
  {
    if (recipe_[p].size() != 1 || recipe_[p][0].segment != p || recipe_[p][0].reversed)
    {
      return true;
    }
  }
  return false;
}

void PathEditWidget::selectPass(int row)
{
  pass_list_->setCurrentRow(row);
}

void PathEditWidget::remember() { history_.push_back(EditState{ recipe_, moves_ }); }

std::vector<ToolPathSegment> PathEditWidget::movedGenerated() const { return applyPoseMoves(generated_, moves_); }

void PathEditWidget::beginPoseMoves() { remember(); }

void PathEditWidget::movePose(const PoseMove& move)
{
  // Une pose deja deplacee reprend le nouveau deplacement a la place de l'ancien
  for (PoseMove& existing : moves_)
  {
    if (existing.segment == move.segment && existing.index == move.index)
    {
      existing.pose = move.pose;
      refresh(selectedPass());
      return;
    }
  }
  moves_.push_back(move);
  refresh(selectedPass());
}

void PathEditWidget::moveSelected(int delta)
{
  const int row = selectedPass();
  const int target = row + delta;
  if (row < 0 || target < 0 || target >= static_cast<int>(recipe_.size()))
  {
    return;
  }
  remember();
  std::swap(recipe_[static_cast<std::size_t>(row)], recipe_[static_cast<std::size_t>(target)]);
  refresh(target);
}

void PathEditWidget::reverseSelected()
{
  const int row = selectedPass();
  if (row < 0)
  {
    return;
  }
  remember();
  recipe_[static_cast<std::size_t>(row)] = reverseSources(recipe_[static_cast<std::size_t>(row)]);
  refresh(row);
}

void PathEditWidget::chainFromSelected()
{
  const int row = selectedPass();
  if (row < 0 || recipe_.size() < 2)
  {
    return;
  }
  remember();
  recipe_ = chainPasses(generated_, recipe_, static_cast<std::size_t>(row));
  refresh(0);
}

void PathEditWidget::weldSelected()
{
  const int row = selectedPass();
  if (row < 1)
  {
    return;
  }
  remember();
  std::vector<SourceSegment>& previous = recipe_[static_cast<std::size_t>(row) - 1];
  const std::vector<SourceSegment>& current = recipe_[static_cast<std::size_t>(row)];
  previous.insert(previous.end(), current.begin(), current.end());
  recipe_.erase(recipe_.begin() + row);
  refresh(row - 1);
}

void PathEditWidget::splitSelected()
{
  const int row = selectedPass();
  if (row < 0 || recipe_[static_cast<std::size_t>(row)].size() < 2)
  {
    return;
  }
  remember();
  const std::vector<SourceSegment> sources = recipe_[static_cast<std::size_t>(row)];
  PathRecipe pieces;
  for (std::size_t i = 0; i < sources.size(); ++i)
  {
    pieces.push_back(std::vector<SourceSegment>(1, sources[i]));
  }
  recipe_.erase(recipe_.begin() + row);
  recipe_.insert(recipe_.begin() + row, pieces.begin(), pieces.end());
  refresh(row);
}

void PathEditWidget::removeSelected()
{
  const int row = selectedPass();
  if (row < 0)
  {
    return;
  }
  remember();
  recipe_.erase(recipe_.begin() + row);
  refresh(std::min(row, static_cast<int>(recipe_.size()) - 1));
}

void PathEditWidget::undo()
{
  if (history_.empty())
  {
    return;
  }
  recipe_ = history_.back().recipe;
  moves_ = history_.back().moves;
  history_.pop_back();
  refresh(selectedPass());
}

void PathEditWidget::reset()
{
  remember();
  recipe_ = identityRecipe(generated_.size());
  moves_.clear();
  refresh(generated_.empty() ? -1 : 0);
}

void PathEditWidget::refresh(int row_to_select)
{
  try
  {
    passes_ = assemblePasses(movedGenerated(), recipe_);
  }
  catch (const std::exception&)
  {
    // Aucun bouton ne peut produire une recette structurellement fausse. Si cela arrive
    // quand meme, mieux vaut une liste vide qu'une exception traversant la boucle Qt.
    passes_.clear();
  }

  rebuildPassList(row_to_select);
  updateReadings();
  if (on_changed)
  {
    on_changed();
  }
}

void PathEditWidget::rebuildPassList(int row_to_select)
{
  // Les signaux sont coupes le temps de la reconstruction : chaque ligne ajoutee deplacerait
  // sinon la selection, et l'hote redessinerait le surlignage a chaque ligne.
  pass_list_->blockSignals(true);
  pass_list_->clear();
  for (std::size_t i = 0; i < passes_.size(); ++i)
  {
    pass_list_->addItem(describePass(i));
  }
  if (row_to_select >= 0 && row_to_select < pass_list_->count())
  {
    pass_list_->setCurrentRow(row_to_select);
  }
  pass_list_->blockSignals(false);
}

void PathEditWidget::updateReadings()
{
  updateButtons();
  updateLabels();
}

void PathEditWidget::updateButtons()
{
  const int row = selectedPass();
  const bool has_row = row >= 0;
  const int count = static_cast<int>(passes_.size());

  up_button_->setEnabled(has_row && row > 0);
  down_button_->setEnabled(has_row && row + 1 < count);
  reverse_button_->setEnabled(has_row);
  weld_button_->setEnabled(has_row && row > 0);
  chain_button_->setEnabled(has_row && count > 1);
  split_button_->setEnabled(has_row && recipe_[static_cast<std::size_t>(row)].size() > 1);
  remove_button_->setEnabled(has_row);
  undo_button_->setEnabled(!history_.empty());
  reset_button_->setEnabled(!generated_.empty());
}

void PathEditWidget::updateLabels()
{
  weld_label_->setText(describeWeld(selectedPass()));

  const std::vector<std::string> problems = faults();
  if (problems.empty())
  {
    fault_label_->setText(QString());
    return;
  }
  QString text = "A corriger avant d'enregistrer :";
  for (std::size_t i = 0; i < problems.size(); ++i)
  {
    text += "\n- " + QString::fromStdString(problems[i]);
  }
  fault_label_->setText(text);
}

QString PathEditWidget::describeWeld(int row) const
{
  if (row < 1)
  {
    return QString();
  }
  const WeldMetrics metrics =
      computeWeldMetrics(passes_[static_cast<std::size_t>(row) - 1], passes_[static_cast<std::size_t>(row)]);
  if (metrics.gap < 0.0)
  {
    return "Soudure avec la precedente : une des deux passes est trop courte pour etre soudee.";
  }
  // L'ecart et l'angle sont montres AVANT l'action : souder deux zones opposees de la piece
  // est parfois voulu, ce n'est pas a la GUI de trancher a la place de l'operateur.
  return QString("Soudure avec la precedente : ecart %1 mm, angle %2 deg.")
      .arg(metrics.gap * kMetresToMillimetres, 0, 'f', 1)
      .arg(metrics.angle * kRadiansToDegrees, 0, 'f', 1);
}

QString PathEditWidget::describePass(std::size_t index) const
{
  QString sources;
  for (std::size_t s = 0; s < recipe_[index].size(); ++s)
  {
    sources += QString("[%1%2]")
                   .arg(static_cast<int>(recipe_[index][s].segment))
                   .arg(recipe_[index][s].reversed ? "<" : "");
  }
  return QString("%1  %2  %3 poses")
      .arg(static_cast<int>(index) + 1, 2)
      .arg(sources, -20)
      .arg(static_cast<int>(passes_[index].size()), 4);
}

}  // namespace noether
