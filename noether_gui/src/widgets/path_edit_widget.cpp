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

namespace noether
{
namespace
{
/// @brief Cree un bouton, le place dans une grille et le rend.
/// @param layout Grille qui recoit le bouton
/// @param row Ligne de la grille
/// @param column Colonne de la grille
QPushButton* addButton(QGridLayout* layout, const int row, const int column)
{
  QPushButton* button = new QPushButton(layout->parentWidget());
  layout->addWidget(button, row, column);
  return button;
}

}  // namespace

PathEditWidget::PathEditWidget(const std::size_t min_poses, QWidget* parent)
  : QWidget(parent), min_poses_(min_poses), path_(min_poses)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  list_label_ = new QLabel(this);
  layout->addWidget(list_label_);
  pass_list_ = new QListWidget(this);
  layout->addWidget(pass_list_);
  buildControls();
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
  up_button_ = addButton(grid, 0, 0);
  down_button_ = addButton(grid, 0, 1);
  remove_button_ = addButton(grid, 1, 0);
  undo_button_ = addButton(grid, 2, 0);
  redo_button_ = addButton(grid, 2, 1);
  // Textes francais tant que ::bindTexts n'a pas lie la langue
  list_label_->setText(QString::fromUtf8("Passes livrées, dans l'ordre d'exécution :"));
  up_button_->setText("Monter");
  down_button_->setText("Descendre");
  remove_button_->setText("Supprimer");
  undo_button_->setText("Annuler");
  redo_button_->setText(QString::fromUtf8("Rétablir"));
  connect(up_button_, &QPushButton::clicked, this, [this](bool) { moveSelected(-1); });
  connect(down_button_, &QPushButton::clicked, this, [this](bool) { moveSelected(1); });
  connect(remove_button_, &QPushButton::clicked, this, [this](bool) { removeSelected(); });
  connect(undo_button_, &QPushButton::clicked, this, [this](bool) { undo(); });
  connect(redo_button_, &QPushButton::clicked, this, [this](bool) { redo(); });
}

void PathEditWidget::bindTexts(Translator& translator)
{
  translator.bindText(list_label_,
                      { "Passes livrées, dans l'ordre d'exécution :", "Delivered passes, in execution order:" });
  translator.bindText(up_button_, { "Monter", "Move up" });
  translator.bindToolTip(up_button_, { "Avance la passe dans l'ordre d'exécution.",
                                       "Moves the pass earlier in the execution order." });
  translator.bindText(down_button_, { "Descendre", "Move down" });
  translator.bindToolTip(down_button_, { "Recule la passe dans l'ordre d'exécution.",
                                         "Moves the pass later in the execution order." });
  translator.bindText(remove_button_, { "Supprimer", "Remove" });
  translator.bindToolTip(remove_button_,
                         { "Retire la passe du chemin livré.", "Drops the pass from the delivered path." });
  translator.bindText(undo_button_, { "Annuler", "Undo" });
  translator.bindToolTip(undo_button_, { "Défait la dernière action.", "Undoes the last action." });
  translator.bindText(redo_button_, { "Rétablir", "Redo" });
  translator.bindToolTip(redo_button_, { "Refait la dernière action défaite.", "Redoes the last undone action." });
  // Les lignes et les defauts sont calcules : ils sont recalcules au changement de langue
  translator_ = &translator;
  translator.bind([this]() { rebuildPassList(selectedPass()); updateReadings(); });
}

QString PathEditWidget::say(const QString& french, const QString& english) const
{
  return translator_ != nullptr ? translator_->text({ french, english }) : french;
}

void PathEditWidget::setPasses(const std::vector<EditedPass>& passes)
{
  path_.reset(passes);
  notifyRenumbered();
  refresh(passes.empty() ? -1 : 0);
}

ToolPath PathEditWidget::passes() const
{
  ToolPath poses;
  for (const EditedPass& pass : path_.passes())
  {
    poses.push_back(pass.poses);
  }
  return poses;
}

int PathEditWidget::selectedPass() const
{
  const int row = pass_list_->currentRow();
  return row < static_cast<int>(path_.passes().size()) ? row : -1;
}

void PathEditWidget::selectPass(const int row) { pass_list_->setCurrentRow(row); }

void PathEditWidget::moveSelected(const int delta)
{
  const int row = selectedPass();
  const int target = row + delta;
  if (row < 0 || target < 0 || target >= static_cast<int>(path_.passes().size()))
  {
    return;
  }
  path_.movePass(static_cast<std::size_t>(row), static_cast<std::size_t>(target));
  notifyRenumbered();
  // Le choix suit la passe deplacee, sinon le clic suivant porterait sur une autre
  refresh(target);
}

void PathEditWidget::removeSelected()
{
  const int row = selectedPass();
  if (row < 0)
  {
    return;
  }
  path_.removePass(static_cast<std::size_t>(row));
  notifyRenumbered();
  refresh(std::min(row, static_cast<int>(path_.passes().size()) - 1));
}

void PathEditWidget::undo()
{
  path_.undo();
  notifyRenumbered();
  refresh(selectedPass());
}

void PathEditWidget::redo()
{
  path_.redo();
  notifyRenumbered();
  refresh(selectedPass());
}

std::optional<std::size_t> PathEditWidget::passTooShortAfterRemoval(const std::vector<PoseRef>& poses) const
{
  return path_.passTooShortAfterRemoval(poses);
}

void PathEditWidget::removePoses(const std::vector<PoseRef>& poses)
{
  path_.removePoses(poses);
  refresh(selectedPass());
}

void PathEditWidget::insertPoses(const std::vector<GapInsertion>& insertions)
{
  if (insertions.empty())
  {
    return;
  }
  path_.insertPoses(insertions);
  refresh(selectedPass());
}

void PathEditWidget::beginDrag() { path_.beginAction(); }

void PathEditWidget::setPose(const PoseRef& ref, const Eigen::Isometry3d& pose) { path_.setPose(ref, pose); }

void PathEditWidget::endDrag() { refresh(selectedPass()); }

void PathEditWidget::notifyRenumbered()
{
  if (on_poses_renumbered)
  {
    on_poses_renumbered();
  }
}

std::vector<std::string> PathEditWidget::faults() const
{
  std::vector<std::string> problems;
  for (std::size_t p = 0; p < path_.passes().size(); ++p)
  {
    const std::size_t count = path_.passes()[p].poses.size();
    if (count < min_poses_)
    {
      problems.push_back(say("passe %1 : %2 poses, il en faut au moins %3", "pass %1: %2 poses, at least %3 needed")
                             .arg(p + 1)
                             .arg(count)
                             .arg(min_poses_)
                             .toStdString());
    }
  }
  return problems;
}

void PathEditWidget::refresh(const int row_to_select)
{
  rebuildPassList(row_to_select);
  updateReadings();
  if (on_changed)
  {
    on_changed();
  }
}

void PathEditWidget::rebuildPassList(const int row_to_select)
{
  // Signaux coupes le temps de la reconstruction : chaque ligne ajoutee deplacerait le choix
  pass_list_->blockSignals(true);
  pass_list_->clear();
  for (std::size_t i = 0; i < path_.passes().size(); ++i)
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
  const int row = selectedPass();
  const int count = static_cast<int>(path_.passes().size());
  up_button_->setEnabled(row > 0);
  down_button_->setEnabled(row >= 0 && row + 1 < count);
  remove_button_->setEnabled(row >= 0);
  undo_button_->setEnabled(path_.canUndo());
  redo_button_->setEnabled(path_.canRedo());
  const std::vector<std::string> problems = faults();
  QString text = problems.empty() ? QString() : say("À corriger avant d'exporter :", "To fix before exporting:");
  for (const std::string& problem : problems)
  {
    text += "\n- " + QString::fromStdString(problem);
  }
  fault_label_->setText(text);
}

QString PathEditWidget::describePass(const std::size_t index) const
{
  return say("%1  %2 poses", "%1  %2 poses").arg(static_cast<int>(index) + 1, 2).arg(
      static_cast<int>(path_.passes()[index].poses.size()), 4);
}

}  // namespace noether
