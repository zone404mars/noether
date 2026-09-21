/**
 * @file path_edit_widget.h
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
#pragma once

#include <noether_tpp/core/tool_path_recipe.h>

#include <QWidget>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

class QLabel;
class QListWidget;
class QPushButton;

namespace noether
{
/**
 * @brief Panel where the operator retouches the generated tool path.
 * @details The generation is left untouched: what the operator builds here is a ::PathRecipe, and
 * the delivered passes are assembled from it on every change. Nothing but a permutation with
 * grouping and direction can be expressed, so no waypoint can be invented or moved.
 *
 * The widget owns no rendering. It reports every change through ::on_changed and lets its host
 * redraw whatever it shows.
 */
class PathEditWidget : public QWidget
{
public:
  /**
   * @param min_poses Fewest waypoints a delivered pass may carry, from the consumer's contract
   * @param parent Parent widget
   */
  PathEditWidget(std::size_t min_poses, QWidget* parent = nullptr);

  /**
   * @brief Takes a fresh generation. The recipe restarts from the identity and the history is lost.
   * @param generated The generated passes, flattened
   */
  void setGenerated(const std::vector<ToolPathSegment>& generated);

  /**
   * @brief Replaces the recipe with one read from a file.
   * @param content The recipe and the generation it was written against
   * @param tolerance Largest position difference still counted as the same generated pass
   * @throws std::runtime_error if the file does not describe the generation at hand, naming the
   *         passes that differ. A recipe replayed on another generation would weld the wrong ones.
   */
  void applyRecipeFile(const RecipeFile& content, double tolerance);

  /** @brief What every delivered pass takes from the generation */
  const PathRecipe& recipe() const { return recipe_; }
  /** @brief The delivered passes, assembled from the recipe */
  const ToolPath& passes() const { return passes_; }
  /** @brief The generated passes the recipe indexes */
  const std::vector<ToolPathSegment>& generated() const { return generated_; }

  /** @brief Index of the delivered pass selected in the list, negative when none */
  int selectedPass() const;

  /**
   * @brief Selects a delivered pass in the list
   * @param row Index of the pass; out of range clears the selection
   */
  void selectPass(int row);

  /** @brief Moves the selected pass up (@p delta negative) or down in the execution order */
  void moveSelected(int delta);
  /** @brief Travels the selected pass the other way round, sources and their direction both */
  void reverseSelected();
  /** @brief Appends the selected pass to the one before it, dropping the standoff waypoints */
  void weldSelected();
  /**
   * @brief Enchaine toutes les passes livrees en une seule, en partant de la selectionnee
   * @details Voir ::chainPasses : du plus proche au plus proche, retournee quand il le faut.
   */
  void chainFromSelected();
  /** @brief Splits the selected pass back into one pass per generated pass it holds */
  void splitSelected();
  /** @brief Drops the selected pass from the delivery */
  void removeSelected();
  /** @brief Goes back to the recipe as it was before the last change */
  void undo();
  /** @brief Goes back to the generation, every pass delivered alone and in order */
  void reset();

  /**
   * @brief Ne garde que Monter, Descendre, Supprimer et Annuler
   * @details Pour une IHM tout-manuel : le sens, la soudure, le degroupage et l'enchainement
   * n'ont pas cours quand chaque passe est un trace fait a la main.
   */
  void hideAdvancedControls();

  /** @brief What forbids delivering the current recipe, one message per fault */
  std::vector<std::string> faults() const;

  /** @brief Whether the operator has retouched anything, that is whether the recipe still
   *         delivers every generated pass alone and in order, and no pose has been moved */
  bool isRetouched() const;

  /** @brief Poses deplacees a la main, dans l'ordre ou elles l'ont ete */
  const std::vector<PoseMove>& moves() const { return moves_; }
  /** @brief La generation avec les poses deplacees : ce que la recette assemble */
  std::vector<ToolPathSegment> movedGenerated() const;
  /**
   * @brief Memorise l'etat courant pour Annuler, avant une suite de deplacements
   * @details Un tirage a la souris deplace la meme pose des dizaines de fois : un seul point
   * d'annulation pour tout le tirage, pris ici avant le premier deplacement.
   */
  void beginPoseMoves();
  /**
   * @brief Deplace une pose de la generation, sans point d'annulation
   * @details Remplace un deplacement anterieur de la meme pose. Voir ::beginPoseMoves.
   */
  void movePose(const PoseMove& move);

  /** @brief Called after every change to the recipe */
  std::function<void()> on_changed;

  /**
   * @brief Called when the operator picks another pass in the list
   * @details Kept apart from ::on_changed because the recipe has not moved: a host that rebuilds
   * one actor per waypoint has no reason to do it again just because the selection moved.
   */
  std::function<void()> on_selection_changed;

protected:
  /** @brief Stores the current recipe so the next change can be undone */
  void remember();
  /** @brief Reassembles the passes, rebuilds the list, and reports the change */
  void refresh(int row_to_select);
  /** @brief Refreshes what the buttons allow and what the readings say */
  void updateReadings();
  /** @brief Enables only the operations the current selection admits */
  void updateButtons();
  /** @brief Refreshes the weld reading and the fault reading */
  void updateLabels();
  /** @brief The joint that welding pass @p row onto the one before it would create */
  QString describeWeld(int row) const;
  /** @brief One list row: rank, the generated passes it takes, and its waypoint count */
  QString describePass(std::size_t index) const;

  /** @brief Builds the buttons; called once from the constructor */
  void buildControls();
  /** @brief Wires every button to the operation it drives */
  void connectControls();
  /** @brief Rewrites the list of delivered passes and restores a selection on it */
  void rebuildPassList(int row_to_select);

  /** @brief Fewest waypoints a delivered pass may carry, from the consumer's contract */
  std::size_t min_poses_;
  /** @brief The generated passes, flattened, as the last planning produced them */
  std::vector<ToolPathSegment> generated_;
  /** @brief What every delivered pass takes from the generation */
  PathRecipe recipe_;
  /** @brief Poses de la generation deplacees a la main */
  std::vector<PoseMove> moves_;
  /** @brief The delivered passes, reassembled after every change */
  ToolPath passes_;
  /** @brief Un etat de la retouche : la recette et les deplacements */
  struct EditState
  {
    PathRecipe recipe;
    std::vector<PoseMove> moves;
  };
  /** @brief Etats tels qu'ils etaient avant chaque changement, le plus recent en dernier */
  std::vector<EditState> history_;

  /** @brief One row per delivered pass, in execution order */
  QListWidget* pass_list_ = nullptr;
  /** @brief Reading of the joint that welding the selected pass would create */
  QLabel* weld_label_ = nullptr;
  /** @brief Reading of what currently forbids delivering the recipe */
  QLabel* fault_label_ = nullptr;

  QPushButton* up_button_ = nullptr;
  QPushButton* down_button_ = nullptr;
  QPushButton* reverse_button_ = nullptr;
  QPushButton* weld_button_ = nullptr;
  /** @brief Enchaine toutes les passes en une seule depuis la selectionnee */
  QPushButton* chain_button_ = nullptr;
  QPushButton* split_button_ = nullptr;
  QPushButton* remove_button_ = nullptr;
  QPushButton* undo_button_ = nullptr;
  QPushButton* reset_button_ = nullptr;
};

}  // namespace noether
