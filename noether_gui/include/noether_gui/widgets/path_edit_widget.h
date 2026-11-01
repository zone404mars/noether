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

#include <noether_gui/widgets/edited_path.h>
#include <noether_gui/widgets/translator.h>

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
 * @brief Panneau ou l'operateur edite le chemin livre : ordre des passes, suppression d'une
 * passe, Annuler et Retablir (spec §18).
 * @details Le chemin livre est une liste de passes modifiee directement (::EditedPath). Le
 * panneau ne dessine rien : il signale chaque changement par ::on_changed et laisse son hote
 * redessiner.
 */
class PathEditWidget : public QWidget
{
public:
  /**
   * @param min_poses Nombre minimal de poses d'une passe livree, approche et retrait compris
   * @param parent Widget parent
   */
  PathEditWidget(std::size_t min_poses, QWidget* parent = nullptr);

  /** @brief Repart d'une nouvelle generation ou d'un chemin repris : piles videes, chemin non modifie */
  void setPasses(const std::vector<EditedPass>& passes);

  /** @brief Les passes livrees, avec leur surface, dans l'ordre d'execution */
  const std::vector<EditedPass>& editedPasses() const { return path_.passes(); }
  /** @brief Les passes livrees, poses seules, dans l'ordre d'execution */
  ToolPath passes() const;
  /** @brief Vrai quand une action reste a annuler depuis la derniere generation ou reprise */
  bool isModified() const { return path_.modified(); }

  /** @brief Rang de la passe choisie dans la liste, negatif sans choix */
  int selectedPass() const;
  /**
   * @brief Choisit une passe dans la liste
   * @param row Rang de la passe ; hors bornes, la liste n'a plus de choix
   */
  void selectPass(int row);

  /** @brief Avance (@p delta negatif) ou recule la passe choisie dans l'ordre d'execution */
  void moveSelected(int delta);
  /** @brief Retire la passe choisie du chemin livre */
  void removeSelected();
  /** @brief Defait la derniere action */
  void undo();
  /** @brief Refait la derniere action defaite */
  void redo();

  /** @brief Voir ::EditedPath::passTooShortAfterRemoval */
  std::optional<std::size_t> passTooShortAfterRemoval(const std::vector<PoseRef>& poses) const;
  /** @brief Supprime des poses de contact en une action ; voir ::EditedPath::removePoses */
  void removePoses(const std::vector<PoseRef>& poses);
  /** @brief Insere des poses dans des ecarts en une action ; voir ::EditedPath::insertPoses */
  void insertPoses(const std::vector<GapInsertion>& insertions);
  /** @brief Commence un glisser : une seule action pour tous les deplacements qui suivent */
  void beginDrag();
  /** @brief Deplace une pose pendant un glisser, sans point d'annulation */
  void setPose(const PoseRef& ref, const Eigen::Isometry3d& pose);
  /** @brief Termine un glisser : la liste et les boutons sont mis a jour une fois */
  void endDrag();

  /**
   * @brief Lie les textes du panneau a la langue de l'interface
   * @param translator Langue de l'interface ; doit vivre plus longtemps que ce widget
   */
  void bindTexts(Translator& translator);

  /** @brief Ce qui interdit de livrer le chemin, un message par defaut */
  std::vector<std::string> faults() const;

  /** @brief Appele apres chaque changement du chemin livre */
  std::function<void()> on_changed;
  /**
   * @brief Appele quand les rangs des poses ne designent plus les memes poses : nouvelles passes,
   * Annuler, Retablir, Monter, Descendre, Supprimer une passe. Appele avant ::on_changed.
   */
  std::function<void()> on_poses_renumbered;
  /**
   * @brief Appele quand l'operateur choisit une autre passe dans la liste
   * @details A part de ::on_changed : le chemin n'a pas change, l'hote n'a pas a tout redessiner.
   */
  std::function<void()> on_selection_changed;

protected:
  /** @brief Reconstruit la liste, met a jour les boutons et previent l'hote */
  void refresh(int row_to_select);
  /** @brief Appelle ::on_poses_renumbered s'il est branche */
  void notifyRenumbered();
  /** @brief Reecrit la liste des passes et y retablit un choix */
  void rebuildPassList(int row_to_select);
  /** @brief Active seulement les boutons que le choix courant permet, et ecrit les defauts */
  void updateReadings();
  /** @brief Une ligne de la liste : rang et nombre de poses */
  QString describePass(std::size_t index) const;
  /** @brief Texte dans la langue de l'interface ; en francais sans ::bindTexts */
  QString say(const QString& french, const QString& english) const;
  /** @brief Construit les boutons ; appele une fois par le constructeur */
  void buildControls();

  /** @brief Nombre minimal de poses d'une passe livree, approche et retrait compris */
  std::size_t min_poses_;
  /** @brief Le chemin livre et ses piles d'Annuler et de Retablir */
  EditedPath path_;

  /** @brief Une ligne par passe livree, dans l'ordre d'execution */
  QListWidget* pass_list_ = nullptr;
  /** @brief Ce qui interdit de livrer le chemin */
  QLabel* fault_label_ = nullptr;
  /** @brief Titre de la liste des passes */
  QLabel* list_label_ = nullptr;
  /** @brief Langue des textes calcules, fixee par ::bindTexts ; nulle : textes en francais */
  const Translator* translator_ = nullptr;
  /** @brief Avance la passe choisie */
  QPushButton* up_button_ = nullptr;
  /** @brief Recule la passe choisie */
  QPushButton* down_button_ = nullptr;
  /** @brief Retire la passe choisie */
  QPushButton* remove_button_ = nullptr;
  /** @brief Defait la derniere action */
  QPushButton* undo_button_ = nullptr;
  /** @brief Refait la derniere action defaite */
  QPushButton* redo_button_ = nullptr;
};

}  // namespace noether
