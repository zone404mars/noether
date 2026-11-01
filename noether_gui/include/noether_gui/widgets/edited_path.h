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
#pragma once

#include <noether_gui/widgets/pass_surface.h>
#include <noether_tpp/core/types.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace noether
{
/** @brief Une passe du chemin livre : ses poses, approche et retrait compris, et sa surface */
struct EditedPass
{
  /** @brief Approche, poses de contact, retrait, dans l'ordre de parcours */
  ToolPathSegment poses;
  /** @brief Surface qui a donne la passe ; nulle pour un chemin repris (spec N23) */
  std::shared_ptr<const PassSurface> surface;
};

/** @brief Une pose du chemin livre, reperee par sa passe et son rang dans la passe */
struct PoseRef
{
  /** @brief Rang de la passe dans le chemin livre */
  std::size_t pass = 0;
  /** @brief Rang de la pose dans la passe, approche comprise (l'approche est le rang 0) */
  std::size_t index = 0;
};

/** @brief Deux reperes designent la meme pose */
inline bool operator==(const PoseRef& a, const PoseRef& b) { return a.pass == b.pass && a.index == b.index; }

/** @brief Poses a inserer dans l'ecart qui suit une pose */
struct GapInsertion
{
  /** @brief Rang de la passe dans le chemin livre */
  std::size_t pass = 0;
  /** @brief Rang de la pose apres laquelle les poses s'inserent */
  std::size_t after = 0;
  /** @brief Poses inserees, dans le sens de la passe */
  ToolPathSegment poses;
};

/**
 * @brief Chemin livre edite : une liste de passes modifiee directement, avec Annuler et Retablir
 * sur une pile de copies (spec §18).
 * @details Chaque passe porte une approche en tete et un retrait en queue. Une action empile une
 * copie de la liste pour Annuler et vide la pile de Retablir. Sans Qt, pour se tester sans fenetre.
 */
class EditedPath
{
public:
  /**
   * @param min_poses Nombre minimal de poses d'une passe, approche et retrait compris
   */
  explicit EditedPath(std::size_t min_poses);

  /** @brief Repart d'une nouvelle generation ou d'un chemin repris : piles videes, chemin non modifie */
  void reset(const std::vector<EditedPass>& passes);

  /** @brief Les passes, dans l'ordre d'execution */
  const std::vector<EditedPass>& passes() const { return passes_; }
  /** @brief Vrai quand une action reste a annuler depuis la derniere generation ou reprise */
  bool modified() const { return !undo_.empty(); }
  /** @brief Vrai quand Annuler a quelque chose a defaire */
  bool canUndo() const { return !undo_.empty(); }
  /** @brief Vrai quand Retablir a quelque chose a refaire */
  bool canRedo() const { return !redo_.empty(); }

  /** @brief Revient a la liste d'avant la derniere action */
  void undo();
  /** @brief Refait la derniere action annulee */
  void redo();

  /**
   * @brief Commence une action : copie la liste pour Annuler et vide Retablir.
   * @details Les operations ci-dessous l'appellent elles-memes, sauf ::setPose : un glisser
   * deplace les memes poses des dizaines de fois et ne fait qu'une action.
   */
  void beginAction();

  /**
   * @brief Remplace une pose, sans point d'annulation (voir ::beginAction)
   * @throws std::out_of_range si la pose n'existe pas
   */
  void setPose(const PoseRef& ref, const Eigen::Isometry3d& pose);

  /**
   * @brief Deplace une passe dans l'ordre d'execution
   * @throws std::out_of_range si l'un des deux rangs n'existe pas
   */
  void movePass(std::size_t from, std::size_t to);

  /**
   * @brief Retire une passe du chemin livre
   * @throws std::out_of_range si la passe n'existe pas
   */
  void removePass(std::size_t index);

  /**
   * @brief Insere des poses dans des ecarts, en une seule action
   * @throws std::out_of_range si une insertion designe une pose absente
   */
  void insertPoses(const std::vector<GapInsertion>& insertions);

  /**
   * @brief La premiere passe qui tomberait sous le minimum de poses si on retirait `poses`
   * @return Le rang de cette passe, ou rien si la suppression est permise
   */
  std::optional<std::size_t> passTooShortAfterRemoval(const std::vector<PoseRef>& poses) const;

  /**
   * @brief Supprime des poses de contact, en une seule action (spec §20.1)
   * @details La pose de contact qui precede une pose supprimee vise sa nouvelle suivante ; une
   * approche ou un retrait suit la nouvelle premiere ou derniere pose de contact, a la meme place
   * dans son repere.
   * @throws std::invalid_argument si la suppression fait passer une passe sous le minimum, ou
   *         designe une approche ou un retrait
   */
  void removePoses(const std::vector<PoseRef>& poses);

private:
  /** @brief Nombre minimal de poses d'une passe, approche et retrait compris */
  std::size_t min_poses_;
  /** @brief Les passes, dans l'ordre d'execution */
  std::vector<EditedPass> passes_;
  /** @brief Listes d'avant chaque action, la plus recente en dernier */
  std::vector<std::vector<EditedPass>> undo_;
  /** @brief Listes defaites par Annuler, la plus recente en dernier */
  std::vector<std::vector<EditedPass>> redo_;
};

/**
 * @brief Passe privee de certaines poses de contact (spec §20.1)
 * @param pass Approche, au moins deux poses de contact, retrait
 * @param removed Rangs des poses de contact a retirer, ni l'approche ni le retrait
 * @return La passe sans ces poses : la pose qui precede chaque trou vise sa nouvelle suivante, ou
 * garde le sens de son dernier troncon si elle est devenue la derniere ; l'approche et le retrait
 * ne bougent pas, sauf si leur pose de contact disparait : ils sont alors replaces devant la
 * nouvelle, a la meme place dans son repere
 * @throws std::invalid_argument s'il resterait moins de deux poses de contact
 */
ToolPathSegment withoutContactPoses(const ToolPathSegment& pass, const std::vector<std::size_t>& removed);

}  // namespace noether
