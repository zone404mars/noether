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

#include <noether_gui/widgets/edited_path.h>

#include <Eigen/Geometry>
#include <optional>
#include <utility>
#include <vector>

namespace noether
{
/** @brief Facon de choisir combien de points ajouter dans un ecart (spec §21) */
enum class DensifyMode
{
  /** @brief Assez de points pour que l'espacement reste sous un pas cible */
  Step,
  /** @brief Le meme nombre de points dans chaque ecart */
  Count
};

/** @brief Reglages de Densifier */
struct DensifySettings
{
  /** @brief Mode de calcul du nombre de points */
  DensifyMode mode;
  /** @brief Pas cible, en metres, pour le mode Step */
  double step;
  /** @brief Nombre de points par ecart, pour le mode Count */
  int count;
};

/**
 * @brief Nombre de points a ajouter dans un ecart de longueur `length` (spec §21)
 * @return `ceil(length / step) - 1` en mode Step, `count` en mode Count ; jamais negatif
 */
int gapPointCount(double length, const DensifySettings& settings);

/** @brief Ce que Densifier ferait entre deux poses */
struct DensifyPlan
{
  /** @brief Une insertion par ecart qui recoit des poses */
  std::vector<GapInsertion> insertions;
  /** @brief Nombre d'ecarts sautes faute de triangle valable ou parce que leurs axes z sont opposes */
  int skipped = 0;
};

/**
 * @brief Poses a ajouter dans chaque ecart de A a B, posees sur la surface de la passe (§21, §21.1)
 * @param passes Le chemin livre
 * @param a Premiere pose, dans le sens de la passe
 * @param b Seconde pose, de la meme passe, apres `a`
 * @throws std::invalid_argument si `a` et `b` ne sont pas deux poses d'une meme passe dans cet ordre,
 *         ou si la passe n'a pas de surface
 */
DensifyPlan planDensify(const std::vector<EditedPass>& passes, const PoseRef& a, const PoseRef& b,
                        const DensifySettings& settings);

/** @brief Rang d'une pose apres des insertions dans sa passe */
PoseRef shiftedByInsertions(const PoseRef& ref, const std::vector<GapInsertion>& insertions);

/**
 * @brief Selection de points de l'outil Editer les points (spec §19.3)
 * @details Un changement de sélection n'est pas une action : il ne passe pas par Annuler.
 */
class PointSelection
{
public:
  /**
   * @brief Clic sans glisser
   * @param point Point saisi, ou rien pour un clic dans le vide
   * @param ctrl Vrai si Ctrl etait tenu a l'appui
   */
  void click(const std::optional<PoseRef>& point, bool ctrl);
  /** @brief Debut d'un glisser sur un point : s'il n'est pas choisi, il devient la seule selection */
  void beginDrag(const PoseRef& pressed);
  /** @brief Vide la selection */
  void clear() { points_.clear(); }
  /** @brief Remplace la selection */
  void set(const std::vector<PoseRef>& points) { points_ = points; }
  /** @brief Vrai si le point est choisi */
  bool contains(const PoseRef& point) const;
  /** @brief Les points choisis, dans l'ordre ou ils l'ont ete */
  const std::vector<PoseRef>& points() const { return points_; }
  /** @brief Vrai si rien n'est choisi */
  bool empty() const { return points_.empty(); }
  /**
   * @brief A et B pour Densifier : exactement deux poses d'une meme passe
   * @return A puis B dans le sens de la passe, ou rien
   */
  std::optional<std::pair<PoseRef, PoseRef>> densifyEnds() const;

private:
  /** @brief Les points choisis, dans l'ordre ou ils l'ont ete */
  std::vector<PoseRef> points_;
};

/**
 * @brief Passe apres un glisser de certaines de ses poses de contact (spec §19.3, §21.1)
 * @details Chaque pose est translatee de `delta` puis posee sur la surface de la passe, la normale
 * attendue etant son axe z d'avant le glisser ; z prend la normale du triangle retenu, x garde
 * son sens de marche. Une pose sans triangle valable reste a sa place. L'approche ou le retrait
 * suit la premiere ou la derniere pose de contact, a la meme place dans son repere.
 * @param before La passe au debut du glisser
 * @param indices Rangs des poses de contact deplacees
 * @param delta Deplacement du curseur sur la piece depuis l'appui, en metres
 * @throws std::invalid_argument si la passe n'a pas de surface, ou si un rang n'est pas une pose de contact
 */
ToolPathSegment draggedPass(const EditedPass& before, const std::vector<std::size_t>& indices,
                            const Eigen::Vector3d& delta);

}  // namespace noether
