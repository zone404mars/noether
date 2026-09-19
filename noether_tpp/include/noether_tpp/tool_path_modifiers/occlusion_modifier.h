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

#include <noether_tpp/core/tool_path_modifier.h>

#include <pcl/PolygonMesh.h>
#include <vtkCellLocator.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Retire les points de passage que l'outil ne peut pas atteindre sans traverser le maillage.
 * @details Le planificateur travaille sur la seule surface selectionnee : il ne voit pas ce qui la
 * recouvre ailleurs dans la piece (retour de bord, raidisseur, autre solide d'un assemblage). Une
 * passe generee sous un tel recouvrement demanderait a l'outil de passer a travers pour atteindre
 * la surface. Ce modificateur recoit le maillage COMPLET et lance, depuis chaque pose, un rayon le
 * long de l'axe outil (+z de la pose, vers l'outil) sur la longueur de degagement. Un rayon qui
 * rencontre le maillage signale une pose inaccessible.
 *
 * Avec un rayon d'outil non nul, des rayons supplementaires partent du bord du disque : une pose
 * dont l'axe est libre mais dont le disque passerait sous un recouvrement est retiree aussi. Des
 * rayons radiaux, du centre vers chaque point du bord et un peu au-dessus de la surface, detectent
 * en plus les parois laterales : une paroi mince a moins d'un rayon de la pose n'est traversee par
 * aucun rayon vertical, mais elle coupe le rayon radial.
 *
 * Les poses retirees coupent leur segment : chaque suite continue de poses accessibles devient un
 * segment a part entiere, et les suites plus courtes que le minimum demande sont abandonnees, une
 * passe d'un ou deux points n'ayant pas de sens. Les modificateurs d'approche et de retrait doivent
 * donc s'appliquer APRES celui-ci, pour que chaque segment issu de la coupe recoive les siens.
 */
class OcclusionModifier : public OneTimeToolPathModifier
{
public:
  /**
   * @param mesh Maillage complet de la piece, celui charge dans la vue, pas la sous-maille planifiee
   * @param clearance Longueur, en metres, sur laquelle l'axe outil doit etre libre au-dessus de la pose
   * @param tool_radius Rayon du disque en metres ; 0 ne teste que l'axe
   * @param ring_samples Nombre de rayons repartis sur le bord du disque quand tool_radius > 0
   * @param min_segment_poses Nombre de poses en dessous duquel une suite accessible est abandonnee
   * @throws std::invalid_argument si clearance <= 0, tool_radius < 0, ring_samples < 3 avec un rayon
   * non nul, ou min_segment_poses == 0
   */
  OcclusionModifier(const pcl::PolygonMesh& mesh,
                    double clearance,
                    double tool_radius,
                    unsigned ring_samples,
                    std::size_t min_segment_poses);

  ToolPaths modify(ToolPaths tool_paths) const override;

  /**
   * @brief Dit si l'outil place sur cette pose atteint la surface sans rencontrer le maillage
   * @param pose Pose du chemin, z vers l'outil
   */
  bool isReachable(const Eigen::Isometry3d& pose) const;

private:
  /** @brief Vrai si le rayon parti de `start` le long de `direction` touche le maillage avant `clearance_` */
  bool rayBlocked(const Eigen::Vector3d& start, const Eigen::Vector3d& direction) const;
  /** @brief Vrai si le segment de `from` a `to` touche le maillage */
  bool segmentBlocked(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
  /** @brief Vrai si un rayon radial, du centre au bord du disque, rencontre une paroi laterale */
  bool spokesBlocked(const Eigen::Isometry3d& pose, const std::vector<Eigen::Vector3d>& origins) const;
  /** @brief Points de depart des rayons : le centre, puis le bord du disque, decolle, si le rayon est non nul */
  std::vector<Eigen::Vector3d> rayOrigins(const Eigen::Isometry3d& pose) const;
  /** @brief Coupe un segment en suites de poses accessibles, sans celles trop courtes */
  std::vector<ToolPathSegment> splitReachable(const ToolPathSegment& segment) const;

  /** @brief Maillage complet sous forme VTK, propriete du localisateur */
  vtkSmartPointer<vtkPolyData> poly_;
  /** @brief Structure d'acceleration des intersections rayon / triangle */
  vtkSmartPointer<vtkCellLocator> locator_;
  /** @brief Longueur de degagement le long de l'axe outil, en metres */
  double clearance_;
  /** @brief Rayon du disque, en metres */
  double tool_radius_;
  /** @brief Rayons repartis sur le bord du disque */
  unsigned ring_samples_;
  /** @brief Longueur minimale d'une suite de poses conservee */
  std::size_t min_segment_poses_;
};

/** @brief Nombre total de poses d'un ensemble de chemins, tous segments confondus */
std::size_t countWaypoints(const ToolPaths& tool_paths);

}  // namespace noether
