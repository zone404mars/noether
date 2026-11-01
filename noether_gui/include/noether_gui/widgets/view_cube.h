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

#include <Eigen/Core>
#include <optional>

namespace noether
{
/** @brief Cote ou se place la camera pour la vue maison, l'Isometrique de V1 (spec §22) */
inline const Eigen::Vector3d kHomeViewDirection(1.0, -1.0, 1.0);

/** @brief Fleche triangulaire du cube de navigation, nommee par son cote de l'ecran (spec §22) */
enum class CubeArrow
{
  Up,
  Down,
  Left,
  Right
};

/**
 * @brief Point ou un rayon entre dans le cube de cote 1 centre a l'origine
 * @param origin Depart du rayon
 * @param direction Direction du rayon, non nulle
 * @return Le premier point de la surface du cube touche, ou rien
 */
std::optional<Eigen::Vector3d> cubeHit(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);

/**
 * @brief Direction de camera designee par un point de la surface du cube (spec §22)
 * @details Chaque face est decoupee en 3 x 3 cases : la case centrale donne l'axe de la face,
 * une case de bord la bissectrice de deux faces, une case d'angle la diagonale du coin.
 * @param hit Point de la surface du cube de cote 1 centre a l'origine
 * @return La direction, unitaire, du cote ou se place la camera
 */
Eigen::Vector3d cubeZoneDirection(const Eigen::Vector3d& hit);

/**
 * @brief Haut de l'ecran pour une vue : +Z, sauf quand l'axe de vue est a moins de 1 deg de +-Z,
 * alors +Y (spec §22)
 * @param direction Cote ou se place la camera, non nul
 */
Eigen::Vector3d defaultViewUp(const Eigen::Vector3d& direction);

/**
 * @brief Axe de la face du cube a moins de 1 deg de l'axe de vue, ou rien
 * @param direction Cote ou se place la camera, non nul
 */
std::optional<Eigen::Vector3d> alignedFace(const Eigen::Vector3d& direction);

/**
 * @brief Direction de la face voisine situee du cote de l'ecran qu'indique la fleche
 * @param direction Cote ou se place la camera, aligne sur une face
 * @param up Haut de l'ecran
 */
Eigen::Vector3d neighborFace(const Eigen::Vector3d& direction, const Eigen::Vector3d& up, CubeArrow arrow);

/**
 * @brief Haut de l'ecran apres une rotation de 90 deg autour de l'axe de vue
 * @param clockwise Vrai pour le sens horaire a l'ecran
 */
Eigen::Vector3d rolledUp(const Eigen::Vector3d& direction, const Eigen::Vector3d& up, bool clockwise);

}  // namespace noether
