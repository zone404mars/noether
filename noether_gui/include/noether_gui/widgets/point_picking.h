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
#include <vector>

class vtkAbstractCellLocator;

namespace noether
{
/**
 * @brief Rang du point visible le plus proche du curseur a l'ecran, a moins de `radius` (spec §19.2)
 * @param screen Position de chaque point a l'ecran, en pixels
 * @param visible Visibilite de chaque point, meme taille que `screen`
 * @param cursor Position du curseur, en pixels
 * @param radius Distance maximale, en pixels
 * @return Le rang du point, ou rien si aucun point visible n'est assez pres
 */
std::optional<std::size_t> nearestOnScreen(const std::vector<Eigen::Vector2d>& screen,
                                           const std::vector<bool>& visible,
                                           const Eigen::Vector2d& cursor,
                                           double radius);

/**
 * @brief Vrai quand le segment de l'oeil au point ne rencontre pas la piece a plus de `margin`
 * avant le point (spec §19.2)
 * @param locator Localisateur construit sur le maillage de la piece
 * @param eye Position de la camera ; en projection parallele, un point eloigne sur l'axe de vue
 * @param point Point a tester
 * @param margin Longueur, en metres, pres du point ou un obstacle ne compte pas : le point est
 * pose sur la piece, son propre triangle ne doit pas le cacher
 */
bool visibleFrom(vtkAbstractCellLocator& locator, const Eigen::Vector3d& eye, const Eigen::Vector3d& point,
                 double margin);

}  // namespace noether
