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

#include <noether_tpp/core/types.h>
#include <Eigen/Core>
#include <vector>

namespace noether
{
/**
 * @brief Points regulierement espaces le long d'une ligne brisee.
 * @details Le premier point est le premier sommet, puis un point tous les `step` metres de
 * longueur d'arc, en ligne droite d'un sommet au suivant. Le dernier sommet est toujours le
 * dernier point, meme s'il tombe a moins d'un pas du precedent : c'est l'arrivee que l'operateur
 * a montree. Sans Qt ni VTK, pour se tester sans fenetre.
 * @param vertices Sommets du trace, dans l'ordre ; un seul sommet donne un seul point
 * @param step Espacement voulu, en metres, strictement positif
 * @throws std::invalid_argument si step <= 0
 */
std::vector<Eigen::Vector3d> sampleTrace(const std::vector<Eigen::Vector3d>& vertices, double step);

/**
 * @brief Poses d'un trace : la position, l'axe z le long de la normale donnee, l'axe x dans le
 * sens de marche.
 * @details L'axe x est la direction vers le point suivant (vers le precedent pour le dernier
 * point), rendue orthogonale a la normale. Un seul point ne donne aucune direction de marche et
 * donc aucune pose.
 * @param points Points du trace, dans l'ordre de parcours
 * @param normals Normale de la surface en chaque point, meme taille que `points`
 * @throws std::invalid_argument si les deux listes n'ont pas la meme taille
 */
ToolPathSegment tracePoses(const std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& normals);

}  // namespace noether
