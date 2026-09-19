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
 * @brief Ce que le disque recouvre de la selection en suivant les passes livrees.
 * @details Une face est couverte quand son centroide est a moins du rayon d'outil d'un troncon
 * reliant deux poses consecutives d'une passe. C'est la mesure qui dit si un chemin, apres
 * retouche ou enchainement, ponce encore toute la surface demandee, et sinon ou il manque.
 */
struct CoverageResult
{
  /** @brief Faces de la selection qu'aucun troncon n'approche a moins du rayon */
  std::vector<int> uncovered_faces;
  /** @brief Nombre de faces de la selection examinees */
  std::size_t selected_count{ 0 };
  /** @brief Part des faces couvertes, de 0 a 1 ; 1 quand la selection est vide */
  double ratio{ 1.0 };
};

/**
 * @brief Mesure la couverture de la selection par les passes livrees.
 * @param passes Passes livrees, dans l'ordre d'execution
 * @param centroids Centroide de chaque face du maillage, indexe par face
 * @param selected_faces Faces de la selection a couvrir
 * @param tool_radius Rayon du disque, en metres, strictement positif
 * @param skirt_poses Poses ecartees a chaque bout de chaque passe : l'approche et le retrait ne
 *        sont pas au contact et ne poncent rien
 * @throws std::invalid_argument si tool_radius <= 0, ou si une face selectionnee n'a pas de centroide
 */
CoverageResult computeCoverage(const ToolPath& passes,
                               const std::vector<Eigen::Vector3d>& centroids,
                               const std::vector<int>& selected_faces,
                               double tool_radius,
                               std::size_t skirt_poses);

}  // namespace noether
