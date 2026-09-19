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
#include <vector>

namespace noether
{
/**
 * @brief Contour polygonal trace a l'ecran, un sommet par clic.
 * @details Remplace le lasso a main levee : l'operateur pose des sommets, le contour se dessine
 * ferme au fur et a mesure, et un clic sur le premier sommet le clot. Sans Qt ni VTK, pour que la
 * regle de fermeture se teste sans fenetre.
 */
class OutlineBuilder
{
public:
  /**
   * @param close_tolerance_px Distance a l'ecran, en pixels, en dessous de laquelle un clic vise le
   * premier sommet et ferme le contour au lieu d'ajouter un sommet
   */
  explicit OutlineBuilder(double close_tolerance_px);

  /**
   * @brief Prend en compte un clic
   * @return Vrai quand le clic FERME le contour : il visait le premier sommet et le contour en a au
   * moins trois. Le contour n'est alors pas modifie, l'appelant le lit puis l'efface. Faux sinon :
   * le sommet a ete ajoute, ou ignore parce qu'il repetait le dernier.
   */
  bool addVertex(const Eigen::Vector2d& point);

  /** @brief Retire le dernier sommet pose, s'il y en a un */
  void removeLastVertex();

  /** @brief Efface le contour */
  void clear();

  /** @brief Vrai quand le contour peut se fermer : trois sommets au moins */
  bool canClose() const;

  const std::vector<Eigen::Vector2d>& vertices() const;

private:
  /** @brief Rayon de fermeture autour du premier sommet, en pixels */
  double close_tolerance_px_;
  /** @brief Sommets poses, en coordonnees d'affichage */
  std::vector<Eigen::Vector2d> vertices_;
};

}  // namespace noether
