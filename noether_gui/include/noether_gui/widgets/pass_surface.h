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

#include <Eigen/Geometry>
#include <array>
#include <optional>
#include <pcl/PolygonMesh.h>
#include <vector>
#include <vtkSmartPointer.h>

class vtkCellLocator;
class vtkPolyData;

namespace noether
{
/** @brief Point pose sur une surface, avec la normale plane du triangle qui le porte */
struct SurfacePoint
{
  /** @brief Position sur le triangle retenu */
  Eigen::Vector3d position;
  /** @brief Normale plane du triangle retenu, unitaire */
  Eigen::Vector3d normal;
};

/**
 * @brief Surface d'une passe : les triangles de la surface validee qui a donne la passe, gardes
 * tels qu'ils etaient au moment de Generer (spec §21.1).
 * @details Densifier pose ses points ici, jamais sur le maillage entier : sur une tole mince
 * courbee, la face la plus proche du maillage entier peut etre la peau interieure.
 * Sans Qt, pour se tester sans fenetre.
 */
class PassSurface
{
public:
  /**
   * @brief Copie les faces de la surface ; un polygone de plus de trois sommets est decoupe en
   * triangles en eventail.
   * @param mesh Maillage de la piece
   * @param faces Indices des faces de la surface dans `mesh`
   * @throws std::invalid_argument si `faces` est vide, nomme une face ou un sommet absent de `mesh`, ou ne
   *         donne aucun triangle
   */
  PassSurface(const pcl::PolygonMesh& mesh, const std::vector<int>& faces);

  /**
   * @brief Point de la surface le plus proche de `start`, pris sur les triangles dont la normale
   * plane est du meme cote que `expected_normal` (produit scalaire strictement positif).
   * @param start Point de depart, en metres
   * @param expected_normal Normale attendue en ce point, unitaire
   * @return Le point retenu, ou rien si aucun triangle n'est du bon cote ou si une entree n'est
   *         pas finie
   */
  std::optional<SurfacePoint> place(const Eigen::Vector3d& start, const Eigen::Vector3d& expected_normal) const;

  /**
   * @brief Point de la surface le plus proche de `start`, sans test de cote
   * @details Pour un sommet du trace, qui n'a pas d'orientation (spec §19.3).
   * @return Le point le plus proche, ou rien si `start` n'est pas fini
   */
  std::optional<SurfacePoint> closest(const Eigen::Vector3d& start) const;

  /** @brief Nombre de triangles de la surface */
  std::size_t size() const { return triangles_.size(); }

private:
  /** @brief Decoupe un polygone en triangles issus de son premier sommet et les garde */
  void addPolygon(const std::vector<Eigen::Vector3d>& corners);
  /** @brief Garde un triangle, sa normale plane et etend la boite englobante */
  void addTriangle(const std::array<Eigen::Vector3d, 3>& triangle);
  /** @brief Construit ::poly_ et ::locator_ sur les triangles gardes */
  void buildLocator();
  /**
   * @brief Le triangle du bon cote le plus proche de toute la surface, par une boite doublee a
   * chaque tour depuis `nearest`
   * @param nearest Distance au triangle le plus proche, tous cotes confondus : rien n'est plus pres
   */
  std::optional<SurfacePoint> closestFacing(const Eigen::Vector3d& start, const Eigen::Vector3d& expected_normal,
                                            double nearest) const;

  /** @brief Le triangle du bon cote le plus proche, cherche dans une boite de demi-cote `radius` */
  std::optional<SurfacePoint> closestFacingWithin(const Eigen::Vector3d& start,
                                                  const Eigen::Vector3d& expected_normal,
                                                  double radius,
                                                  double& squared_distance) const;

  /** @brief Sommets de chaque triangle, dans l'ordre des cellules de ::poly_ */
  std::vector<std::array<Eigen::Vector3d, 3>> triangles_;
  /** @brief Normale plane unitaire de chaque triangle */
  std::vector<Eigen::Vector3d> normals_;
  /** @brief Les memes triangles pour VTK : la cellule k est le triangle k */
  vtkSmartPointer<vtkPolyData> poly_;
  /** @brief Recherche du triangle le plus proche et des triangles d'une boite */
  vtkSmartPointer<vtkCellLocator> locator_;
  /** @brief Boite englobante de la surface : une recherche qui la contient a tout vu */
  Eigen::AlignedBox3d bounds_;
};

/**
 * @brief Normale attendue entre deux normales, par interpolation spherique (spec §21.1).
 * @param from Normale au depart, unitaire
 * @param to Normale a l'arrivee, unitaire
 * @param fraction Position entre les deux, de 0 (depart) a 1 (arrivee)
 * @return La normale interpolee, unitaire, ou rien si les deux normales sont opposees
 */
std::optional<Eigen::Vector3d> interpolateNormal(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                                                 double fraction);

/**
 * @brief Poses ajoutees dans l'ecart entre deux poses de contact voisines (spec §21.1).
 * @details Les points sont repartis regulierement sur la droite de l'ecart, puis poses sur la
 * surface. z prend la normale du triangle retenu ; x suit la direction de l'ecart, rendue
 * orthogonale a z.
 * @param surface Surface de la passe
 * @param from Pose qui ouvre l'ecart
 * @param to Pose qui ferme l'ecart
 * @param count Nombre de poses a ajouter ; zero ou moins ne donne aucune pose
 * @return Les poses ajoutees, dans le sens de la passe, ou rien si l'ecart est saute : axes z
 * opposes, ou un point sans triangle du bon cote
 */
std::optional<ToolPathSegment> densifyGap(const PassSurface& surface,
                                          const Eigen::Isometry3d& from,
                                          const Eigen::Isometry3d& to,
                                          int count);

}  // namespace noether
