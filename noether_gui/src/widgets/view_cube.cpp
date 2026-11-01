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
#include <noether_gui/widgets/view_cube.h>

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace
{
/** @brief Demi-cote du cube de navigation */
constexpr double kHalfSide = 0.5;
/** @brief Tolerance d'angle, 1 deg (spec §22, valeur validee le 31/10/2026) */
const double kAlignedCosine = std::cos(M_PI / 180.0);

/** @brief Vecteur arrondi a l'axe du repere le plus proche, signe compris */
Eigen::Vector3d nearestAxis(const Eigen::Vector3d& vector)
{
  Eigen::Vector3d::Index axis = 0;
  vector.cwiseAbs().maxCoeff(&axis);
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  result[axis] = vector[axis] >= 0.0 ? 1.0 : -1.0;
  return result;
}
}  // namespace

namespace noether
{
std::optional<Eigen::Vector3d> cubeHit(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction)
{
  // Methode des tranches : intervalle d'entree et de sortie du rayon sur chaque axe
  double enter = -std::numeric_limits<double>::infinity();
  double leave = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis)
  {
    if (std::abs(direction[axis]) < 1e-12)
    {
      if (std::abs(origin[axis]) > kHalfSide)
      {
        return std::nullopt;
      }
      continue;
    }
    const double a = (-kHalfSide - origin[axis]) / direction[axis];
    const double b = (kHalfSide - origin[axis]) / direction[axis];
    enter = std::max(enter, std::min(a, b));
    leave = std::min(leave, std::max(a, b));
  }
  if (enter > leave || leave < 0.0)
  {
    return std::nullopt;
  }
  return origin + direction * std::max(enter, 0.0);
}

Eigen::Vector3d cubeZoneDirection(const Eigen::Vector3d& hit)
{
  const Eigen::Vector3d face = nearestAxis(hit);
  Eigen::Vector3d direction = face;
  for (int axis = 0; axis < 3; ++axis)
  {
    // Hors de l'axe de la face : tiers bas, milieu ou haut de la face
    if (face[axis] == 0.0)
    {
      direction[axis] = hit[axis] > kHalfSide / 3.0 ? 1.0 : (hit[axis] < -kHalfSide / 3.0 ? -1.0 : 0.0);
    }
  }
  return direction.normalized();
}

Eigen::Vector3d defaultViewUp(const Eigen::Vector3d& direction)
{
  return std::abs(direction.normalized().z()) >= kAlignedCosine ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitZ();
}

std::optional<Eigen::Vector3d> alignedFace(const Eigen::Vector3d& direction)
{
  const Eigen::Vector3d axis = nearestAxis(direction);
  if (axis.dot(direction.normalized()) >= kAlignedCosine)
  {
    return axis;
  }
  return std::nullopt;
}

Eigen::Vector3d neighborFace(const Eigen::Vector3d& direction, const Eigen::Vector3d& up, const CubeArrow arrow)
{
  // La camera regarde vers -direction ; la droite de l'ecran est le produit de la visee par le haut
  const Eigen::Vector3d right = (-direction).cross(up);
  switch (arrow)
  {
    case CubeArrow::Up:
      return nearestAxis(up);
    case CubeArrow::Down:
      return nearestAxis(-up);
    case CubeArrow::Left:
      return nearestAxis(-right);
    case CubeArrow::Right:
      break;
  }
  return nearestAxis(right);
}

Eigen::Vector3d rolledUp(const Eigen::Vector3d& direction, const Eigen::Vector3d& up, const bool clockwise)
{
  // Image tournee dans le sens horaire : ce qui etait a gauche monte en haut
  const Eigen::Vector3d right = (-direction).cross(up).normalized();
  return clockwise ? Eigen::Vector3d(-right) : right;
}

}  // namespace noether
