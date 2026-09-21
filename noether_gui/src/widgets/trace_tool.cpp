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
#include <noether_gui/widgets/trace_tool.h>

#include <Eigen/Geometry>
#include <stdexcept>

namespace
{
/** @brief Norme en dessous de laquelle deux points ou deux directions sont confondus */
constexpr double kDegenerate = 1e-9;

/// @brief Repere d'une pose : z le long de la normale, x le sens de marche rendu orthogonal a z.
/// @param travel Sens de marche, non normalise
/// @param normal Normale de la surface, non normalisee
/// @return Matrice de rotation ; si le sens de marche est colineaire a la normale, x est pris
/// dans un axe quelconque du plan, ce cas n'arrivant que sur un trace degenere
Eigen::Matrix3d traceFrame(const Eigen::Vector3d& travel, const Eigen::Vector3d& normal)
{
  const Eigen::Vector3d z = normal.normalized();
  Eigen::Vector3d x = travel - travel.dot(z) * z;
  if (x.norm() < kDegenerate)
  {
    x = z.unitOrthogonal();
  }
  x.normalize();
  Eigen::Matrix3d frame;
  frame.col(0) = x;
  frame.col(1) = z.cross(x);
  frame.col(2) = z;
  return frame;
}

/// @brief Pose les points d'un segment droit, au pas demande, en reportant l'arc deja parcouru.
/// @param from Depart du segment
/// @param to Fin du segment
/// @param step Espacement voulu
/// @param since_last Entree : arc parcouru depuis le dernier point pose ; sortie : la meme
/// mesure a la fin du segment
/// @param points Liste a laquelle les points sont ajoutes
void sampleChord(const Eigen::Vector3d& from,
                 const Eigen::Vector3d& to,
                 const double step,
                 double& since_last,
                 std::vector<Eigen::Vector3d>& points)
{
  const Eigen::Vector3d chord = to - from;
  const double length = chord.norm();
  // Deux sommets confondus : rien a poser, et surtout pas de division par zero
  if (length < kDegenerate)
  {
    return;
  }
  for (double along = step - since_last; along <= length + kDegenerate; along += step)
  {
    points.push_back(from + chord * (along / length));
    since_last = -along;
  }
  since_last += length;
}
}  // namespace

namespace noether
{
std::vector<Eigen::Vector3d> sampleTrace(const std::vector<Eigen::Vector3d>& vertices, const double step)
{
  if (!(step > 0.0))
  {
    throw std::invalid_argument("Le pas du trace doit etre strictement positif");
  }
  std::vector<Eigen::Vector3d> points;
  if (vertices.empty())
  {
    return points;
  }
  points.push_back(vertices.front());
  // Longueur d'arc parcourue depuis le dernier point pose, reportee d'un segment au suivant
  double since_last = 0.0;
  for (std::size_t k = 1; k < vertices.size(); ++k)
  {
    sampleChord(vertices[k - 1], vertices[k], step, since_last, points);
  }
  if ((points.back() - vertices.back()).norm() > kDegenerate)
  {
    points.push_back(vertices.back());
  }
  return points;
}

ToolPathSegment tracePoses(const std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& normals)
{
  if (points.size() != normals.size())
  {
    throw std::invalid_argument("Il faut une normale par point du trace");
  }
  ToolPathSegment poses;
  if (points.size() < 2)
  {
    return poses;
  }
  for (std::size_t k = 0; k < points.size(); ++k)
  {
    // Le dernier point n'a pas de suivant : il garde le sens du dernier segment
    const Eigen::Vector3d travel = (k + 1 < points.size()) ? points[k + 1] - points[k] : points[k] - points[k - 1];
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = traceFrame(travel, normals[k]);
    pose.translation() = points[k];
    poses.push_back(pose);
  }
  return poses;
}

ToolPathSegment withApproachAndRetract(const ToolPathSegment& contact, const double height)
{
  if (contact.empty())
  {
    throw std::invalid_argument("Il faut au moins une pose de contact pour poser l'approche et le retrait");
  }
  if (!(height > 0.0))
  {
    throw std::invalid_argument("La hauteur d'approche doit etre strictement positive");
  }
  Eigen::Isometry3d approach = contact.front();
  approach.translation() += approach.linear().col(2) * height;
  Eigen::Isometry3d retract = contact.back();
  retract.translation() += retract.linear().col(2) * height;

  ToolPathSegment pass;
  pass.push_back(approach);
  pass.insert(pass.end(), contact.begin(), contact.end());
  pass.push_back(retract);
  return pass;
}

}  // namespace noether
