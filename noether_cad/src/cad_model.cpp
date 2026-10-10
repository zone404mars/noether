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

#include <noether_cad/cad_model.h>

namespace noether
{
namespace cad
{
std::string faceTypeName(const FaceType type)
{
  switch (type)
  {
    case FaceType::PLANE:
      return "plane";
    case FaceType::CYLINDER:
      return "cylinder";
    case FaceType::CONE:
      return "cone";
    case FaceType::SPHERE:
      return "sphere";
    case FaceType::TORUS:
      return "torus";
    case FaceType::BSPLINE:
      return "bspline";
    default:
      return "other";
  }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> CadModel::boundingBox() const
{
  if (mesh.vertices.empty())
  {
    return { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
  }
  Eigen::Vector3d lower = mesh.vertices.front();
  Eigen::Vector3d upper = mesh.vertices.front();
  for (const Eigen::Vector3d& vertex : mesh.vertices)
  {
    lower = lower.cwiseMin(vertex);
    upper = upper.cwiseMax(vertex);
  }
  return { lower, upper };
}

}  // namespace cad
}  // namespace noether
