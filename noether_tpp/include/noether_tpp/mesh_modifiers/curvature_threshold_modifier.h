/**
 * @file curvature_threshold_modifier.h
 * @copyright Copyright (c) 2026, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

#include <string>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief MeshModifier that retains the portion of the mesh whose surface curvature magnitude is at
 * or below a threshold, i.e. the regions "flat enough" to be reached by a finishing tool of a given
 * radius.
 * @details The per-vertex curvature is computed with `vtkCurvatures` (which preserves the input
 * vertex ordering, so vertex @c i of the input mesh maps to point @c i of the curvature array). A
 * triangle is retained only if all three of its vertices satisfy the threshold; the retained
 * triangles are extracted into a single submesh via ::extractSubMeshFromInlierVertices, which
 * preserves every vertex data field of the parent cloud (xyz, normals, etc.).
 *
 * @note Curvature is expressed in the inverse of the mesh's length unit. For a mesh in meters, a
 * spherical cap of radius @c r has mean and principal curvatures equal to @c 1/r, so a tool of
 * diameter @c D (radius @c D/2) corresponds to a threshold of @c 2/D [1/m]. A disc sander of
 * @c Ø125&nbsp;mm therefore maps to a threshold of @c 2/0.125 = 16&nbsp;m⁻¹.
 *
 * @note ::CurvatureType::MEAN under-reports tightness in a single direction (a cylinder of radius
 * @c r has mean curvature @c 1/(2r) but a principal curvature of @c 1/r); prefer
 * ::CurvatureType::MAX_PRINCIPAL when the criterion is whether a rigid disc tool physically fits.
 */
class CurvatureThresholdMeshModifier : public MeshModifier
{
public:
  /** @brief Curvature measure to threshold against */
  enum class CurvatureType
  {
    /** @brief Mean curvature, (k_max + k_min) / 2 */
    MEAN,
    /** @brief Gaussian curvature, k_max * k_min */
    GAUSSIAN,
    /** @brief Tightest principal curvature in any direction, max(|k_max|, |k_min|) */
    MAX_PRINCIPAL,
    /** @brief Shallowest principal curvature in any direction, min(|k_max|, |k_min|) */
    MIN_PRINCIPAL,
  };

  /**
   * @param max_abs_curvature Maximum allowed absolute curvature [1/length] for a vertex to be
   * retained
   * @param curvature_type Curvature measure to threshold against
   */
  CurvatureThresholdMeshModifier(double max_abs_curvature, CurvatureType curvature_type = CurvatureType::MEAN);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  double max_abs_curvature_;
  CurvatureType curvature_type_;

  CurvatureThresholdMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(CurvatureThresholdMeshModifier)
};

/** @brief Serializes a ::CurvatureThresholdMeshModifier::CurvatureType to its YAML/GUI string */
std::string toString(CurvatureThresholdMeshModifier::CurvatureType type);
/** @brief Parses a ::CurvatureThresholdMeshModifier::CurvatureType from its YAML/GUI string */
CurvatureThresholdMeshModifier::CurvatureType curvatureTypeFromString(const std::string& str);

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::CurvatureThresholdMeshModifier)
