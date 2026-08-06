/**
 * @file curvature_threshold_modifier.cpp
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
#include <noether_tpp/mesh_modifiers/curvature_threshold_modifier.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>
#include <noether_tpp/serialization.h>

#include <pcl/io/vtk_lib_io.h>
#include <vtkCurvatures.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace noether
{
namespace
{
/** @brief Runs vtkCurvatures for a single curvature type and returns the resulting per-point array */
vtkSmartPointer<vtkDataArray> computeCurvature(vtkPolyData* poly, int vtk_curvature_type, const char* array_name)
{
  auto curvatures = vtkSmartPointer<vtkCurvatures>::New();
  curvatures->SetInputData(poly);
  curvatures->SetCurvatureType(vtk_curvature_type);
  curvatures->Update();

  vtkDataArray* array = curvatures->GetOutput()->GetPointData()->GetArray(array_name);
  if (array == nullptr)
    throw std::runtime_error("CurvatureThresholdMeshModifier: vtkCurvatures did not produce the '" +
                             std::string(array_name) + "' array");

  // The array is reference-counted and owned by the (soon-to-be-destroyed) filter output; returning
  // a vtkSmartPointer increments its reference count so it outlives this function.
  return vtkSmartPointer<vtkDataArray>(array);
}

}  // namespace

CurvatureThresholdMeshModifier::CurvatureThresholdMeshModifier(double max_abs_curvature, CurvatureType curvature_type)
  : max_abs_curvature_(max_abs_curvature), curvature_type_(curvature_type)
{
}

std::vector<pcl::PolygonMesh> CurvatureThresholdMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Convert to VTK; mesh2vtk preserves the vertex ordering so point i == mesh vertex i
  vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh, poly);

  const vtkIdType num_points = poly->GetNumberOfPoints();

  // Compute the per-vertex scalar metric that will be thresholded
  std::vector<double> metric(static_cast<std::size_t>(num_points), 0.0);
  switch (curvature_type_)
  {
    case CurvatureType::MEAN:
    {
      auto mean = computeCurvature(poly, VTK_CURVATURE_MEAN, "Mean_Curvature");
      for (vtkIdType i = 0; i < num_points; ++i)
        metric[static_cast<std::size_t>(i)] = std::abs(mean->GetTuple1(i));
      break;
    }
    case CurvatureType::GAUSSIAN:
    {
      auto gauss = computeCurvature(poly, VTK_CURVATURE_GAUSS, "Gauss_Curvature");
      for (vtkIdType i = 0; i < num_points; ++i)
        metric[static_cast<std::size_t>(i)] = std::abs(gauss->GetTuple1(i));
      break;
    }
    case CurvatureType::MAX_PRINCIPAL:
    case CurvatureType::MIN_PRINCIPAL:
    {
      auto k_max = computeCurvature(poly, VTK_CURVATURE_MAXIMUM, "Maximum_Curvature");
      auto k_min = computeCurvature(poly, VTK_CURVATURE_MINIMUM, "Minimum_Curvature");
      const bool take_max = (curvature_type_ == CurvatureType::MAX_PRINCIPAL);
      for (vtkIdType i = 0; i < num_points; ++i)
      {
        const double a = std::abs(k_max->GetTuple1(i));
        const double b = std::abs(k_min->GetTuple1(i));
        metric[static_cast<std::size_t>(i)] = take_max ? std::max(a, b) : std::min(a, b);
      }
      break;
    }
  }

  // Retain the vertices whose curvature magnitude is at or below the threshold. NaN values (which
  // vtkCurvatures can produce at degenerate/boundary vertices) fail the comparison and are dropped.
  std::vector<int> inlier_vertices;
  inlier_vertices.reserve(static_cast<std::size_t>(num_points));
  for (vtkIdType i = 0; i < num_points; ++i)
  {
    if (metric[static_cast<std::size_t>(i)] <= max_abs_curvature_)
      inlier_vertices.push_back(static_cast<int>(i));
  }

  if (inlier_vertices.empty())
    throw std::runtime_error("CurvatureThresholdMeshModifier: no vertices satisfy the curvature threshold (" +
                             std::to_string(max_abs_curvature_) + ")");

  // A triangle is kept only if all of its vertices are inliers (handled by extractSubMeshFromInlierVertices)
  return { extractSubMeshFromInlierVertices(mesh, inlier_vertices) };
}

std::string toString(CurvatureThresholdMeshModifier::CurvatureType type)
{
  switch (type)
  {
    case CurvatureThresholdMeshModifier::CurvatureType::MEAN:
      return "mean";
    case CurvatureThresholdMeshModifier::CurvatureType::GAUSSIAN:
      return "gaussian";
    case CurvatureThresholdMeshModifier::CurvatureType::MAX_PRINCIPAL:
      return "max_principal";
    case CurvatureThresholdMeshModifier::CurvatureType::MIN_PRINCIPAL:
      return "min_principal";
  }
  throw std::runtime_error("CurvatureThresholdMeshModifier: unknown curvature type");
}

CurvatureThresholdMeshModifier::CurvatureType curvatureTypeFromString(const std::string& str)
{
  if (str == "mean")
    return CurvatureThresholdMeshModifier::CurvatureType::MEAN;
  if (str == "gaussian")
    return CurvatureThresholdMeshModifier::CurvatureType::GAUSSIAN;
  if (str == "max_principal")
    return CurvatureThresholdMeshModifier::CurvatureType::MAX_PRINCIPAL;
  if (str == "min_principal")
    return CurvatureThresholdMeshModifier::CurvatureType::MIN_PRINCIPAL;
  throw std::runtime_error("CurvatureThresholdMeshModifier: unknown curvature type '" + str + "'");
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::CurvatureThresholdMeshModifier>::encode(const noether::CurvatureThresholdMeshModifier& val)
{
  Node node;
  node["max_abs_curvature"] = val.max_abs_curvature_;
  node["curvature_type"] = noether::toString(val.curvature_type_);
  return node;
}

bool convert<noether::CurvatureThresholdMeshModifier>::decode(const Node& node,
                                                              noether::CurvatureThresholdMeshModifier& val)
{
  val.max_abs_curvature_ = getMember<double>(node, "max_abs_curvature");

  // curvature_type is optional and defaults to mean curvature
  if (node["curvature_type"])
    val.curvature_type_ = noether::curvatureTypeFromString(node["curvature_type"].as<std::string>());
  else
    val.curvature_type_ = noether::CurvatureThresholdMeshModifier::CurvatureType::MEAN;

  return true;
}
/** @endcond */

}  // namespace YAML
