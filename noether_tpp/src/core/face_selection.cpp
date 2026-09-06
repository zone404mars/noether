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
#include <noether_tpp/core/face_selection.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>
#include <unordered_map>
#include <utility>

namespace
{
/** @brief Min-heap of (distance, face) driving the geodesic search */
using FaceQueue = std::priority_queue<std::pair<double, int>,
                                      std::vector<std::pair<double, int>>,
                                      std::greater<std::pair<double, int>>>;

/** @brief An edge, as the pair of its vertex indices, smaller first so both faces agree on the key */
using Edge = std::pair<int, int>;

/**
 * @brief Maps every edge of a mesh to the faces that use it.
 * @details Faces with fewer than three vertices are skipped: they carry no edge, and admitting them
 * would create a self-loop from the degenerate pair (v0, v0).
 */
std::map<Edge, std::vector<int>> mapEdgesToFaces(const pcl::PolygonMesh& mesh)
{
  std::map<Edge, std::vector<int>> edge_to_faces;
  for (std::size_t f = 0; f < mesh.polygons.size(); ++f)
  {
    const pcl::Indices& v = mesh.polygons[f].vertices;
    const std::size_t n = v.size();
    if (n < 3)
    {
      continue;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
      int a = static_cast<int>(v[i]);
      int b = static_cast<int>(v[(i + 1) % n]);
      if (a > b)
      {
        std::swap(a, b);
      }
      edge_to_faces[Edge(a, b)].push_back(static_cast<int>(f));
    }
  }
  return edge_to_faces;
}

/**
 * @brief Whether the first three vertex indices of a face all address a point of the cloud.
 * @details pcl::Indices holds signed indices, so a negative value is possible and must be rejected
 * before any cast to an unsigned size.
 */
bool hasValidTriangle(const pcl::Indices& v, const std::size_t vertex_count)
{
  if (v.size() < 3)
  {
    return false;
  }
  for (std::size_t i = 0; i < 3; ++i)
  {
    if (v[i] < 0 || static_cast<std::size_t>(v[i]) >= vertex_count)
    {
      return false;
    }
  }
  return true;
}

/**
 * @brief Relaxes every neighbor of a face, queueing those whose tentative distance improves.
 */
void relaxNeighbors(const noether::FaceAdjacency& adjacency,
                    const std::vector<Eigen::Vector3d>& centroids,
                    const int face,
                    const double face_distance,
                    std::unordered_map<int, double>& distance,
                    FaceQueue& queue)
{
  const Eigen::Vector3d& from = centroids[static_cast<std::size_t>(face)];
  for (const int neighbor : adjacency[static_cast<std::size_t>(face)])
  {
    const double candidate = face_distance + (from - centroids[static_cast<std::size_t>(neighbor)]).norm();
    const std::unordered_map<int, double>::const_iterator known = distance.find(neighbor);
    if (known == distance.end() || candidate < known->second)
    {
      distance[neighbor] = candidate;
      queue.emplace(candidate, neighbor);
    }
  }
}

/**
 * @brief Writes a region id into the listed faces and reports the other regions they belonged to.
 * @return The distinct region ids the faces were taken from, sorted and deduplicated
 */
std::vector<int> assignAndCollectOverlapped(std::vector<int>& face_region,
                                            const std::vector<int>& faces,
                                            const int region_id)
{
  std::vector<int> overlapped;
  for (const int face : faces)
  {
    if (face < 0 || static_cast<std::size_t>(face) >= face_region.size())
    {
      continue;
    }
    const int previous = face_region[static_cast<std::size_t>(face)];
    if (previous >= 0 && previous != region_id)
    {
      overlapped.push_back(previous);
    }
    face_region[static_cast<std::size_t>(face)] = region_id;
  }

  std::sort(overlapped.begin(), overlapped.end());
  overlapped.erase(std::unique(overlapped.begin(), overlapped.end()), overlapped.end());
  return overlapped;
}

}  // namespace

namespace noether
{
FaceAdjacency buildFaceAdjacency(const pcl::PolygonMesh& mesh)
{
  FaceAdjacency adjacency(mesh.polygons.size());

  // Every face using an edge neighbors every other face using it. Two faces is the manifold case; a
  // non-manifold edge shared by three or more faces links them all, which is what a flood fill
  // should follow rather than pick a side.
  for (const std::pair<const Edge, std::vector<int>>& entry : mapEdgesToFaces(mesh))
  {
    const std::vector<int>& faces = entry.second;
    for (std::size_t i = 0; i < faces.size(); ++i)
    {
      for (std::size_t j = i + 1; j < faces.size(); ++j)
      {
        adjacency[static_cast<std::size_t>(faces[i])].push_back(faces[j]);
        adjacency[static_cast<std::size_t>(faces[j])].push_back(faces[i]);
      }
    }
  }
  return adjacency;
}

FaceGeometry computeFaceGeometry(const pcl::PolygonMesh& mesh)
{
  const std::size_t num_faces = mesh.polygons.size();
  FaceGeometry geometry;
  geometry.normals.assign(num_faces, Eigen::Vector3d::Zero());
  geometry.centroids.assign(num_faces, Eigen::Vector3d::Zero());

  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  for (std::size_t f = 0; f < num_faces; ++f)
  {
    // Left at zero rather than NaN: one malformed triangle must not poison a whole search.
    const pcl::Indices& v = mesh.polygons[f].vertices;
    if (!hasValidTriangle(v, vertices.size()))
    {
      continue;
    }

    const Eigen::Vector3d a = vertices[v[0]].getVector3fMap().cast<double>();
    const Eigen::Vector3d b = vertices[v[1]].getVector3fMap().cast<double>();
    const Eigen::Vector3d c = vertices[v[2]].getVector3fMap().cast<double>();

    geometry.centroids[f] = (a + b + c) / 3.0;
    const Eigen::Vector3d cross = (b - a).cross(c - a);
    const double area_norm = cross.norm();
    if (area_norm > 1e-12)
    {
      geometry.normals[f] = cross / area_norm;
    }
  }
  return geometry;
}

std::vector<int> facesWithinGeodesicRadius(const FaceAdjacency& adjacency,
                                           const std::vector<Eigen::Vector3d>& centroids,
                                           const int seed_face,
                                           const double radius)
{
  std::vector<int> region;
  const std::size_t num_faces = adjacency.size();
  if (seed_face < 0 || static_cast<std::size_t>(seed_face) >= num_faces || centroids.size() != num_faces)
  {
    return region;
  }

  // Sparse distance table: one brush dab reaches a few hundred faces of a mesh that may hold
  // hundreds of thousands. A full-size vector, cleared on every mouse move, would cost more than
  // the search itself.
  std::unordered_map<int, double> distance;
  FaceQueue queue;
  distance[seed_face] = 0.0;
  queue.emplace(0.0, seed_face);

  while (!queue.empty())
  {
    const double face_distance = queue.top().first;
    const int face = queue.top().second;
    queue.pop();

    // A stale entry: a shorter path to this face turned up after this one was queued.
    if (face_distance > distance.at(face))
    {
      continue;
    }
    // The queue is a min-heap, so nothing closer than this is left to visit.
    if (face_distance > radius)
    {
      break;
    }

    region.push_back(face);
    relaxNeighbors(adjacency, centroids, face, face_distance, distance, queue);
  }
  return region;
}

bool pointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon)
{
  const std::size_t n = polygon.size();
  if (n < 3)
  {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, previous = n - 1; i < n; previous = i++)
  {
    const Eigen::Vector2d& current = polygon[i];
    const Eigen::Vector2d& before = polygon[previous];

    // Only an edge straddling the point's ordinate can be crossed by a ray cast along +x. Pairing a
    // strict test with a non-strict one makes a shared vertex count once instead of twice.
    if ((current.y() > point.y()) != (before.y() > point.y()))
    {
      // The denominator cannot vanish: the straddle test puts the two ordinates on opposite sides.
      const double t = (point.y() - current.y()) / (before.y() - current.y());
      if (point.x() < current.x() + t * (before.x() - current.x()))
      {
        inside = !inside;
      }
    }
  }
  return inside;
}

void assignFacesToRegion(std::vector<int>& face_region, const std::vector<int>& faces, const int region_id)
{
  if (region_id < 0)
  {
    return;
  }

  const std::vector<int> overlapped = assignAndCollectOverlapped(face_region, faces, region_id);
  if (overlapped.empty())
  {
    return;
  }

  // Absorb the overlapped regions whole, so a stroke bridging two of them fuses them.
  for (int& region : face_region)
  {
    if (std::binary_search(overlapped.begin(), overlapped.end(), region))
    {
      region = region_id;
    }
  }
}

void clearFacesFromRegions(std::vector<int>& face_region, const std::vector<int>& faces)
{
  for (const int face : faces)
  {
    if (face >= 0 && static_cast<std::size_t>(face) < face_region.size())
    {
      face_region[static_cast<std::size_t>(face)] = -1;
    }
  }
}

}  // namespace noether
