/**
 * @file face_selection.h
 * @copyright Copyright (c) 2026, Zone 404
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

#include <Eigen/Core>
#include <pcl/PolygonMesh.h>
#include <vector>

namespace noether
{
/**
 * @brief Edge-adjacency graph of a mesh: for each face, the indices of the faces sharing an edge
 * with it.
 */
using FaceAdjacency = std::vector<std::vector<int>>;

/**
 * @brief Unit normal and centroid of every face of a mesh.
 * @details Returned as one struct because both are derived from the same vertex fetch, and every
 * caller that needs one needs the other.
 */
struct FaceGeometry
{
  /** @brief Unit normal of each face, parallel to the mesh's polygon list */
  std::vector<Eigen::Vector3d> normals;
  /** @brief Centroid of each face, parallel to the mesh's polygon list */
  std::vector<Eigen::Vector3d> centroids;
};

/**
 * @brief Builds the edge-adjacency graph of a mesh.
 * @details Two faces are neighbors when they share an edge, an edge being an unordered pair of
 * vertex indices. Sharing a single vertex is not adjacency: a region grown across vertex contacts
 * would leak through the pinch point of an hourglass, and across the corner of a hole.
 * @param mesh Mesh whose faces are indexed in the order of @c mesh.polygons
 * @return A vector parallel to @c mesh.polygons; entry @c i lists the neighbors of face @c i
 */
FaceAdjacency buildFaceAdjacency(const pcl::PolygonMesh& mesh);

/**
 * @brief Computes the unit normal and centroid of every face of a mesh.
 * @details A degenerate face (fewer than three vertices, or zero area) gets a zero normal and a
 * zero centroid rather than a NaN, so a single bad triangle cannot poison a whole search.
 * @note For a face carrying more than three vertices, the normal and centroid are those of the
 * triangle formed by its first three. Noether's planners work on triangle meshes, and the rest of
 * the selection code makes the same assumption.
 * @param mesh Mesh whose faces are indexed in the order of @c mesh.polygons
 */
FaceGeometry computeFaceGeometry(const pcl::PolygonMesh& mesh);

/**
 * @brief Faces reachable from a seed face within a geodesic radius, walking the face graph.
 * @details Dijkstra over the face adjacency graph, the cost of a step being the distance between
 * the two face centroids. The distance is geodesic rather than euclidean, which is what makes the
 * result follow the surface: it cannot jump to the far side of a thin wall, it cannot cross a hole,
 * and reaching around a fold costs the real path length. That is the property a brush needs, and it
 * is also why the radius stays meaningful whatever the tessellation.
 *
 * The centroid-to-centroid path is a staircase along the mesh, so the distance it reports is an
 * upper bound of the true surface distance. The returned set is therefore slightly smaller than the
 * exact geodesic disc, never larger.
 *
 * @param adjacency Face adjacency graph, from ::buildFaceAdjacency
 * @param centroids Face centroids, from ::computeFaceGeometry; must have one entry per face
 * @param seed_face Face the search starts from
 * @param radius Radius in the mesh's length unit (metres in this project); a radius of zero returns
 *               the seed face alone
 * @return The reachable faces, seed included, in no particular order. Empty if the seed is out of
 *         range or the inputs disagree on the face count.
 */
std::vector<int> facesWithinGeodesicRadius(const FaceAdjacency& adjacency,
                                           const std::vector<Eigen::Vector3d>& centroids,
                                           int seed_face,
                                           double radius);

/**
 * @brief Whether a 2D point lies inside a closed polygon.
 * @details Even-odd (crossing number) rule: a ray is cast from the point and the crossings of the
 * polygon's edges are counted, an odd count meaning inside. The polygon is implicitly closed, its
 * last vertex connecting back to the first, and it may be concave or self-intersecting.
 * @note The result for a point lying exactly on an edge or on a vertex is unspecified. Callers feed
 * this function screen coordinates of face centroids, where an exact hit is a measure-zero event
 * whose outcome does not matter; pinning it down would cost a tolerance parameter that nothing
 * needs.
 * @param point Point to test
 * @param polygon Polygon vertices in order; fewer than three vertices always returns false
 */
bool pointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

/**
 * @brief Assigns a set of faces to a region, merging into it every region they overlap.
 * @details Beyond writing @p region_id into the listed faces, every *other* region that those faces
 * belonged to is relabeled to @p region_id in its entirety. A stroke that bridges two regions
 * therefore fuses them into one.
 *
 * That fusion is the point of the function. Without it, painting a bridge between two panels would
 * leave two regions, and two regions mean two tool paths, each paying its own approach and
 * departure. Fragmenting a surface by accident is the failure this prevents.
 *
 * @param face_region In/out: for each face, its region id, or a negative value when unselected
 * @param faces Faces to assign; indices out of range are ignored
 * @param region_id Region the faces join; must not be negative (use ::clearFacesFromRegions to
 *                  deselect)
 */
void assignFacesToRegion(std::vector<int>& face_region, const std::vector<int>& faces, int region_id);

/**
 * @brief Removes a set of faces from any region they belong to.
 * @details Only the listed faces are cleared. The rest of their regions is left alone, so erasing
 * across the middle of a region splits it into two halves that keep the same id, and will still be
 * planned as one tool path. Splitting a region into two independent ones is a deliberate act: paint
 * the two halves again as separate strokes.
 * @param face_region In/out: for each face, its region id, or a negative value when unselected
 * @param faces Faces to clear; indices out of range are ignored
 */
void clearFacesFromRegions(std::vector<int>& face_region, const std::vector<int>& faces);

}  // namespace noether
