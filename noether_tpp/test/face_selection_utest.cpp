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
#include <filesystem>
#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <set>
#include <stdexcept>

using namespace noether;

namespace
{
/** @brief Spacing between the vertices of the synthetic grids, in metres */
constexpr double kStep = 0.01;

/** @brief Builds one triangle from three vertex indices */
pcl::Vertices triangle(const int a, const int b, const int c)
{
  pcl::Vertices face;
  face.vertices = { a, b, c };
  return face;
}

/**
 * @brief Builds a flat triangulated grid in a plane parallel to XY.
 * @details Vertices are laid out row-major, so vertex (r, c) has index r * cols + c. Each cell is
 * split into two triangles along its (v00, v11) diagonal, which is the diagonal the tests rely on
 * to know which faces share an edge.
 * @param rows Number of vertex rows
 * @param cols Number of vertex columns
 * @param z Height of the plane
 */
pcl::PolygonMesh makeGrid(const int rows, const int cols, const double z = 0.0)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int r = 0; r < rows; ++r)
  {
    for (int c = 0; c < cols; ++c)
    {
      cloud.push_back(pcl::PointXYZ(static_cast<float>(c * kStep), static_cast<float>(r * kStep),
                                    static_cast<float>(z)));
    }
  }

  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  for (int r = 0; r + 1 < rows; ++r)
  {
    for (int c = 0; c + 1 < cols; ++c)
    {
      const int v00 = r * cols + c;
      const int v01 = v00 + 1;
      const int v10 = v00 + cols;
      const int v11 = v10 + 1;
      mesh.polygons.push_back(triangle(v00, v01, v11));
      mesh.polygons.push_back(triangle(v00, v11, v10));
    }
  }
  return mesh;
}

/**
 * @brief Concatenates two meshes into one, keeping them topologically disconnected.
 * @details The second mesh's vertex indices are shifted past the first mesh's cloud, so no vertex
 * and therefore no edge is shared. The result is a single mesh holding two components.
 */
pcl::PolygonMesh mergeMeshes(const pcl::PolygonMesh& first, const pcl::PolygonMesh& second)
{
  pcl::PointCloud<pcl::PointXYZ> first_cloud;
  pcl::PointCloud<pcl::PointXYZ> second_cloud;
  pcl::fromPCLPointCloud2(first.cloud, first_cloud);
  pcl::fromPCLPointCloud2(second.cloud, second_cloud);

  const int offset = static_cast<int>(first_cloud.size());
  first_cloud += second_cloud;

  pcl::PolygonMesh merged;
  pcl::toPCLPointCloud2(first_cloud, merged.cloud);
  merged.polygons = first.polygons;
  for (const pcl::Vertices& face : second.polygons)
  {
    pcl::Vertices shifted;
    for (const pcl::index_t v : face.vertices)
    {
      shifted.vertices.push_back(v + offset);
    }
    merged.polygons.push_back(shifted);
  }
  return merged;
}

/**
 * @brief Builds a strip folded back on itself, a hairpin seen from the side.
 * @details The topology is that of a flat two-row strip; only the vertex positions are folded. The
 * outward arm runs along +x at z = 0, the return arm comes back along -x at z = @p gap, and the two
 * meet at the far end. The first and last faces therefore sit @p gap apart in space while the only
 * path between them runs the whole length of the strip, twice.
 *
 * This is the shape that separates a geodesic criterion from a euclidean one: a brush placed on one
 * arm must not bleed onto the other, however close the two arms are.
 * @param columns Number of vertex columns per arm
 * @param length Length of each arm
 * @param gap Distance between the two arms
 */
pcl::PolygonMesh makeHairpinStrip(const int columns, const double length, const double gap)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const auto push_column = [&cloud](const double x, const double z) {
    cloud.push_back(pcl::PointXYZ(static_cast<float>(x), 0.0F, static_cast<float>(z)));
    cloud.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(kStep), static_cast<float>(z)));
  };

  for (int c = 0; c < columns; ++c)
  {
    push_column(length * c / (columns - 1), 0.0);
  }
  for (int c = columns - 1; c >= 0; --c)
  {
    push_column(length * c / (columns - 1), gap);
  }

  // Two vertices per column, so column c owns vertices 2c and 2c + 1.
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  const int total_columns = 2 * columns;
  for (int c = 0; c + 1 < total_columns; ++c)
  {
    const int v0 = 2 * c;
    const int v1 = v0 + 1;
    const int v2 = v0 + 2;
    const int v3 = v0 + 3;
    mesh.polygons.push_back(triangle(v0, v1, v3));
    mesh.polygons.push_back(triangle(v0, v3, v2));
  }
  return mesh;
}

/** @brief Loads a mesh from the test mesh directory, throwing when it cannot be read */
pcl::PolygonMesh loadMesh(const std::string& mesh_file)
{
  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFile(mesh_file, mesh) > 0)
  {
    return mesh;
  }
  throw std::runtime_error("Failed to load test mesh from '" + mesh_file + "'");
}

/** @brief The faces of a search result, as a set, for order-independent comparison */
std::set<int> asSet(const std::vector<int>& faces) { return std::set<int>(faces.begin(), faces.end()); }

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// buildFaceAdjacency
// ─────────────────────────────────────────────────────────────────────────────

/** @brief The graph must be symmetric, loop-free, and bounded by three neighbors per triangle */
TEST(FaceAdjacency, IsSymmetricAndLoopFree)
{
  const pcl::PolygonMesh mesh = makeGrid(5, 5);
  const FaceAdjacency adjacency = buildFaceAdjacency(mesh);

  ASSERT_EQ(adjacency.size(), mesh.polygons.size());
  for (std::size_t f = 0; f < adjacency.size(); ++f)
  {
    const std::vector<int>& neighbors = adjacency[f];
    EXPECT_LE(neighbors.size(), 3U) << "face " << f << " has more neighbors than it has edges";
    for (const int neighbor : neighbors)
    {
      EXPECT_NE(neighbor, static_cast<int>(f)) << "face " << f << " is its own neighbor";
      const std::vector<int>& back = adjacency[static_cast<std::size_t>(neighbor)];
      EXPECT_NE(std::find(back.begin(), back.end(), static_cast<int>(f)), back.end())
          << "adjacency is not symmetric between " << f << " and " << neighbor;
    }
  }
}

/** @brief The two triangles of a grid cell share its diagonal, so they must be neighbors */
TEST(FaceAdjacency, TrianglesOfOneCellAreNeighbors)
{
  const FaceAdjacency adjacency = buildFaceAdjacency(makeGrid(3, 3));

  // makeGrid emits the two triangles of each cell back to back.
  const std::vector<int>& neighbors_of_first = adjacency[0];
  EXPECT_NE(std::find(neighbors_of_first.begin(), neighbors_of_first.end(), 1), neighbors_of_first.end());
}

/**
 * @brief Sharing a single vertex is not adjacency.
 * @details In a 3x3 grid, the cell at (0, 0) and the cell at (1, 1) meet at vertex 4 and share no
 * edge. A graph built on vertex contact instead of edge contact would link them, and a flood fill
 * would then leak diagonally through every pinch point of the mesh.
 */
TEST(FaceAdjacency, VertexContactIsNotAdjacency)
{
  const FaceAdjacency adjacency = buildFaceAdjacency(makeGrid(3, 3));

  // Cell (0, 0) emits faces 0 and 1; cell (1, 1) emits faces 6 and 7.
  for (const int diagonal_face : { 6, 7 })
  {
    for (const int corner_face : { 0, 1 })
    {
      const std::vector<int>& neighbors = adjacency[static_cast<std::size_t>(corner_face)];
      EXPECT_EQ(std::find(neighbors.begin(), neighbors.end(), diagonal_face), neighbors.end())
          << "faces " << corner_face << " and " << diagonal_face << " share only a vertex";
    }
  }
}

/** @brief No adjacency may bridge two topologically separate components */
TEST(FaceAdjacency, DoesNotBridgeDisconnectedComponents)
{
  const pcl::PolygonMesh first = makeGrid(5, 5, 0.0);
  const int faces_in_first = static_cast<int>(first.polygons.size());
  const FaceAdjacency adjacency = buildFaceAdjacency(mergeMeshes(first, makeGrid(5, 5, 0.001)));

  for (int f = 0; f < faces_in_first; ++f)
  {
    for (const int neighbor : adjacency[static_cast<std::size_t>(f)])
    {
      EXPECT_LT(neighbor, faces_in_first) << "face " << f << " reaches the second component";
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// computeFaceGeometry
// ─────────────────────────────────────────────────────────────────────────────

/** @brief On a plane parallel to XY, every normal is unit and along Z, every centroid in the plane */
TEST(FaceGeometry, FlatGridGivesUnitNormalsAlongZ)
{
  const double height = 0.25;
  const pcl::PolygonMesh mesh = makeGrid(4, 4, height);
  const FaceGeometry geometry = computeFaceGeometry(mesh);

  ASSERT_EQ(geometry.normals.size(), mesh.polygons.size());
  ASSERT_EQ(geometry.centroids.size(), mesh.polygons.size());
  for (std::size_t f = 0; f < geometry.normals.size(); ++f)
  {
    EXPECT_NEAR(geometry.normals[f].norm(), 1.0, 1e-9);
    EXPECT_NEAR(std::abs(geometry.normals[f].z()), 1.0, 1e-9);
    EXPECT_NEAR(geometry.centroids[f].z(), height, 1e-6);
  }
}

/** @brief A face with too few vertices yields zeros, not NaN, and does not abort the pass */
TEST(FaceGeometry, DegenerateFaceYieldsZeros)
{
  pcl::PolygonMesh mesh = makeGrid(3, 3);
  const std::size_t degenerate = mesh.polygons.size();
  mesh.polygons.push_back(triangle(0, 1, 1));  // zero area
  pcl::Vertices two_vertices;
  two_vertices.vertices = { 0, 1 };
  mesh.polygons.push_back(two_vertices);

  const FaceGeometry geometry = computeFaceGeometry(mesh);

  ASSERT_EQ(geometry.normals.size(), mesh.polygons.size());
  EXPECT_TRUE(geometry.normals[degenerate].isZero());
  EXPECT_TRUE(geometry.normals[degenerate + 1].isZero());
  EXPECT_TRUE(geometry.centroids[degenerate + 1].isZero());
  // The valid faces around it are still described.
  EXPECT_NEAR(geometry.normals[0].norm(), 1.0, 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// facesWithinGeodesicRadius
// ─────────────────────────────────────────────────────────────────────────────

/** @brief Fixture holding a single flat grid and its derived graph and geometry */
class GeodesicRadius : public testing::Test
{
protected:
  void SetUp() override
  {
    mesh_ = makeGrid(21, 21);
    adjacency_ = buildFaceAdjacency(mesh_);
    geometry_ = computeFaceGeometry(mesh_);
    // A face near the middle of the grid, far from every border.
    seed_ = static_cast<int>(mesh_.polygons.size()) / 2;
  }

  pcl::PolygonMesh mesh_;
  FaceAdjacency adjacency_;
  FaceGeometry geometry_;
  int seed_ = 0;
};

/** @brief A radius of zero reaches the seed and nothing else */
TEST_F(GeodesicRadius, ZeroRadiusReturnsTheSeedAlone)
{
  const std::vector<int> region = facesWithinGeodesicRadius(adjacency_, geometry_.centroids, seed_, 0.0);
  EXPECT_EQ(region, std::vector<int>{ seed_ });
}

/** @brief Growing the radius can only add faces, never drop one */
TEST_F(GeodesicRadius, GrowsMonotonicallyWithRadius)
{
  const std::set<int> small = asSet(facesWithinGeodesicRadius(adjacency_, geometry_.centroids, seed_, 2 * kStep));
  const std::set<int> medium = asSet(facesWithinGeodesicRadius(adjacency_, geometry_.centroids, seed_, 4 * kStep));
  const std::set<int> large = asSet(facesWithinGeodesicRadius(adjacency_, geometry_.centroids, seed_, 8 * kStep));

  EXPECT_LT(small.size(), medium.size());
  EXPECT_LT(medium.size(), large.size());
  EXPECT_TRUE(std::includes(medium.begin(), medium.end(), small.begin(), small.end()));
  EXPECT_TRUE(std::includes(large.begin(), large.end(), medium.begin(), medium.end()));
}

/**
 * @brief Every reached face lies within the radius as the crow flies.
 * @details The graph distance runs centroid to centroid, a staircase that is never shorter than the
 * straight line, so a face reached within a geodesic radius is within the same euclidean radius.
 * The converse is false, which is exactly why the criterion follows the surface.
 */
TEST_F(GeodesicRadius, EveryReachedFaceIsWithinTheEuclideanRadius)
{
  const double radius = 6 * kStep;
  const std::vector<int> region = facesWithinGeodesicRadius(adjacency_, geometry_.centroids, seed_, radius);

  ASSERT_FALSE(region.empty());
  const Eigen::Vector3d& origin = geometry_.centroids[static_cast<std::size_t>(seed_)];
  for (const int face : region)
  {
    const double straight = (geometry_.centroids[static_cast<std::size_t>(face)] - origin).norm();
    EXPECT_LE(straight, radius + 1e-9) << "face " << face << " is farther than the radius";
  }
}

/** @brief A radius larger than the mesh reaches every face of a connected mesh */
TEST_F(GeodesicRadius, LargeRadiusReachesTheWholeConnectedMesh)
{
  const std::vector<int> region = facesWithinGeodesicRadius(adjacency_, geometry_.centroids, seed_, 100.0);
  EXPECT_EQ(region.size(), mesh_.polygons.size());
}

/** @brief Malformed inputs return nothing rather than reading out of bounds */
TEST_F(GeodesicRadius, RejectsInvalidInputs)
{
  EXPECT_TRUE(facesWithinGeodesicRadius(adjacency_, geometry_.centroids, -1, 1.0).empty());
  EXPECT_TRUE(
      facesWithinGeodesicRadius(adjacency_, geometry_.centroids, static_cast<int>(adjacency_.size()), 1.0).empty());

  const std::vector<Eigen::Vector3d> too_few(adjacency_.size() - 1, Eigen::Vector3d::Zero());
  EXPECT_TRUE(facesWithinGeodesicRadius(adjacency_, too_few, seed_, 1.0).empty());
}

/**
 * @brief The search never crosses to a component that is close by but not connected.
 * @details The two plates sit 1 mm apart, so a euclidean neighborhood of any radius above 1 mm
 * would swallow the second one. This is the test that tells a geodesic brush from a euclidean one,
 * and the reason the brush cannot paint the far side of a sheet.
 */
TEST(GeodesicRadiusAcrossComponents, StaysInsideItsOwnComponent)
{
  const pcl::PolygonMesh first = makeGrid(11, 11, 0.0);
  const std::size_t faces_in_first = first.polygons.size();
  const pcl::PolygonMesh both = mergeMeshes(first, makeGrid(11, 11, 0.001));

  const FaceAdjacency adjacency = buildFaceAdjacency(both);
  const FaceGeometry geometry = computeFaceGeometry(both);

  const std::vector<int> region = facesWithinGeodesicRadius(adjacency, geometry.centroids, 0, 100.0);

  EXPECT_EQ(region.size(), faces_in_first);
  for (const int face : region)
  {
    EXPECT_LT(static_cast<std::size_t>(face), faces_in_first) << "face " << face << " belongs to the other plate";
  }
}

/**
 * @brief On a folded strip, the far arm is reached by its path length, not by its proximity.
 * @details The two arms of the hairpin are 1 mm apart, and the strip is 1 m long. A brush of any
 * radius under a metre must stay on the arm it was put on. A criterion measuring the straight-line
 * distance from the seed would cross to the other arm immediately, and the operator would find the
 * back of the fold painted without ever having touched it.
 */
TEST(GeodesicRadiusOnAFold, DoesNotBleedOntoTheFacingArm)
{
  const double length = 0.5;
  const double gap = 0.001;
  const pcl::PolygonMesh mesh = makeHairpinStrip(51, length, gap);

  const FaceAdjacency adjacency = buildFaceAdjacency(mesh);
  const FaceGeometry geometry = computeFaceGeometry(mesh);
  const int last_face = static_cast<int>(mesh.polygons.size()) - 1;

  // Face 0 and the last face face each other across the gap.
  const double straight = (geometry.centroids[0] - geometry.centroids[static_cast<std::size_t>(last_face)]).norm();
  ASSERT_LT(straight, 5 * gap) << "the two arms must be close in space for this test to mean anything";

  const std::set<int> narrow = asSet(facesWithinGeodesicRadius(adjacency, geometry.centroids, 0, 10 * gap));
  EXPECT_EQ(narrow.count(last_face), 0U) << "a 10 mm brush crossed a 1 mm gap it should have walked around";

  // Long enough to travel the whole strip: the far arm becomes reachable, proving it is connected
  // and that the test above measured the path and not a missing link.
  const std::set<int> wide = asSet(facesWithinGeodesicRadius(adjacency, geometry.centroids, 0, 4 * length));
  EXPECT_EQ(wide.count(last_face), 1U) << "the far arm is unreachable at any radius: the graph is broken";
}

/**
 * @brief Flooding from any face of a component yields that whole component.
 * @details Checked on the project's own test mesh, which carries a hole. The property needs no
 * knowledge of the geometry: reachability is symmetric and transitive, so an unbounded flood from
 * any member of a component must return the same set. It fails as soon as the graph is wrong.
 */
TEST(GeodesicRadiusOnRealMesh, FloodIsIndependentOfTheSeed)
{
  const pcl::PolygonMesh mesh = loadMesh(std::filesystem::path(MESH_DIR) / "wavy_mesh_with_hole.ply");
  ASSERT_GT(mesh.polygons.size(), 100U);

  const FaceAdjacency adjacency = buildFaceAdjacency(mesh);
  const FaceGeometry geometry = computeFaceGeometry(mesh);

  const std::vector<int> from_first = facesWithinGeodesicRadius(adjacency, geometry.centroids, 0, 1.0e6);
  ASSERT_GT(from_first.size(), 1U);

  const int other_seed = from_first[from_first.size() / 2];
  const std::vector<int> from_other = facesWithinGeodesicRadius(adjacency, geometry.centroids, other_seed, 1.0e6);

  EXPECT_EQ(asSet(from_first), asSet(from_other));
}

// ─────────────────────────────────────────────────────────────────────────────
// pointInPolygon
// ─────────────────────────────────────────────────────────────────────────────

/** @brief A convex square: inside is inside, and each side's outside is outside */
TEST(PointInPolygon, HandlesASquare)
{
  const std::vector<Eigen::Vector2d> square = {
    { 0.0, 0.0 },
    { 10.0, 0.0 },
    { 10.0, 10.0 },
    { 0.0, 10.0 },
  };

  EXPECT_TRUE(pointInPolygon({ 5.0, 5.0 }, square));
  EXPECT_TRUE(pointInPolygon({ 0.1, 0.1 }, square));
  EXPECT_FALSE(pointInPolygon({ -1.0, 5.0 }, square));
  EXPECT_FALSE(pointInPolygon({ 11.0, 5.0 }, square));
  EXPECT_FALSE(pointInPolygon({ 5.0, -1.0 }, square));
  EXPECT_FALSE(pointInPolygon({ 5.0, 11.0 }, square));
}

/**
 * @brief A concave outline: the notch is outside even though it is inside the bounding box.
 * @details This is the case a bounding-box test would get wrong, and the reason the lasso needs a
 * real crossing count: an operator who traces around a bracket expects the gap to stay unselected.
 */
TEST(PointInPolygon, ExcludesTheNotchOfAConcaveOutline)
{
  // An L, occupying the bottom row and the left column of a 10 x 10 box.
  const std::vector<Eigen::Vector2d> el = {
    { 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 3.0 }, { 3.0, 3.0 }, { 3.0, 10.0 }, { 0.0, 10.0 },
  };

  EXPECT_TRUE(pointInPolygon({ 5.0, 1.5 }, el)) << "the bottom arm";
  EXPECT_TRUE(pointInPolygon({ 1.5, 5.0 }, el)) << "the left arm";
  EXPECT_FALSE(pointInPolygon({ 6.0, 6.0 }, el)) << "the notch, inside the bounding box";
}

/**
 * @brief A slanted outline: the crossing abscissa has to be interpolated along the edge.
 * @details Every other outline in this file is axis-aligned, where taking an edge's endpoint
 * instead of the true crossing point happens to give the right answer. A freehand lasso is made of
 * nothing but oblique edges, so this is the case that actually runs in production.
 */
TEST(PointInPolygon, InterpolatesAlongObliqueEdges)
{
  // A right triangle: the hypotenuse runs from (10, 0) to (0, 10), so x + y = 10 inside.
  const std::vector<Eigen::Vector2d> wedge = {
    { 0.0, 0.0 },
    { 10.0, 0.0 },
    { 0.0, 10.0 },
  };

  // Points straddling the hypotenuse, at a height where the endpoint of the edge (x = 10 or x = 0)
  // would place the crossing far from its true position (x = 3).
  EXPECT_TRUE(pointInPolygon({ 2.0, 7.0 }, wedge)) << "just inside the hypotenuse";
  EXPECT_FALSE(pointInPolygon({ 4.0, 7.0 }, wedge)) << "just outside the hypotenuse";
  EXPECT_TRUE(pointInPolygon({ 1.0, 1.0 }, wedge)) << "deep inside";
  EXPECT_FALSE(pointInPolygon({ 9.0, 9.0 }, wedge)) << "outside, beyond the hypotenuse";
}

/** @brief An outline that cannot enclose anything encloses nothing */
TEST(PointInPolygon, RejectsDegenerateOutlines)
{
  EXPECT_FALSE(pointInPolygon({ 0.0, 0.0 }, {}));
  EXPECT_FALSE(pointInPolygon({ 0.0, 0.0 }, { { 0.0, 0.0 } }));
  EXPECT_FALSE(pointInPolygon({ 0.0, 0.0 }, { { -1.0, 0.0 }, { 1.0, 0.0 } }));
}

// ─────────────────────────────────────────────────────────────────────────────
// assignFacesToRegion and clearFacesFromRegions
// ─────────────────────────────────────────────────────────────────────────────

/** @brief Painting on unselected faces creates the region without disturbing the rest */
TEST(AssignFacesToRegion, PaintsOnUnselectedFaces)
{
  std::vector<int> face_region = { -1, -1, -1, -1 };

  assignFacesToRegion(face_region, { 1, 2 }, 7);

  EXPECT_EQ(face_region, std::vector<int>({ -1, 7, 7, -1 }));
}

/**
 * @brief A stroke touching two regions fuses them entirely, not only where it passed.
 * @details Two regions mean two tool paths, each paying an approach and a departure. A stroke that
 * bridges two panels is the operator saying they are one surface, so the whole of both must follow,
 * including the faces the stroke never covered.
 */
TEST(AssignFacesToRegion, FusesEveryRegionTheStrokeTouches)
{
  //            f0  f1  f2  f3  f4  f5
  //            <region 0>  <gap> <region 1>
  std::vector<int> face_region = { 0, 0, -1, -1, 1, 1 };

  // The stroke covers the gap and one face of each region.
  assignFacesToRegion(face_region, { 1, 2, 3, 4 }, 9);

  EXPECT_EQ(face_region, std::vector<int>({ 9, 9, 9, 9, 9, 9 })) << "the untouched faces of both regions must follow";
}

/** @brief A stroke that touches nothing else leaves the other regions alone */
TEST(AssignFacesToRegion, LeavesUntouchedRegionsAlone)
{
  std::vector<int> face_region = { 0, 0, -1, -1, 1, 1 };

  assignFacesToRegion(face_region, { 2 }, 9);

  EXPECT_EQ(face_region, std::vector<int>({ 0, 0, 9, -1, 1, 1 }));
}

/** @brief Out-of-range indices and negative region ids are ignored rather than fatal */
TEST(AssignFacesToRegion, IgnoresInvalidArguments)
{
  std::vector<int> face_region = { -1, 3, -1 };

  assignFacesToRegion(face_region, { -1, 99, 1000000 }, 5);
  EXPECT_EQ(face_region, std::vector<int>({ -1, 3, -1 })) << "no valid face was named";

  assignFacesToRegion(face_region, { 0 }, -2);
  EXPECT_EQ(face_region, std::vector<int>({ -1, 3, -1 })) << "a negative region id is not a selection";
}

/** @brief Erasing clears the named faces only, and keeps the rest of their region */
TEST(ClearFacesFromRegions, ClearsOnlyTheNamedFaces)
{
  std::vector<int> face_region = { 4, 4, 4, 4 };

  clearFacesFromRegions(face_region, { 1, 2, -1, 99 });

  EXPECT_EQ(face_region, std::vector<int>({ 4, -1, -1, 4 }));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
