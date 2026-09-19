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
#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/tpp_pipeline_widget.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QApplication>
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <yaml-cpp/yaml.h>

using namespace noether;

namespace
{
/** @brief Configuration du projet, celle que le mainteneur ouvre reellement */
const char* kProjectConfig = R"(
mesh_modifiers:
  - name: NormalsFromMeshFaces
tool_path_planner:
  name: PlaneSlicerLegacy
  direction_generator:
    name: FixedDirection
    direction: {x: 1, y: 0, z: 0}
  origin_generator:
    name: Centroid
  line_spacing: 0.15
  point_spacing: 0.025
  min_hole_size: 0.05
  min_segment_size: 0.05
  bidirectional: true
tool_path_modifiers:
  - name: RasterOrganization
)";

/** @brief Fabrique chargee des greffons des deux paquets */
std::shared_ptr<const WidgetFactory> makeFactory()
{
  auto loader = std::make_shared<boost_plugin_loader::PluginLoader>();
  loader->search_libraries.emplace_back(NOETHER_PLUGIN_LIB);
  loader->search_libraries.emplace_back(NOETHER_GUI_PLUGIN_LIB);
  loader->search_paths.emplace_back(GUI_PLUGIN_DIR);
  loader->search_paths.emplace_back(TPP_PLUGIN_DIR);
  return std::make_shared<const WidgetFactory>(loader);
}

/** @brief Cote du plan d'essai le long de x, en metres */
const double kPlaneWidth = 0.62;

/** @brief Cote du plan d'essai le long de y, en metres */
const double kPlaneHeight = 0.43;

/**
 * @brief Plan rectangulaire dans z = 0, maille tous les @p step metres.
 * @details Les deux cotes sont volontairement differents, et aucun n'est un multiple du pas de
 * balayage : sur un carre parfait, un plan de coupe tombe exactement sur une arete du bord et le
 * decoupeur en sort un segment degenere, a coordonnees non finies.
 */
pcl::PolygonMesh rectangularPlane(double width, double height, double step)
{
  const int columns = static_cast<int>(width / step) + 1;
  const int rows = static_cast<int>(height / step) + 1;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int row = 0; row < rows; ++row)
  {
    for (int column = 0; column < columns; ++column)
    {
      cloud.push_back(pcl::PointXYZ(static_cast<float>(column * step), static_cast<float>(row * step), 0.0F));
    }
  }

  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  for (int row = 0; row + 1 < rows; ++row)
  {
    for (int column = 0; column + 1 < columns; ++column)
    {
      const std::uint32_t a = static_cast<std::uint32_t>(row * columns + column);
      const std::uint32_t c = a + static_cast<std::uint32_t>(columns);
      pcl::Vertices first;
      first.vertices = { a, a + 1, c + 1 };
      pcl::Vertices second;
      second.vertices = { a, c + 1, c };
      mesh.polygons.push_back(first);
      mesh.polygons.push_back(second);
    }
  }
  return mesh;
}

/** @brief Axe le long duquel une course progresse le plus, 0 pour x, 1 pour y, 2 pour z */
int dominantAxis(const Eigen::Vector3d& run)
{
  int axis = 0;
  for (int i = 1; i < 3; ++i)
  {
    if (std::abs(run[i]) > std::abs(run[axis]))
    {
      axis = i;
    }
  }
  return axis;
}

/** @brief Course de chaque raster planifie sur le plan d'essai, avec la direction donnee */
std::vector<Eigen::Vector3d> rasterRuns(const ToolPathPlannerPipeline& pipeline)
{
  const pcl::PolygonMesh plane = rectangularPlane(kPlaneWidth, kPlaneHeight, 0.01);
  const std::vector<pcl::PolygonMesh> meshes = pipeline.mesh_modifier->modify(plane);
  EXPECT_FALSE(meshes.empty());

  std::vector<Eigen::Vector3d> runs;
  const ToolPaths paths = pipeline.planner->plan(meshes.front());
  for (std::size_t p = 0; p < paths.size(); ++p)
  {
    for (std::size_t seg = 0; seg < paths[p].size(); ++seg)
    {
      if (paths[p][seg].size() >= 2)
      {
        runs.push_back(paths[p][seg].back().translation() - paths[p][seg].front().translation());
      }
    }
  }
  return runs;
}

/**
 * @brief Verifie que tous les rasters suivent l'axe attendu, sur la longueur attendue.
 * @param runs Course de chaque raster
 * @param axis Axe attendu, 0 pour x, 1 pour y
 * @param length Longueur attendue d'un raster, en metres
 */
void expectRastersAlong(const std::vector<Eigen::Vector3d>& runs, int axis, double length)
{
  ASSERT_FALSE(runs.empty());
  for (std::size_t i = 0; i < runs.size(); ++i)
  {
    ASSERT_TRUE(runs[i].allFinite()) << "raster " << i;
    EXPECT_EQ(dominantAxis(runs[i]), axis) << "raster " << i << " ne suit pas l'axe attendu";
    // La longueur confirme que la direction a bien bascule : un raster le long de x traverse la
    // largeur, un raster le long de y traverse la hauteur, et les deux different.
    EXPECT_NEAR(std::abs(runs[i][axis]), length, 1e-6) << "raster " << i;
  }
}

}  // namespace

TEST(PipelineDirection, TheRasterFollowsTheDirectionItIsGiven)
{
  TPPPipelineWidget widget(makeFactory());
  widget.configure(YAML::Load(kProjectConfig));

  const Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d y_axis(0.0, 1.0, 0.0);

  const std::vector<Eigen::Vector3d> along_x = rasterRuns(widget.createPipeline(x_axis));
  expectRastersAlong(along_x, 0, kPlaneWidth);

  const std::vector<Eigen::Vector3d> along_y = rasterRuns(widget.createPipeline(y_axis));
  expectRastersAlong(along_y, 1, kPlaneHeight);

  // Le nombre de rasters bascule avec la direction : le pas de balayage traverse la hauteur
  // dans un cas, la largeur dans l'autre. C'est un controle qui ne depend pas de l'orientation.
  EXPECT_NE(along_x.size(), along_y.size());
}

TEST(PipelineDirection, TheConfiguredDirectionIsLeftAloneWhenNoneIsGiven)
{
  TPPPipelineWidget widget(makeFactory());
  widget.configure(YAML::Load(kProjectConfig));

  // La configuration porte x : sans consigne, c'est elle qui decide
  expectRastersAlong(rasterRuns(widget.createPipeline()), 0, kPlaneWidth);
}

TEST(PipelineDirection, ADirectionIsRefusedByAPlannerThatTakesNone)
{
  // Le planificateur d'aretes ne balaie pas une surface : il n'a pas de direction a recevoir,
  // et le lui imposer doit se refuser au lieu d'etre ignore en silence.
  //
  // Le nom du greffon doit etre exact : TPPPipelineWidget::configure avale l'erreur dans une
  // QMessageBox modale, qui BLOQUE une execution sans ecran au lieu d'echouer.
  TPPPipelineWidget widget(makeFactory());
  widget.configure(YAML::Load("mesh_modifiers: []\n"
                              "tool_path_planner:\n"
                              "  name: Boundary\n"
                              "tool_path_modifiers: []\n"));

  EXPECT_THROW(widget.createPipeline(Eigen::Vector3d::UnitY()), std::runtime_error);
}

TEST(PipelineDirection, ANullDirectionProducesNothingUsable)
{
  // Ce que le generateur fait d'un vecteur nul, et pourquoi la fenetre le refuse avant de
  // planifier : la normalisation en sort des NaN, sans qu'aucune erreur ne soit levee.
  TPPPipelineWidget widget(makeFactory());
  widget.configure(YAML::Load(kProjectConfig));

  const Eigen::Vector3d nothing(0.0, 0.0, 0.0);
  const std::vector<Eigen::Vector3d> runs = rasterRuns(widget.createPipeline(nothing));

  bool all_finite = true;
  for (std::size_t i = 0; i < runs.size(); ++i)
  {
    all_finite = all_finite && runs[i].allFinite();
  }
  EXPECT_TRUE(runs.empty() || !all_finite) << "un vecteur nul devrait ne rien donner d'exploitable";
}

namespace
{
/** @brief Ecart d'un chemin aux deux points demandes : premiere pose au depart, derniere a l'arrivee */
double endpointsCost(const std::vector<ToolPaths>& paths, const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
  const Eigen::Vector3d first = paths.front().front().front().front().translation();
  const Eigen::Vector3d last = paths.front().back().back().back().translation();
  return (first - start).norm() + (last - end).norm();
}
}  // namespace

TEST(PipelineDirection, StartAndEndReplaceTheOrganizersOfTheConfiguration)
{
  TPPPipelineWidget widget(makeFactory());
  widget.configure(YAML::Load(kProjectConfig));
  const pcl::PolygonMesh plane = rectangularPlane(kPlaneWidth, kPlaneHeight, 0.01);

  // Depart au coin bas gauche, arrivee au coin haut gauche du plan
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d end(0.0, kPlaneHeight, 0.0);
  const std::vector<ToolPaths> organized = widget.createPipeline(nullptr, start, end).plan(plane);
  const std::vector<ToolPaths> configured = widget.createPipeline().plan(plane);

  ASSERT_FALSE(organized.empty());
  ASSERT_FALSE(organized.front().empty());
  EXPECT_EQ(organized.front().size(), configured.front().size()) << "toutes les lignes sont conservees";
  // L'organisateur minimise la somme des deux ecarts : il ne peut pas faire pire que l'ordre de la
  // configuration, et ici il tient le depart exactement
  EXPECT_LE(endpointsCost(organized, start, end), endpointsCost(configured, start, end) + 1e-9);
  const Eigen::Vector3d first = organized.front().front().front().front().translation();
  EXPECT_LT((first - start).norm(), 0.10) << "premiere pose " << first.transpose();
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
