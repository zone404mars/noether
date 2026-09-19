#pragma once

#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/plugin_loader_widget.h>

#include <noether_tpp/core/tool_path_planner_pipeline.h>

#include <Eigen/Core>

namespace Ui
{
class TPPPipeline;
}

namespace noether
{
/**
 * @brief Widget for creating a tool path planning pipeline
 */
class TPPPipelineWidget : public BaseWidget
{
public:
  TPPPipelineWidget(std::shared_ptr<const WidgetFactory> factory, QWidget* parent = nullptr);

  ToolPathPlannerPipeline createPipeline() const;

  /**
   * @brief Builds a pipeline from the current settings, with the raster direction replaced
   * @details Identical to ::createPipeline in every other respect. The raster direction decides
   * which way the lines run across a surface, and one workpiece often wants a different answer on
   * each of its surfaces: sanding along a panel and across the one beside it is a normal demand.
   * Replacing it per surface costs one pipeline per surface, which is nothing next to the planning
   * itself.
   * @param raster_direction Direction the raster lines follow, in the mesh frame
   * @throws std::runtime_error if the selected planner takes no direction generator
   */
  ToolPathPlannerPipeline createPipeline(const Eigen::Vector3d& raster_direction) const;
  /**
   * @brief Pipeline pour une surface dont le depart et l'arrivee sont fixes
   * @details Les organisateurs RasterOrganization et SnakeOrganization de la configuration
   * decident de l'ordre des lignes ; ils sont remplaces par StartEndOrganization, insere a la
   * place du premier d'entre eux, ou en tete des modificateurs s'il n'y en a pas. Toutes les
   * lignes sont conservees, la couverture aussi.
   * @param raster_direction Direction de passe de la surface, ou nullptr pour celle du pipeline
   * @param start Point pres duquel le chemin commence
   * @param end Point pres duquel il finit
   */
  ToolPathPlannerPipeline createPipeline(const Eigen::Vector3d* raster_direction,
                                         const Eigen::Vector3d& start,
                                         const Eigen::Vector3d& end) const;

  /**
   * @brief Modificateur de chemin pour une surface tracee a la main
   * @details Le trace remplace le planificateur : ni lui ni le modificateur de maillage ne sont
   * construits, leur configuration peut donc etre incomplete sans gener. Le trace fixe aussi
   * l'ordre et le sens de marche : les organisateurs (RasterOrganization, SnakeOrganization)
   * sont retires. Les autres modificateurs restent, en particulier l'approche et le retrait que
   * le consommateur du chemin exige.
   */
  ToolPathModifier::ConstPtr createToolPathModifierWithoutOrganization() const;

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  std::shared_ptr<const WidgetFactory> factory_;
  PluginLoaderWidget<MeshModifierWidgetPlugin>* mesh_modifier_loader_widget_;
  PluginLoaderWidget<ToolPathModifierWidgetPlugin>* tool_path_modifier_loader_widget_;
  Ui::TPPPipeline* ui_;
};

}  // namespace noether
