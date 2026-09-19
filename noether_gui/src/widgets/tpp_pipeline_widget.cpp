#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_tpp/tool_path_modifiers/compound_modifier.h>
#include "ui_tpp_pipeline_widget.h"
#include <noether_gui/widgets/plugin_loader_widget.h>
#include <noether_gui/plugin_interface.h>
#include <noether_gui/utils.h>

#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <QMenu>
#include <QMessageBox>
#include <yaml-cpp/yaml.h>

static const std::string MESH_MODIFIERS_KEY = "mesh_modifiers";
static const std::string TOOL_PATH_PLANNER_KEY = "tool_path_planner";
static const std::string TOOL_PATH_MODIFIERS_KEY = "tool_path_modifiers";
static const std::string DIRECTION_GENERATOR_KEY = "direction_generator";
static const std::string FIXED_DIRECTION_PLUGIN = "FixedDirection";

namespace noether
{
TPPPipelineWidget::TPPPipelineWidget(std::shared_ptr<const WidgetFactory> factory, QWidget* parent)
  : BaseWidget(parent)
  , factory_(factory)
  , ui_(new Ui::TPPPipeline())
  , mesh_modifier_loader_widget_(new PluginLoaderWidget<MeshModifierWidgetPlugin>(factory, "Mesh Modifier", this))
  , tool_path_modifier_loader_widget_(
        new PluginLoaderWidget<ToolPathModifierWidgetPlugin>(factory, "Tool Path Modifier", this))
{
  ui_->setupUi(this);

  // Populate the combo boxes
  QStringList plugin_names = toQStringList(factory_->getAvailablePlugins<ToolPathPlannerWidgetPlugin>());
  plugin_names.sort();
  ui_->combo_box_tpp->addItems(plugin_names);

  // Add the TPP plugins
  for (const QString& plugin_name : plugin_names)
  {
    if (plugin_name.isEmpty())
    {
      ui_->stacked_widget->addWidget(new QWidget(this));
    }
    else
    {
      ui_->stacked_widget->addWidget(factory->createToolPathPlannerWidget(plugin_name.toStdString(), {}, this));
    }
  }

  // // Change the stacked widget with the combo box
  connect(ui_->combo_box_tpp,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          ui_->stacked_widget,
          &QStackedWidget::setCurrentIndex);

  // Add the tool path modifier loader widget
  {
    auto layout = new QVBoxLayout();
    layout->addWidget(tool_path_modifier_loader_widget_);
    ui_->tab_tool_path_modifier->setLayout(layout);
  }

  // Add the mesh modifier loader widget
  {
    auto layout = new QVBoxLayout();
    layout->addWidget(mesh_modifier_loader_widget_);
    ui_->tab_mesh_modifier->setLayout(layout);
  }
}

void TPPPipelineWidget::configure(const YAML::Node& config)
{
  try
  {
    // Mesh modifier
    mesh_modifier_loader_widget_->configure(config[MESH_MODIFIERS_KEY]);

    // Tool path planner
    try
    {
      auto tpp_config = config[TOOL_PATH_PLANNER_KEY];

      // Prefer to load the GUI plugin from the key "gui_plugin_name" first
      const std::string name_key = tpp_config["gui_plugin_name"] ? "gui_plugin_name" : "name";
      auto name = YAML::getMember<std::string>(tpp_config, name_key);

      int index = ui_->combo_box_tpp->findText(QString::fromStdString(name));
      if (index >= 0)
      {
        ui_->combo_box_tpp->setCurrentIndex(index);
        auto* tpp_widget = dynamic_cast<BaseWidget*>(ui_->stacked_widget->widget(index));
        if (tpp_widget)
          tpp_widget->configure(tpp_config);
      }
      else
      {
        const std::vector<std::string> tpp_plugins = factory_->getAvailablePlugins<ToolPathPlannerWidgetPlugin>();
        std::stringstream ss;
        ss << "Failed to find tool path planner '" << name << "'. Available plugins:\n";
        for (const std::string& tpp_plugin : tpp_plugins)
          ss << "    - " << tpp_plugin << "\n";

        throw std::runtime_error(ss.str());
      }
    }
    catch (const std::exception&)
    {
      ui_->combo_box_tpp->setCurrentIndex(0);
      std::throw_with_nested(std::runtime_error("Error configuring tool path planner: "));
    }

    // Tool path modifiers
    tool_path_modifier_loader_widget_->configure(config[TOOL_PATH_MODIFIERS_KEY]);
  }
  catch (const std::exception& ex)
  {
    // Clear the widgets
    ui_->combo_box_tpp->setCurrentIndex(0);
    mesh_modifier_loader_widget_->removeWidgets();
    tool_path_modifier_loader_widget_->removeWidgets();

    std::stringstream ss;
    printException(ex, ss);
    QMessageBox::warning(this, "Configuration Error", QString::fromStdString(ss.str()));
  }
}

void TPPPipelineWidget::save(YAML::Node& config) const
{
  // Mesh modifier
  {
    YAML::Node mm_config;
    mesh_modifier_loader_widget_->save(mm_config);
    config[MESH_MODIFIERS_KEY] = mm_config;
  }

  // Tool path planner
  {
    auto tpp_widget = dynamic_cast<const BaseWidget*>(ui_->stacked_widget->currentWidget());
    if (!tpp_widget)
      throw std::runtime_error("No tool path planner selected");

    YAML::Node tpp_config;
    tpp_config["name"] = ui_->combo_box_tpp->currentText().toStdString();
    tpp_widget->save(tpp_config);
    config[TOOL_PATH_PLANNER_KEY] = tpp_config;
  }

  // Tool path modifiers
  {
    YAML::Node tpm_config;
    tool_path_modifier_loader_widget_->save(tpm_config);
    config[TOOL_PATH_MODIFIERS_KEY] = tpm_config;
  }
}

ToolPathPlannerPipeline TPPPipelineWidget::createPipeline() const
{
  YAML::Node config;
  save(config);
  return ToolPathPlannerPipeline(*factory_, config);
}

namespace
{
/// @brief Greffons d'organisation que le depart et l'arrivee remplacent
const char* const kOrganizationPlugins[] = { "RasterOrganization", "SnakeOrganization" };

/// @brief Vrai si ce modificateur de la configuration est un organisateur de lignes
bool isOrganization(const YAML::Node& modifier)
{
  const std::string name = modifier["name"].as<std::string>();
  for (const char* plugin : kOrganizationPlugins)
  {
    if (name == plugin)
    {
      return true;
    }
  }
  return false;
}

/// @brief `first` suivi des `others` : sans organisateur dans la configuration, le notre passe en
/// tete, pour voir les lignes telles que le planificateur les rend, avant toute concatenation
YAML::Node fronted(const YAML::Node& first, const YAML::Node& others)
{
  YAML::Node result(YAML::NodeType::Sequence);
  result.push_back(first);
  for (const YAML::Node& modifier : others)
  {
    result.push_back(modifier);
  }
  return result;
}

/// @brief La liste des modificateurs, les organisateurs remplaces par StartEndOrganization
YAML::Node withStartEndOrganization(const YAML::Node& modifiers, const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
  YAML::Node organizer;
  organizer["name"] = "StartEndOrganization";
  organizer["start"] = start;
  organizer["end"] = end;

  YAML::Node result(YAML::NodeType::Sequence);
  bool inserted = false;
  for (const YAML::Node& modifier : modifiers)
  {
    if (!isOrganization(modifier))
    {
      result.push_back(modifier);
    }
    else if (!inserted)
    {
      result.push_back(organizer);
      inserted = true;
    }
  }
  return inserted ? result : fronted(organizer, result);
}
}  // namespace

ToolPathPlannerPipeline TPPPipelineWidget::createPipeline(const Eigen::Vector3d* raster_direction,
                                                          const Eigen::Vector3d& start,
                                                          const Eigen::Vector3d& end) const
{
  YAML::Node config;
  save(config);
  if (raster_direction != nullptr)
  {
    YAML::Node planner = config[TOOL_PATH_PLANNER_KEY];
    if (!planner[DIRECTION_GENERATOR_KEY])
    {
      throw std::runtime_error("Planner '" + planner["name"].as<std::string>() +
                               "' takes no raster direction, so a per-surface direction cannot apply to it.");
    }
    YAML::Node direction_generator;
    direction_generator["name"] = FIXED_DIRECTION_PLUGIN;
    direction_generator["direction"] = *raster_direction;
    planner[DIRECTION_GENERATOR_KEY] = direction_generator;
  }
  config[TOOL_PATH_MODIFIERS_KEY] = withStartEndOrganization(config[TOOL_PATH_MODIFIERS_KEY], start, end);
  return ToolPathPlannerPipeline(*factory_, config);
}

ToolPathModifier::ConstPtr TPPPipelineWidget::createToolPathModifierWithoutOrganization() const
{
  // Seul l'onglet des modificateurs de chemin est lu : le planificateur n'est pas sollicite
  YAML::Node modifiers_config;
  tool_path_modifier_loader_widget_->save(modifiers_config);
  std::vector<ToolPathModifier::ConstPtr> modifiers;
  for (const YAML::Node& modifier : modifiers_config)
  {
    if (!isOrganization(modifier))
    {
      modifiers.push_back(factory_->createToolPathModifier(modifier));
    }
  }
  return std::make_unique<CompoundModifier>(std::move(modifiers));
}

ToolPathPlannerPipeline TPPPipelineWidget::createPipeline(const Eigen::Vector3d& raster_direction) const
{
  YAML::Node config;
  save(config);

  YAML::Node planner = config[TOOL_PATH_PLANNER_KEY];
  if (!planner[DIRECTION_GENERATOR_KEY])
  {
    throw std::runtime_error("Planner '" + planner["name"].as<std::string>() +
                             "' takes no raster direction, so a per-surface direction cannot apply to it.");
  }

  // Le generateur est REMPLACE, pas regle : celui qui est en place peut etre un generateur
  // calcule, l'axe principal par exemple, qui n'a aucune direction a recevoir. Fixer la
  // direction est justement ce que l'appelant demande.
  YAML::Node direction_generator;
  direction_generator["name"] = FIXED_DIRECTION_PLUGIN;
  direction_generator["direction"] = raster_direction;
  planner[DIRECTION_GENERATOR_KEY] = direction_generator;

  return ToolPathPlannerPipeline(*factory_, config);
}

}  // namespace noether
