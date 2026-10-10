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

/**
 * @file tpp_widget_layout.cpp
 * @brief Disposition de la fenetre TPPWidget : barre du process en haut, arbre a gauche, options a
 * droite, et la langue de l'interface. La logique de selection, de trace et de generation reste
 * dans tpp_widget.cpp.
 */

#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include "tpp_widget_defaults.h"
#include <noether_gui/widgets/path_edit_widget.h>
#include <noether_gui/widgets/process_ribbon.h>
#include <noether_gui/widgets/step_import.h>

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QStatusBar>
#include <QToolBar>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <vtkCamera.h>
#include <vtkRenderer.h>

namespace
{
/** @brief Donnee d'un element de l'arbre : identifiant de la surface */
const int kSurfaceIdRole = Qt::UserRole;
/** @brief Donnee d'un element de l'arbre : nombre de faces de la surface */
const int kFaceCountRole = Qt::UserRole + 1;
/** @brief Donnee d'un element de l'arbre : rang de la passe dans le dock des passes */
const int kPassRowRole = Qt::UserRole + 2;

/** @brief Champ numerique a unite, avec ses bornes et sa valeur initiale */
QDoubleSpinBox* makeSpinBox(QWidget* parent, double minimum, double maximum, int decimals, double value, const char* suffix)
{
  auto* spin_box = new QDoubleSpinBox(parent);
  spin_box->setRange(minimum, maximum);
  spin_box->setDecimals(decimals);
  spin_box->setValue(value);
  spin_box->setSuffix(suffix);
  return spin_box;
}
}  // namespace

namespace noether
{
// ---------------------------------------------------------------------------------------------
// Langue

QString TPPWidget::say(const char* french, const char* english) const
{
  return translator_.text({ QString::fromUtf8(french), QString::fromUtf8(english) });
}

void TPPWidget::applyLanguage(const Language language)
{
  Translator::saveLanguage(language);
  translator_.setLanguage(language);
  // Les textes de l'arbre sont calcules, pas lies : ils sont recalcules
  relabelTree();
}

void TPPWidget::translateMenus()
{
  translator_.bindText(ui_->menuFile, { "Fichier", "File" });
  translator_.bindText(ui_->menuView, { "Affichage", "View" });
  translator_.bindText(ui_->menuMesh, { "Maillage", "Mesh" });
  translator_.bindText(ui_->menuTool_Path, { "Points du chemin", "Tool path waypoints" });
  translator_.bindText(ui_->menuTool_path_lines, { "Lignes du chemin", "Tool path lines" });
  translator_.bindText(ui_->menuTPP, { "Génération", "Generation" });
  for (QAction* action : { ui_->action_show_unmodified_mesh, ui_->action_show_unmodified_tool_path,
                           ui_->action_show_unmodified_tool_path_lines })
  {
    translator_.bindText(action, { "Avant modification", "Unmodified" });
  }
  for (QAction* action : { ui_->action_show_modified_mesh, ui_->action_show_modified_tool_path,
                           ui_->action_show_modified_tool_path_lines })
  {
    translator_.bindText(action, { "Après modification", "Modified" });
  }
  translateActions();
}

void TPPWidget::translateActions()
{
  translator_.bindText(ui_->action_load_mesh, { "Ouvrir une pièce...", "Open part..." });
  translator_.bindText(ui_->action_load_config, { "Charger une configuration TPP...", "Load TPP configuration..." });
  translator_.bindText(ui_->action_save_config, { "Enregistrer la configuration TPP...", "Save TPP configuration..." });
  translator_.bindText(ui_->action_save_modified_mesh, { "Enregistrer les maillages des surfaces...", "Save surface meshes..." });
  translator_.bindText(ui_->action_save_toolpath, { "Exporter le chemin...", "Export tool path..." });
  translator_.bindText(ui_->action_clear_selection, { "Effacer toutes les surfaces", "Clear all surfaces" });
  translator_.bindText(ui_->action_execute_pipeline, { "Générer", "Generate" });
  // Les infobulles du fichier .ui decrivent l'ancienne interface : elles sont retirees
  for (QAction* action : findChildren<QAction*>())
  {
    action->setToolTip(QString());
  }
}

// ---------------------------------------------------------------------------------------------
// Barre du process

void TPPWidget::buildRibbon()
{
  ribbon_ = new ProcessRibbon(translator_, this);
  // Les outils qui prennent la souris s'excluent : un seul mode de tirage a la fois
  mode_group_ = new QButtonGroup(this);
  mode_group_->setExclusive(true);
  buildFileTab(ribbon_->addTab({ "1  Fichier", "1  File" }));
  buildSelectionTab(ribbon_->addTab({ "2  Sélection", "2  Selection" }));
  buildTreatmentTab(ribbon_->addTab({ "3  Traitement", "3  Treatment" }));
  buildPathTab(ribbon_->addTab({ "4  Trajectoire", "4  Tool path" }));
  buildViewTab(ribbon_->addTab({ "Affichage", "View" }));
  ribbon_->on_tab_changed = [this](int tab) { tab_entries_.at(static_cast<std::size_t>(tab))(); };

  auto* bar = new QToolBar(this);
  bar->setMovable(false);
  bar->setFloatable(false);
  bar->addWidget(ribbon_);
  addToolBar(Qt::TopToolBarArea, bar);
  translator_.bind([this, bar]() { bar->setWindowTitle(translator_.text({ "Étapes", "Steps" })); });
}

QToolButton* TPPWidget::addModeTool(const int tab, const QString& icon, const Bilingual& label, const Bilingual& tip)
{
  QToolButton* button = ribbon_->addTool(tab, icon, label, tip);
  button->setCheckable(true);
  mode_group_->addButton(button);
  return button;
}

void TPPWidget::releaseDragTools()
{
  // L'outil de selection courant reprend la souris ; le groupe exclusif relache l'autre
  if (tracing() || movingPoints() || densifying())
  {
    buttonOfTool(tool_)->setChecked(true);
  }
}

void TPPWidget::buildFileTab(const int tab)
{
  QToolButton* open = ribbon_->addTool(tab, ":/icons/open.svg", { "Ouvrir une pièce", "Open part" },
                                       { "STEP, PLY ou STL. Un STEP est converti en maillage.",
                                         "STEP, PLY or STL. A STEP file is converted to a mesh." });
  connect(open, &QToolButton::clicked, this, [this]() {
    showOpenOptions();
    onLoadMesh(false);
  });
  QToolButton* load_path = ribbon_->addTool(tab, ":/icons/load_path.svg", { "Reprendre un chemin", "Reopen tool path" },
                                            { "Rouvre un chemin exporté pour le retoucher.",
                                              "Reopens an exported tool path to retouch it." });
  connect(load_path, &QToolButton::clicked, this, [this]() { onReopenToolPathTool(); });
  tab_entries_.push_back([this]() {
    releaseDragTools();
    showOpenOptions();
  });
}

void TPPWidget::onReopenToolPathTool()
{
  showOptions("empty", { "Reprendre un chemin", "Reopen tool path" },
              { "Les passes du fichier prennent la place d'une génération : ordre, suppression et "
                "points déplacés s'y appliquent, puis Exporter réécrit le résultat.",
                "The passes of the file replace a generation: order, removal and moved points "
                "apply to them, then Export writes the result again." });
  onLoadToolPaths();
}

void TPPWidget::buildSelectionTab(const int tab)
{
  addSelectionTools(tab);
  ribbon_->addSeparator(tab);
  QToolButton* add = ribbon_->addTool(tab, ":/icons/add_surface.svg", { "Valider la surface", "Add surface" },
                                      { "La sélection en cours devient une surface de l'arbre.",
                                        "The current selection becomes a surface of the tree." });
  connect(add, &QToolButton::clicked, this, [this]() { onAddSurfaceTool(); });
  // Un bouton deja enfonce n'emet pas toggled : l'outil est alors reapplique directement
  tab_entries_.push_back([this]() {
    QToolButton* button = buttonOfTool(tool_);
    if (button->isChecked())
    {
      selectTool(tool_);
    }
    button->setChecked(true);
  });
}

void TPPWidget::addSelectionTools(const int tab)
{
  region_button_ = addModeTool(tab, ":/icons/region.svg", { "Région", "Region" },
                               { "Clic sur une face : la région grandit tant que la surface reste peu courbée.",
                                 "Click a face: the region grows while the surface stays gently curved." });
  brush_button_ = addModeTool(tab, ":/icons/brush.svg", { "Brosse", "Brush" },
                              { "Glisser : peint les faces sous la brosse.", "Drag: paints the faces under the brush." });
  polygon_button_ = addModeTool(tab, ":/icons/polygon.svg", { "Polygone", "Polygon" },
                                { "Clics : contour posé à l'écran.", "Clicks: outline drawn on screen." });
  eraser_button_ = addModeTool(tab, ":/icons/eraser.svg", { "Gomme", "Eraser" },
                               { "Glisser : retire des faces de la sélection en cours.",
                                 "Drag: removes faces from the current selection." });
  connectSelectionTool(region_button_, SelectionTool::Region);
  connectSelectionTool(brush_button_, SelectionTool::Brush);
  connectSelectionTool(polygon_button_, SelectionTool::Polygon);
  connectSelectionTool(eraser_button_, SelectionTool::Eraser);
  // L'outil de depart est arme : la souris a toujours un outil de selection
  buttonOfTool(tool_)->setChecked(true);
}

void TPPWidget::onAddSurfaceTool()
{
  showOptions("empty", { "Valider la surface", "Add surface" },
              { "La sélection en cours, en orange, devient une nouvelle surface de l'arbre. Refusé si "
                "elle est faite de plusieurs morceaux.",
                "The current selection, in orange, becomes a new surface of the tree. Refused when it "
                "is made of several disconnected pieces." });
  onAddSurface();
}

void TPPWidget::connectSelectionTool(QToolButton* button, const SelectionTool tool)
{
  connect(button, &QToolButton::toggled, this, [this, tool](bool checked) {
    if (checked)
    {
      selectTool(tool);
    }
  });
}

QToolButton* TPPWidget::buttonOfTool(const SelectionTool tool) const
{
  switch (tool)
  {
    case SelectionTool::Brush:
      return brush_button_;
    case SelectionTool::Polygon:
      return polygon_button_;
    case SelectionTool::Eraser:
      return eraser_button_;
    default:
      return region_button_;
  }
}

void TPPWidget::selectTool(const SelectionTool tool)
{
  tool_ = tool;
  // Un contour laisse a moitie trace ne survit pas a un changement d'outil
  outline_.clear();
  updateOutlineOverlay();
  updateToolControls();
  updateTraceMarkers();
  showSelectionOptions(tool);
  render();
}

void TPPWidget::showSelectionOptions(const SelectionTool tool)
{
  if (tool == SelectionTool::Region)
  {
    showOptions("region", { "Région", "Region" },
                { "Clic sur une face : la région grandit tant que l'angle entre faces voisines reste sous le seuil.",
                  "Click a face: the region grows while the angle between neighbouring faces stays under the "
                  "threshold." });
  }
  else if (tool == SelectionTool::Polygon)
  {
    showOptions("empty", { "Polygone", "Polygon" },
                { "Clic : ajoute un sommet. Clic sur le premier, Entrée ou double-clic : ferme. Retour arrière : "
                  "retire le dernier. Échap : annule.",
                  "Click: adds a vertex. Click the first one, Enter or double-click: closes. Backspace: removes "
                  "the last one. Escape: cancels." });
  }
  else
  {
    // La brosse et la gomme partagent leur diametre, donc leur page
    showOptions("brush", tool == SelectionTool::Brush ? Bilingual{ "Brosse", "Brush" } : Bilingual{ "Gomme", "Eraser" },
                { "Glisser sur la pièce. Maj : efface. Alt : tourne la vue sans quitter l'outil.",
                  "Drag on the part. Shift: erases. Alt: orbits the view without leaving the tool." });
  }
}

void TPPWidget::buildTreatmentTab(const int tab)
{
  trace_button_ = addModeTool(tab, ":/icons/trace.svg", { "Tracé manuel", "Manual trace" },
                              { "Choisir la surface dans l'arbre, cliquer le départ, puis tirer : chaque "
                                "tirage ajoute un segment.",
                                "Pick the surface in the tree, click the start, then drag: each drag adds "
                                "a segment." });
  connect(trace_button_, &QToolButton::toggled, this, [this](bool checked) { onTraceToggled(checked); });
  QToolButton* generate = ribbon_->addTool(tab, ":/icons/generate.svg", { "Générer", "Generate" },
                                           { "Une passe par surface tracée.", "One pass per traced surface." });
  connect(generate, &QToolButton::clicked, this, [this]() { onGenerateTool(); });
  ribbon_->addSeparator(tab);
  addPointTools(tab);
  tab_entries_.push_back([this]() {
    if (trace_button_->isChecked())
    {
      onTraceToggled(true);
    }
    trace_button_->setChecked(true);
  });
}

void TPPWidget::addPointTools(const int tab)
{
  move_points_button_ = addModeTool(tab, ":/icons/move_points.svg", { "Déplacer des points", "Move points" },
                                    { "Glisser : saisit le point de contact le plus proche et le pose sur la pièce.",
                                      "Drag: grabs the nearest contact point and drops it on the part." });
  connect(move_points_button_, &QToolButton::toggled, this, [this](bool checked) { onMovePointsToggled(checked); });
  densify_button_ = addModeTool(tab, ":/icons/densify.svg", { "Densifier", "Densify" },
                                { "Clic sur deux points d'une passe : des points s'ajoutent entre eux au pas voulu.",
                                  "Click two points of a pass: points are added between them at the wanted step." });
  connect(densify_button_, &QToolButton::toggled, this, [this](bool checked) { onDensifyToggled(checked); });
}

void TPPWidget::onGenerateTool()
{
  showOptions("empty", { "Générer", "Generate" },
              { "Chaque surface tracée donne une passe, avec son approche et son retrait.",
                "Each traced surface gives one pass, with its approach and retract." });
  plan();
}

void TPPWidget::buildPathTab(const int tab)
{
  QToolButton* order = ribbon_->addTool(tab, ":/icons/order.svg", { "Ordre des passes", "Pass order" },
                                        { "Ordre d'exécution des passes par la cellule.", "Execution order of the passes." });
  connect(order, &QToolButton::clicked, this, [this]() { showOrderOptions(); });
  QToolButton* export_button = ribbon_->addTool(tab, ":/icons/export.svg", { "Exporter", "Export" },
                                                { "Écrit le chemin retouché, et la retouche à côté.",
                                                  "Writes the retouched path, and the retouching beside it." });
  connect(export_button, &QToolButton::clicked, this, [this]() {
    showOptions("empty", { "Exporter", "Export" },
                { "Écrit le chemin retouché pour la cellule, et la retouche à côté pour pouvoir la rejouer.",
                  "Writes the retouched path for the cell, and the retouching beside it so it can be replayed." });
    onSaveToolPaths(false);
  });
  tab_entries_.push_back([this]() {
    releaseDragTools();
    showOrderOptions();
  });
}

void TPPWidget::buildViewTab(const int tab)
{
  QToolButton* layers = ribbon_->addTool(tab, ":/icons/layers.svg", { "Calques", "Layers" },
                                         { "Ce qui est dessiné dans la vue.", "What the view draws." });
  connect(layers, &QToolButton::clicked, this, [this]() { showLayersOptions(); });
  QToolButton* views = ribbon_->addTool(tab, ":/icons/views.svg", { "Vues standard", "Standard views" },
                                        { "Orientation de la caméra.", "Camera orientation." });
  connect(views, &QToolButton::clicked, this, [this]() {
    showOptions("views", { "Vues standard", "Standard views" }, { "Orientation de la caméra.", "Camera orientation." });
  });
  QToolButton* language = ribbon_->addTool(tab, ":/icons/language.svg", { "Langue", "Language" },
                                           { "Langue de l'interface.", "Interface language." });
  connect(language, &QToolButton::clicked, this, [this]() {
    showOptions("language", { "Langue", "Language" },
                { "Langue de l'interface, gardée au prochain démarrage.", "Interface language, kept for the next start." });
  });
  tab_entries_.push_back([this]() {
    releaseDragTools();
    showLayersOptions();
  });
}

// ---------------------------------------------------------------------------------------------
// Panneau des options

void TPPWidget::buildOptionsDock()
{
  options_pages_ = new QStackedWidget(this);
  auto* panel = new QWidget(this);
  auto* layout = new QVBoxLayout(panel);
  createOptionsHeader(layout);
  layout->addWidget(options_pages_);
  layout->addStretch();
  auto* dock = new QDockWidget(this);
  dock->setObjectName("options_dock");
  dock->setWidget(panel);
  dock->setFeatures(QDockWidget::DockWidgetMovable);
  addDockWidget(Qt::RightDockWidgetArea, dock);
  translator_.bindText(dock, { "Options", "Options" });
  buildOptionPages();
}

void TPPWidget::createOptionsHeader(QVBoxLayout* layout)
{
  options_title_ = new QLabel(this);
  QFont title_font = options_title_->font();
  title_font.setBold(true);
  title_font.setPointSizeF(title_font.pointSizeF() * 1.15);
  options_title_->setFont(title_font);
  options_help_ = new QLabel(this);
  options_help_->setWordWrap(true);
  layout->addWidget(options_title_);
  layout->addWidget(options_help_);
  // Le titre et l'aide affiches suivent la langue
  translator_.bind([this]() {
    options_title_->setText(translator_.text(options_title_text_));
    options_help_->setText(translator_.text(options_help_text_));
  });
}

void TPPWidget::buildOptionPages()
{
  addOptionPage("empty", new QWidget(this));
  addOptionPage("open", buildOpenPage());
  addOptionPage("region", buildRegionPage());
  addOptionPage("brush", buildBrushPage());
  addOptionPage("trace", buildTracePage());
  addOptionPage("move", buildMovePage());
  addOptionPage("densify", buildDensifyPage());
  addOptionPage("order", buildOrderPage());
  addOptionPage("layers", buildLayersPage());
  addOptionPage("views", buildViewsPage());
  addOptionPage("language", buildLanguagePage());
}

void TPPWidget::addOptionPage(const std::string& id, QWidget* page)
{
  // La pile prend la hauteur de sa plus grande page : l'espace en trop va sous les champs
  auto* holder = new QWidget(this);
  auto* layout = new QVBoxLayout(holder);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(page);
  layout->addStretch();
  option_page_index_[id] = options_pages_->addWidget(holder);
}

void TPPWidget::showOptions(const std::string& id, const Bilingual& title, const Bilingual& help)
{
  options_title_text_ = title;
  options_help_text_ = help;
  options_title_->setText(translator_.text(title));
  options_help_->setText(translator_.text(help));
  options_pages_->setCurrentIndex(option_page_index_.at(id));
}

QLabel* TPPWidget::boundLabel(const Bilingual& text)
{
  auto* label = new QLabel(this);
  label->setWordWrap(true);
  translator_.bindText(label, text);
  return label;
}

QWidget* TPPWidget::labelledRow(const Bilingual& label, QWidget* field)
{
  auto* row = new QWidget(this);
  auto* layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(boundLabel(label));
  layout->addWidget(field);
  return row;
}

QWidget* TPPWidget::buildOpenPage()
{
  open_info_label_ = new QLabel(this);
  open_info_label_->setWordWrap(true);
  translator_.bind([this]() { open_info_label_->setText(stepTessellationText()); });
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(boundLabel({ "Formats acceptés : STEP (.step, .stp), PLY, STL.", "Accepted formats: STEP (.step, .stp), PLY, STL." }));
  layout->addWidget(open_info_label_);
  return page;
}

void TPPWidget::showOpenOptions()
{
  showOptions("open", { "Ouvrir une pièce", "Open part" },
              { "Un STEP est converti en maillage à la finesse du projet ; un PLY ou un STL est chargé tel quel.",
                "A STEP file is converted to a mesh at the project's tessellation; a PLY or STL file is loaded as is." });
  open_info_label_->setText(stepTessellationText());
}

QString TPPWidget::stepTessellationText() const
{
  if (step_tessellation_file_.isEmpty())
  {
    return say("Aucune finesse STEP configurée : un STEP sera refusé.", "No STEP tessellation configured: a STEP file will be refused.");
  }
  try
  {
    const StepTessellation tessellation = readStepTessellation(step_tessellation_file_.toStdString());
    return say("Finesse STEP : déflexion linéaire %1 mm, angulaire %2 rad, solides : %3.",
               "STEP tessellation: linear deflection %1 mm, angular %2 rad, solids: %3.")
        .arg(tessellation.linear_deflection_m * 1000.0)
        .arg(tessellation.angular_deflection_rad)
        .arg(QString::fromStdString(tessellation.solids));
  }
  catch (const std::exception& ex)
  {
    return QString::fromStdString(ex.what());
  }
}

QWidget* TPPWidget::buildRegionPage()
{
  flatness_angle_spin_box_ = makeSpinBox(this, 0.0, 180.0, 1, kDefaultFlatnessAngleDeg, " °");
  flatness_angle_spin_box_->setSingleStep(1.0);
  return labelledRow({ "Angle de planéité", "Flatness angle" }, flatness_angle_spin_box_);
}

QWidget* TPPWidget::buildBrushPage()
{
  // Diametre mesure sur la surface : zoomer ne change pas ce qu'un trait couvre
  brush_diameter_spin_box_ = makeSpinBox(this, 0.1, 10000.0, 1, kDefaultBrushDiameterMm, " mm");
  brush_diameter_spin_box_->setSingleStep(5.0);
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(labelledRow({ "Diamètre", "Diameter" }, brush_diameter_spin_box_));
  layout->addWidget(boundLabel({ "Réglé au diamètre du disque, la brosse montre ce qu'une passe couvre.",
                                 "Set to the disc diameter, the brush shows what one pass covers." }));
  return page;
}

QWidget* TPPWidget::buildTracePage()
{
  createTraceFields();
  auto* undo_vertex = new QPushButton(this);
  translator_.bindText(undo_vertex, { "Retirer le dernier", "Remove last" });
  connect(undo_vertex, &QPushButton::clicked, this, [this](bool) { onTraceKey("BackSpace"); });
  auto* clear_trace = new QPushButton(this);
  translator_.bindText(clear_trace, { "Effacer le tracé", "Clear trace" });
  connect(clear_trace, &QPushButton::clicked, this, [this](bool) { onTraceKey("Escape"); });

  auto* page = new QWidget(this);
  auto* layout = new QGridLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(labelledRow({ "Pas du tracé", "Trace step" }, trace_step_spin_box_), 0, 0, 1, 2);
  layout->addWidget(labelledRow({ "Hauteur d'approche", "Approach height" }, approach_height_spin_box_), 1, 0, 1, 2);
  layout->addWidget(tool_radius_enabled_check_box_, 2, 0);
  layout->addWidget(tool_radius_spin_box_, 2, 1);
  layout->addWidget(undo_vertex, 3, 0);
  layout->addWidget(clear_trace, 3, 1);
  return page;
}

void TPPWidget::createTraceFields()
{
  trace_step_spin_box_ = makeSpinBox(this, 0.1, 10000.0, 1, kDefaultTraceStepMm, " mm");
  trace_step_spin_box_->setSingleStep(5.0);
  connect(trace_step_spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
    updateTraceMarkers();
    render();
  });
  approach_height_spin_box_ = makeSpinBox(this, 1.0, 1000.0, 0, kDefaultApproachHeightMm, " mm");
  approach_height_spin_box_->setSingleStep(10.0);
  tool_radius_spin_box_ = makeSpinBox(this, 0.0, 10000.0, 1, kDefaultToolRadiusMm, " mm");
  tool_radius_enabled_check_box_ = new QCheckBox(this);
  tool_radius_enabled_check_box_->setChecked(true);
  translator_.bindText(tool_radius_enabled_check_box_, { "Rayon de l'outil", "Tool radius" });
  translator_.bindToolTip(tool_radius_enabled_check_box_,
                          { "Cercles du disque autour des points, et surfaces rognées de ce rayon à l'affichage.",
                            "Disc circles around the points, and surfaces trimmed by this radius on screen." });
  connect(tool_radius_spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
    refreshToolRadiusViews();
  });
  connect(tool_radius_enabled_check_box_, &QCheckBox::toggled, this, [this](bool enabled) {
    tool_radius_spin_box_->setEnabled(enabled);
    refreshToolRadiusViews();
  });
}

void TPPWidget::refreshToolRadiusViews()
{
  updateSelectionColors();
  updateDiscCircles();
  updateTraceMarkers();
  render();
}

QWidget* TPPWidget::buildMovePage()
{
  validate_layout_button_ = new QPushButton(this);
  translator_.bindText(validate_layout_button_, { "Valider la disposition", "Confirm layout" });
  connect(validate_layout_button_, &QPushButton::clicked, this, [this](bool) { validateLayout(); });
  auto* undo = new QPushButton(this);
  translator_.bindText(undo, { "Annuler le dernier déplacement", "Undo last move" });
  connect(undo, &QPushButton::clicked, this, [this](bool) { path_edit_->undo(); });
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(validate_layout_button_);
  layout->addWidget(undo);
  return page;
}

QWidget* TPPWidget::buildOrderPage()
{
  // En tout-manuel, seules restent les retouches d'ordre et de suppression
  path_edit_ = new PathEditWidget(kMinPassPoses, this);
  path_edit_->hideAdvancedControls();
  path_edit_->bindTexts(translator_);
  path_edit_->on_changed = [this]() { onPathEdited(); };
  path_edit_->on_selection_changed = [this]() {
    updatePathHighlight();
    selectPassItem(path_edit_->selectedPass());
  };
  refreshPassItems();
  return path_edit_;
}

void TPPWidget::showOrderOptions()
{
  showOptions("order", { "Ordre des passes", "Pass order" },
              { "Ordre dans lequel la cellule exécute les passes.", "Order in which the cell runs the passes." });
}

QWidget* TPPWidget::buildLayersPage()
{
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  addLayerBox(layout, ui_->action_show_unmodified_mesh, { "Pièce", "Part" });
  addLayerBox(layout, ui_->action_show_modified_mesh, { "Surfaces générées", "Generated surfaces" });
  addLayerBox(layout, ui_->action_show_unmodified_tool_path_lines, { "Tracés", "Traces" });
  addLayerBox(layout, ui_->action_show_modified_tool_path_lines, { "Chemin livré", "Delivered path" });
  addLayerBox(layout, ui_->action_show_modified_tool_path, { "Repères des poses", "Pose frames" });
  layout->addWidget(buildDiscGroup());
  // Taille des reperes et origine : repris du dock de configuration, cache
  layout->addWidget(ui_->groupBox);
  translator_.bind([this]() {
    ui_->groupBox->setTitle(say("Repères", "Frames"));
    ui_->label->setText(say("Taille des repères", "Axis size"));
    ui_->label_2->setText(say("Afficher l'origine", "Show origin"));
    ui_->label_3->setText(say("Taille de l'origine", "Origin size"));
  });
  return page;
}

void TPPWidget::addLayerBox(QVBoxLayout* layout, QAction* action, const Bilingual& label)
{
  // La case et l'action du menu Affichage restent d'accord dans les deux sens
  auto* box = new QCheckBox(this);
  box->setChecked(action->isChecked());
  translator_.bindText(box, label);
  connect(box, &QCheckBox::toggled, this, [action](bool checked) {
    if (action->isChecked() != checked)
    {
      action->trigger();
    }
  });
  connect(action, &QAction::toggled, box, &QCheckBox::setChecked);
  layout->addWidget(box);
}

void TPPWidget::showLayersOptions()
{
  showOptions("layers", { "Calques", "Layers" }, { "Ce qui est dessiné dans la vue.", "What the view draws." });
}

QWidget* TPPWidget::buildViewsPage()
{
  auto* page = new QWidget(this);
  auto* layout = new QGridLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  addViewButton(layout, 0, 0, { "Face", "Front" }, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d::UnitZ());
  addViewButton(layout, 0, 1, { "Dessus", "Top" }, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY());
  addViewButton(layout, 1, 0, { "Côté", "Side" }, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
  addViewButton(layout, 1, 1, { "Isométrique", "Isometric" }, Eigen::Vector3d(1, -1, 1), Eigen::Vector3d::UnitZ());
  return page;
}

void TPPWidget::addViewButton(QGridLayout* layout, const int row, const int column, const Bilingual& label,
                              const Eigen::Vector3d& direction, const Eigen::Vector3d& up)
{
  auto* button = new QPushButton(this);
  translator_.bindText(button, label);
  connect(button, &QPushButton::clicked, this, [this, direction, up](bool) { setCameraView(direction, up); });
  layout->addWidget(button, row, column);
}

void TPPWidget::setCameraView(const Eigen::Vector3d& direction, const Eigen::Vector3d& up)
{
  // La camera regarde l'origine depuis la direction donnee, puis le cadrage englobe la scene
  vtkCamera* camera = renderer_->GetActiveCamera();
  const Eigen::Vector3d position = direction.normalized();
  camera->SetFocalPoint(0.0, 0.0, 0.0);
  camera->SetPosition(position.x(), position.y(), position.z());
  camera->SetViewUp(up.x(), up.y(), up.z());
  renderer_->ResetCamera();
  render();
}

QWidget* TPPWidget::buildLanguagePage()
{
  auto* combo = new QComboBox(this);
  combo->addItem("Français", static_cast<int>(Language::FRENCH));
  combo->addItem("English", static_cast<int>(Language::ENGLISH));
  combo->setCurrentIndex(translator_.language() == Language::FRENCH ? 0 : 1);
  // Connecte apres le choix initial : la langue courante n'est pas reappliquee pour rien
  connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, combo](int index) {
    applyLanguage(static_cast<Language>(combo->itemData(index).toInt()));
  });
  return labelledRow({ "Langue", "Language" }, combo);
}

// ---------------------------------------------------------------------------------------------
// Arbre de la piece

void TPPWidget::buildFeatureTree()
{
  tree_ = new QTreeWidget(this);
  tree_->setHeaderHidden(true);
  piece_item_ = new QTreeWidgetItem(tree_);
  surfaces_item_ = new QTreeWidgetItem(tree_);
  passes_item_ = new QTreeWidgetItem(tree_);
  tree_->expandAll();
  connect(tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem*, QTreeWidgetItem*) {
    onTreeCurrentChanged();
  });
  connect(tree_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int) { onTreeItemChanged(item); });

  auto* panel = new QWidget(this);
  auto* layout = new QVBoxLayout(panel);
  layout->addWidget(tree_);
  layout->addWidget(buildSurfaceButtons());
  auto* dock = new QDockWidget(this);
  dock->setObjectName("tree_dock");
  dock->setWidget(panel);
  dock->setFeatures(QDockWidget::DockWidgetMovable);
  addDockWidget(Qt::LeftDockWidgetArea, dock);
  translator_.bindText(dock, { "Arbre", "Tree" });
  relabelTree();
}

QWidget* TPPWidget::buildSurfaceButtons()
{
  auto* edit = new QPushButton(this);
  translator_.bindText(edit, { "Reprendre", "Reselect" });
  translator_.bindToolTip(edit, { "Rend les faces de la surface à la sélection en cours, pour la retoucher ; son tracé est perdu.",
                                  "Gives the faces of the surface back to the current selection to retouch it; its trace is lost." });
  connect(edit, &QPushButton::clicked, this, [this](bool) { onEditSurface(); });
  auto* remove = new QPushButton(this);
  translator_.bindText(remove, { "Supprimer", "Remove" });
  translator_.bindToolTip(remove, { "Supprime la surface et son tracé.", "Removes the surface and its trace." });
  connect(remove, &QPushButton::clicked, this, [this](bool) { onRemoveSurface(); });
  auto* row = new QWidget(this);
  auto* layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(edit);
  layout->addWidget(remove);
  return row;
}

void TPPWidget::relabelTree()
{
  // Le texte d'un element qui change emet itemChanged : bloque, il ne doit rien declencher
  const QSignalBlocker blocker(tree_);
  piece_item_->setText(0, mesh_file_.empty() ? say("Aucune pièce", "No part") :
                                               QFileInfo(QString::fromStdString(mesh_file_)).completeBaseName());
  surfaces_item_->setText(0, say("Surfaces (%1)", "Surfaces (%1)").arg(surfaces_item_->childCount()));
  for (int index = 0; index < surfaces_item_->childCount(); ++index)
  {
    relabelSurface(surfaces_item_->child(index));
  }
  refreshPassItems();
}

QTreeWidgetItem* TPPWidget::currentSurfaceItem() const
{
  QTreeWidgetItem* item = tree_ != nullptr ? tree_->currentItem() : nullptr;
  // L'element du trace, sous une surface, designe aussi cette surface
  if (item != nullptr && item->parent() != nullptr && item->parent()->parent() == surfaces_item_)
  {
    item = item->parent();
  }
  return (item != nullptr && item->parent() == surfaces_item_) ? item : nullptr;
}

int TPPWidget::selectedSurfaceId() const
{
  QTreeWidgetItem* item = currentSurfaceItem();
  return item != nullptr ? item->data(0, kSurfaceIdRole).toInt() : -1;
}

void TPPWidget::relabelSurface(QTreeWidgetItem* item)
{
  const QSignalBlocker blocker(tree_);
  const int surface_id = item->data(0, kSurfaceIdRole).toInt();
  const int face_count = item->data(0, kFaceCountRole).toInt();
  item->setText(0, say("Surface %1 · %2 faces", "Surface %1 · %2 faces").arg(surface_id + 1).arg(face_count));
  // Le trace est l'unique enfant de sa surface
  if (item->childCount() == 0)
  {
    new QTreeWidgetItem(item);
    item->setExpanded(true);
  }
  const auto found = surface_traces_.find(surface_id);
  const std::size_t vertex_count = found == surface_traces_.end() ? 0 : found->second.size();
  item->child(0)->setText(0, vertex_count == 0 ? say("sans tracé", "no trace") :
                                                 say("Tracé, %1 sommets", "Trace, %1 vertices").arg(vertex_count));
}

void TPPWidget::onAddSurface()
{
  const std::vector<std::vector<int>> components = pendingComponents();
  if (!checkPendingIsOneSurface(components.size()))
  {
    return;
  }
  const int surface_id = next_region_id_++;
  assignFacesToRegion(face_region_, components.front(), surface_id);
  std::fill(pending_faces_.begin(), pending_faces_.end(), 0);

  auto* item = new QTreeWidgetItem();
  item->setData(0, kSurfaceIdRole, surface_id);
  item->setData(0, kFaceCountRole, static_cast<int>(components.front().size()));
  // La case de la surface dit si elle est dessinee
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
  item->setCheckState(0, Qt::Checked);
  surfaces_item_->addChild(item);
  relabelTree();
  tree_->setCurrentItem(item);
  updateSelectionColors();
  render();
}

void TPPWidget::onRemoveSurface()
{
  QTreeWidgetItem* item = currentSurfaceItem();
  if (item != nullptr)
  {
    removeSurface(item, false);
  }
}

void TPPWidget::onEditSurface()
{
  QTreeWidgetItem* item = currentSurfaceItem();
  if (item != nullptr)
  {
    removeSurface(item, true);
  }
}

void TPPWidget::removeSurface(QTreeWidgetItem* item, const bool back_to_selection)
{
  const int surface_id = item->data(0, kSurfaceIdRole).toInt();
  for (std::size_t f = 0; f < face_region_.size(); ++f)
  {
    if (face_region_[f] == surface_id)
    {
      face_region_[f] = -1;
      // Reprendre une surface la rend a la selection en cours, pour la retoucher puis la revalider
      pending_faces_[f] = back_to_selection ? 1 : pending_faces_[f];
    }
  }
  surface_traces_.erase(surface_id);
  hidden_surfaces_.erase(surface_id);
  trace_preview_active_ = false;
  delete item;
  relabelTree();
  updateTraceMarkers();
  updateSelectionColors();
  render();
}

void TPPWidget::clearSurfaceItems()
{
  const QSignalBlocker blocker(tree_);
  qDeleteAll(surfaces_item_->takeChildren());
  hidden_surfaces_.clear();
  relabelTree();
}

void TPPWidget::onTreeItemChanged(QTreeWidgetItem* item)
{
  if (item->parent() != surfaces_item_)
  {
    return;
  }
  const int surface_id = item->data(0, kSurfaceIdRole).toInt();
  if (item->checkState(0) == Qt::Checked)
  {
    hidden_surfaces_.erase(surface_id);
  }
  else
  {
    hidden_surfaces_.insert(surface_id);
  }
  updateSelectionColors();
  render();
}

void TPPWidget::onTreeCurrentChanged()
{
  QTreeWidgetItem* item = tree_->currentItem();
  // Une passe choisie dans l'arbre est aussi choisie dans le dock des passes, qui la surligne
  if (item != nullptr && item->parent() == passes_item_ && path_edit_ != nullptr)
  {
    path_edit_->selectPass(item->data(0, kPassRowRole).toInt());
  }
  updateTraceMarkers();
  render();
}

void TPPWidget::refreshPassItems()
{
  if (passes_item_ == nullptr || path_edit_ == nullptr)
  {
    return;
  }
  const QSignalBlocker blocker(tree_);
  qDeleteAll(passes_item_->takeChildren());
  const ToolPath& passes = path_edit_->passes();
  for (std::size_t row = 0; row < passes.size(); ++row)
  {
    auto* item = new QTreeWidgetItem(passes_item_);
    item->setData(0, kPassRowRole, static_cast<int>(row));
    item->setText(0, say("Passe %1 · %2 poses", "Pass %1 · %2 poses").arg(row + 1).arg(passes[row].size()));
  }
  const bool single = (passes.size() == 1);
  passes_item_->setText(0, single ? say("Trajectoire (1 passe)", "Tool path (1 pass)") :
                                    say("Trajectoire (%1 passes)", "Tool path (%1 passes)").arg(passes.size()));
  passes_item_->setExpanded(true);
  // La reconstruction a retire l'element courant : la passe choisie dans le panneau le redevient
  selectPassItem(path_edit_->selectedPass());
}

void TPPWidget::selectPassItem(const int row)
{
  if (passes_item_ == nullptr || row < 0 || row >= passes_item_->childCount())
  {
    return;
  }
  const QSignalBlocker blocker(tree_);
  tree_->setCurrentItem(passes_item_->child(row));
}

}  // namespace noether
