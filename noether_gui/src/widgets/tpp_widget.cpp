#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>
#include <noether_gui/widgets/path_edit_widget.h>
#include <noether_gui/widgets/trace_tool.h>
#include <Eigen/Geometry>
#include <noether_tpp/core/tool_path_recipe.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <pcl/io/vtk_lib_io.h>
#include <QColorDialog>
#include <QFileDialog>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTextStream>
#include <yaml-cpp/yaml.h>

// Rendering includes
#include <QVTKOpenGLNativeWidget.h>
#include <vtkAxesActor.h>
#include <vtkPropAssembly.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLActor.h>
#include <vtkOpenGLRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkProp3DCollection.h>
#include <vtkSTLReader.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxes.h>
#include <vtkTextActor.h>
#include <vtkTransformFilter.h>
#include <vtkTransform.h>
#include <vtkTubeFilter.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkColorSeries.h>
#include <vtkLeaderActor2D.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkActor.h>
#include <vtkPropCollection.h>
#include <vtkActor2D.h>
#include <vtkCamera.h>
#include <vtkCellLocator.h>
#include <vtkCellPicker.h>
#include <vtkCoordinate.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkPolyLine.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkIdList.h>
#include <vtkHardwareSelector.h>
#include <vtkIdTypeArray.h>
#include <vtkInformation.h>
#include <vtkNew.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkColorSeries.h>

#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QToolBar>
#include <QVBoxLayout>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <utility>

namespace
{
/** @brief Screen spacing between two brush samples when filling a drag segment, in pixels */
constexpr double kBrushSampleSpacingPx = 4.0;
/** @brief Ceiling on the samples one move event may paint, so a fast flick cannot stall the view */
constexpr int kMaxBrushSamplesPerMove = 24;
/** @brief Brush diameter used when the toolbar spin box does not exist yet, in millimetres */
constexpr double kDefaultBrushDiameterMm = 30.0;
/** @brief Radius around the first vertex, in pixels, within which a click closes the outline */
constexpr double kOutlineCloseTolerancePx = 8.0;
/** @brief Axis-aligned bounds of a 2D outline, as (lower, upper) */
/**
 * @brief Fewest waypoints a delivered pass may carry.
 * @details One approach waypoint, one departure waypoint, and at least two in contact, as
 * z404_sdg_planning splits a pass (kApproachPoses, kRetractPoses and kMinPoses in its
 * src/tool_path.cpp). A shorter pass is refused there, in front of the workpiece; refusing it here
 * is the same rule, applied where it can still be fixed.
 */
const std::size_t kMinPassPoses = 4;

/**
 * @brief Poses hors contact a chaque bout d'une passe livree : une d'approche, une de retrait.
 * @details Les memes que kMinPassPoses compte ; elles ne poncent rien et ne couvrent donc rien.
 */
const std::size_t kSkirtPoses = 1;

/** @brief Rayon d'outil a l'ouverture, en millimetres : le disque de 125 mm de la cellule */
const double kDefaultToolRadiusMm = 62.5;

/** @brief Pas du trace manuel a l'ouverture, en millimetres */
const double kDefaultTraceStepMm = 20.0;

/** @brief Hauteur d'approche et de retrait a l'ouverture, en millimetres */
const double kDefaultApproachHeightMm = 50.0;

/**
 * @brief Couleurs des points du trace manuel : courant, depart, arrivee.
 * @details Le cyan n'est pas dans la palette des surfaces (Brewer Set1) : un point se distingue
 * de toute surface, quelle que soit sa couleur.
 */
const unsigned char kTraceRgb[3] = { 0, 235, 255 };
const unsigned char kStartRgb[3] = { 40, 230, 60 };
const unsigned char kEndRgb[3] = { 255, 40, 40 };
/** @brief Couleur des cercles du disque autour des points du chemin, hors palette des surfaces */
const unsigned char kDiscRgb[3] = { 0, 235, 255 };
/** @brief Segments d'un cercle du disque */
const int kCircleSegments = 48;
/** @brief Couleur de la pose saisie pour deplacement */
const unsigned char kGrabRgb[3] = { 255, 60, 220 };
/** @brief Distance, en metres, au-dela de laquelle un clic ne saisit aucune pose */
const double kGrabToleranceM = 0.05;

/**
 * @brief Largest position difference still counted as the same generated pass, in metres.
 * @details Generation is deterministic, so the same inputs give back the very same numbers. This
 * only absorbs the rounding of the recipe file, which keeps twelve significant digits.
 */
const double kFingerprintTolerance = 1e-9;

std::pair<Eigen::Vector2d, Eigen::Vector2d> outlineBounds(const std::vector<Eigen::Vector2d>& outline)
{
  Eigen::Vector2d lower = outline.front();
  Eigen::Vector2d upper = outline.front();
  for (const Eigen::Vector2d& point : outline)
  {
    lower = lower.cwiseMin(point);
    upper = upper.cwiseMax(point);
  }
  return { lower, upper };
}

/**
 * @brief Interactor style that shares the left mouse button between the camera and the selection
 * tools.
 * @details With no drag tool active the behavior is that of the trackball camera, plus reporting of
 * clicks that did not drag (@ref on_click) so a click can pick a region. With a drag tool active
 * (brush, eraser) the left button feeds @ref on_drag_start / @ref on_drag_move / @ref on_drag_end
 * instead of orbiting, unless Alt is held: the orbit then stays reachable without leaving the tool.
 * Holding Shift marks the drag or the click as an erase, which is reported back to the tool.
 * Key presses are reported through @ref on_key for the tools driven by clicks (polygon).
 *
 * The modifier keys are read once, when the button goes down. Releasing Shift halfway through a
 * stroke does not turn its remainder into a selection, which is how paint tools behave elsewhere
 * and what keeps a stroke from ending up half painted and half erased.
 */
class SelectionInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static SelectionInteractorStyle* New();
  vtkTypeMacro(SelectionInteractorStyle, vtkInteractorStyleTrackballCamera);

  /** @brief Returns true when the active tool consumes left-button drags */
  std::function<bool()> drag_selects;
  /** @brief Called with the display (x, y) of a click that did not drag, whether Shift was held, and
   * whether it was the second click of a double-click */
  std::function<void(int x, int y, bool shift, bool double_click)> on_click;
  /** @brief Called with the key symbol of a key press and whether Shift was held */
  std::function<void(const std::string& key, bool shift)> on_key;
  /** @brief Start of a drag consumed by the tool; `erase` reports whether Shift was held */
  std::function<void(int x, int y, bool erase)> on_drag_start;
  /** @brief Continuation of a consumed drag */
  std::function<void(int x, int y, bool erase)> on_drag_move;
  /** @brief End of a consumed drag */
  std::function<void(bool erase)> on_drag_end;

  void OnLeftButtonDown() override
  {
    vtkRenderWindowInteractor* interactor = this->GetInteractor();
    const int* pos = interactor->GetEventPosition();
    down_x_ = pos[0];
    down_y_ = pos[1];
    dragged_ = false;
    erasing_ = interactor->GetShiftKey() != 0;
    // Qt reports the second press of a double-click with a repeat count of one
    double_click_ = interactor->GetRepeatCount() > 0;

    // Alt reserves the orbit for the camera even while a drag tool is active.
    painting_ = drag_selects && drag_selects() && interactor->GetAltKey() == 0;
    if (painting_)
    {
      // Return before the base class so the camera never enters its rotate state.
      if (on_drag_start)
      {
        on_drag_start(pos[0], pos[1], erasing_);
      }
      return;
    }

    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

  void OnMouseMove() override
  {
    const int* pos = this->GetInteractor()->GetEventPosition();
    if (std::abs(pos[0] - down_x_) > kDragThreshold || std::abs(pos[1] - down_y_) > kDragThreshold)
    {
      dragged_ = true;
    }

    if (painting_)
    {
      if (on_drag_move)
      {
        on_drag_move(pos[0], pos[1], erasing_);
      }
      return;
    }

    vtkInteractorStyleTrackballCamera::OnMouseMove();
  }

  void OnLeftButtonUp() override
  {
    if (painting_)
    {
      painting_ = false;
      if (on_drag_end)
      {
        on_drag_end(erasing_);
      }
      return;
    }

    if (!dragged_ && on_click)
    {
      const int* pos = this->GetInteractor()->GetEventPosition();
      on_click(pos[0], pos[1], erasing_, double_click_);
    }

    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
  }

  void OnRightButtonDown() override
  {
    // The base class dollies with the right button, but the wheel already zooms. Rotating
    // instead keeps the camera reachable while a drag tool owns the left button.
    const int* pos = this->GetInteractor()->GetEventPosition();
    this->FindPokedRenderer(pos[0], pos[1]);
    if (this->CurrentRenderer != nullptr)
    {
      this->StartRotate();
    }
  }

  void OnRightButtonUp() override
  {
    this->EndRotate();
  }

  void OnKeyPress() override
  {
    if (on_key)
    {
      vtkRenderWindowInteractor* interactor = this->GetInteractor();
      const char* symbol = interactor->GetKeySym();
      on_key(symbol != nullptr ? std::string(symbol) : std::string(), interactor->GetShiftKey() != 0);
    }
    vtkInteractorStyleTrackballCamera::OnKeyPress();
  }

private:
  /** @brief Pixels of motion above which a press counts as a drag rather than a click */
  static constexpr int kDragThreshold = 3;
  /** @brief Display coordinates of the last left-button press */
  int down_x_ = 0;
  int down_y_ = 0;
  /** @brief Whether the current press has moved far enough to count as a drag */
  bool dragged_ = false;
  /** @brief Whether the current press is being consumed by a selection tool */
  bool painting_ = false;
  /** @brief Whether Shift was held when the current press started (erase instead of select) */
  bool erasing_ = false;
  /** @brief Whether the current press is the second one of a double-click */
  bool double_click_ = false;
};
vtkStandardNewMacro(SelectionInteractorStyle);

}  // namespace

namespace noether
{
/// @brief Remplace un acteur dans le rendu par un nouveau construit sur `poly` ; defini plus bas
vtkSmartPointer<vtkActor> replaceActor(vtkRenderer& renderer, vtkSmartPointer<vtkActor> previous, vtkPolyData* poly);
/// @brief Ajoute un cercle de rayon donne dans le plan (x, y) d'une pose ; defini plus bas
void appendDiscCircle(const Eigen::Isometry3d& pose, double radius, vtkPoints& points, vtkCellArray& lines);
/// @brief Acteurs des points de passage, un par pose ; defini plus bas
vtkSmartPointer<vtkPropAssembly> createToolPathActors(const std::vector<ToolPaths>& tool_paths,
                                                      vtkAlgorithmOutput* waypoint_shape_output_port);
/// @brief Acteur des lignes reliant les points de passage ; defini plus bas
vtkSmartPointer<vtkPropAssembly> createToolPathPolylineActor(const std::vector<ToolPaths>& tool_paths,
                                                             vtkAlgorithmOutput* waypoint_shape_output_port);
/// @brief Acteurs des fragments de maillage planifies ; defini plus bas
vtkSmartPointer<vtkPropAssembly> createMeshActors(const std::vector<pcl::PolygonMesh>& meshes);

TPPWidget::TPPWidget(std::shared_ptr<const WidgetFactory> factory, QWidget* parent)
  : QMainWindow(parent)
  , ui_(new Ui::TPP())
  , pipeline_widget_(new ConfigurableTPPPipelineWidget(factory, "", this))
  , render_widget_(new QVTKOpenGLNativeWidget(this))
  , renderer_(vtkSmartPointer<vtkOpenGLRenderer>::New())
  , mesh_mapper_(vtkSmartPointer<vtkOpenGLPolyDataMapper>::New())
  , mesh_actor_(vtkSmartPointer<vtkOpenGLActor>::New())
  , mesh_fragment_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , tool_path_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , connected_path_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , unmodified_tool_path_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , unmodified_connected_path_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , highlight_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , axes_(vtkSmartPointer<vtkAxes>::New())
  , axes_actor_(vtkSmartPointer<vtkAxesActor>::New())
  , tube_filter_(vtkSmartPointer<vtkTubeFilter>::New())
  , outline_(kOutlineCloseTolerancePx)
{
  ui_->setupUi(this);

  // Replace the pipeline widget into the dock widget
  overwriteWidget(ui_->verticalLayout, ui_->widget, pipeline_widget_);

  // Set the central widget to the render widget
  setCentralWidget(render_widget_);

  // Set the background color of the dock widget title to the mid-light palette color
  ui_->dock->setStyleSheet(
      QString("QDockWidget::title { background-color: %1; }").arg(palette().color(QPalette::Midlight).name()));

  // Set up the VTK objects
  renderer_->SetBackground(0.2, 0.2, 0.2);

  // Original mesh mapper/actor. This is the selection canvas: the mesh is drawn white and per-face
  // colors carry the region-selection highlight, so it must be pickable and use cell-data scalars.
  mesh_actor_->SetMapper(mesh_mapper_);
  mesh_actor_->SetPickable(true);
  mesh_mapper_->SetScalarModeToUseCellData();
  mesh_mapper_->SetColorModeToDirectScalars();
  renderer_->AddActor(mesh_actor_);

  // Mesh fragment mapper/actor
  renderer_->AddActor(mesh_fragment_actor_);

  // Outline marking the pass selected in the Trajectory dock
  renderer_->AddActor(highlight_actor_);

  // Tool path axis display object
  axes_->SetScaleFactor(ui_->double_spin_box_axis_size->value());
  tube_filter_->SetInputConnection(axes_->GetOutputPort());
  tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
  tube_filter_->SetNumberOfSides(10);
  tube_filter_->CappingOn();

  // Zero ref frame axis display
  {
    axes_actor_->SetTotalLength(ui_->double_spin_box_origin_size->value(),
                                ui_->double_spin_box_origin_size->value(),
                                ui_->double_spin_box_origin_size->value());

    axes_actor_->SetXAxisLabelText("X");
    axes_actor_->SetYAxisLabelText("Y");
    axes_actor_->SetZAxisLabelText("Z");

    // Set the scale mode to None such that the font size controls the size of the text
    axes_actor_->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
    axes_actor_->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
    axes_actor_->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();

    // Add the actor
    renderer_->AddActor(axes_actor_);

    showAxes(ui_->check_box_show_axes->isChecked());
  }

  vtkRenderWindow* window = render_widget_->renderWindow();
  window->AddRenderer(renderer_);

  // Interactor style shared between the camera and the selection tools: a plain click picks a
  // region or places an outline vertex, a drag paints when the brush or the eraser is active.
  cell_picker_ = vtkSmartPointer<vtkCellPicker>::New();
  cell_picker_->SetTolerance(0.0005);
  auto interactor_style = vtkSmartPointer<SelectionInteractorStyle>::New();
  interactor_style->on_click = [this](int x, int y, bool shift, bool double_click) {
    if (tool_ == SelectionTool::Polygon)
    {
      onOutlineClick(x, y, shift, double_click);
      return;
    }
    const int face = pickFace(x, y);
    if (face >= 0)
    {
      onCellClicked(face);
    }
  };
  interactor_style->on_key = [this](const std::string& key, bool shift) { onKeyPressed(key, shift); };
  // The polygon tool works by clicks, so it leaves the drag to the camera
  interactor_style->drag_selects = [this]() {
    return movingPoints() || tracing() || tool_ == SelectionTool::Brush || tool_ == SelectionTool::Eraser;
  };
  interactor_style->on_drag_start = [this](int x, int y, bool erase) { onDragStart(x, y, erase); };
  interactor_style->on_drag_move = [this](int x, int y, bool erase) { onDragMove(x, y, erase); };
  interactor_style->on_drag_end = [this](bool erase) { onDragEnd(erase); };
  render_widget_->interactor()->SetInteractorStyle(interactor_style);
  render_widget_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  // Outline overlay for the polygon tool. Its points live in display coordinates, so the transform
  // coordinate is set explicitly rather than relying on the mapper's default interpretation.
  outline_poly_ = vtkSmartPointer<vtkPolyData>::New();
  outline_mapper_ = vtkSmartPointer<vtkPolyDataMapper2D>::New();
  outline_mapper_->SetInputData(outline_poly_);
  {
    auto display_coordinate = vtkSmartPointer<vtkCoordinate>::New();
    display_coordinate->SetCoordinateSystemToDisplay();
    outline_mapper_->SetTransformCoordinate(display_coordinate);
  }
  outline_actor_ = vtkSmartPointer<vtkActor2D>::New();
  outline_actor_->SetMapper(outline_mapper_);
  outline_actor_->GetProperty()->SetColor(1.0, 1.0, 0.0);
  outline_actor_->GetProperty()->SetLineWidth(2.0);
  outline_actor_->SetVisibility(false);
  renderer_->AddActor2D(outline_actor_);

  // Selection tool selector. The three tools fill the same face_region_ table, so switching is free
  // and leaves what is already selected untouched.
  tool_combo_box_ = new QComboBox(this);
  tool_combo_box_->addItem("Region (click)", static_cast<int>(SelectionTool::Region));
  tool_combo_box_->addItem("Brush (drag)", static_cast<int>(SelectionTool::Brush));
  tool_combo_box_->addItem("Polygon (clicks)", static_cast<int>(SelectionTool::Polygon));
  tool_combo_box_->addItem("Eraser (drag)", static_cast<int>(SelectionTool::Eraser));
  tool_combo_box_->setToolTip(
      "Selection tool. Shift while dragging, or while closing an outline, erases instead of "
      "selecting; Alt while dragging orbits the camera without leaving the brush.\n"
      "Polygon: click to place vertices, then click the first one, press Enter or double-click to "
      "close. Backspace removes the last vertex, Escape cancels the outline.");
  ui_->toolBar->addSeparator();
  ui_->toolBar->addWidget(new QLabel("Tool:", this));
  ui_->toolBar->addWidget(tool_combo_box_);

  // Toolbar control for how "flat" a region must stay while growing (max face-to-face angle)
  flatness_angle_spin_box_ = new QDoubleSpinBox(this);
  flatness_angle_spin_box_->setRange(0.0, 180.0);
  flatness_angle_spin_box_->setSingleStep(1.0);
  flatness_angle_spin_box_->setValue(20.0);
  flatness_angle_spin_box_->setSuffix(" °");
  flatness_angle_spin_box_->setToolTip(
      "Region flatness: maximum angle between adjacent faces for a click-selected region to keep "
      "growing. Region growth stops at edges sharper than this.");
  ui_->toolBar->addWidget(new QLabel("Region flatness:", this));
  ui_->toolBar->addWidget(flatness_angle_spin_box_);

  // Brush diameter, measured along the surface rather than on screen: zooming does not change what
  // a stroke covers, and the value can be set to the actual disc diameter to preview its footprint.
  brush_diameter_spin_box_ = new QDoubleSpinBox(this);
  brush_diameter_spin_box_->setRange(0.1, 10000.0);
  brush_diameter_spin_box_->setDecimals(1);
  brush_diameter_spin_box_->setSingleStep(5.0);
  brush_diameter_spin_box_->setValue(kDefaultBrushDiameterMm);
  brush_diameter_spin_box_->setSuffix(" mm");
  brush_diameter_spin_box_->setToolTip(
      "Brush diameter, measured on the surface. Set it to the disc diameter to see what one pass of "
      "the tool covers.");
  ui_->toolBar->addWidget(new QLabel("Brush diameter:", this));
  ui_->toolBar->addWidget(brush_diameter_spin_box_);

  // Connected after the items were added: addItem() emits currentIndexChanged while inserting the
  // first entry, and the handler reads controls that do not exist yet at that point.
  connect(tool_combo_box_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
    tool_ = static_cast<SelectionTool>(tool_combo_box_->itemData(index).toInt());
    // Choisir un outil rend le tirage a cet outil : le mode « Deplacer des points » se relache,
    // sinon il avalerait chaque tirage en silence
    if (move_points_button_ != nullptr)
    {
      move_points_button_->setChecked(false);
    }
    if (trace_button_ != nullptr)
    {
      trace_button_->setChecked(false);
    }
    // An outline left half traced must not survive a tool change.
    outline_.clear();
    updateOutlineOverlay();
    updateToolControls();
    updateTraceMarkers();
    render();
  });
  updateToolControls();

  // Toolbar control for the tool radius (mm) used to erode the selected regions. A selected strip
  // narrower than the tool diameter is eroded away (the disc would overhang onto its neighbors).
  tool_radius_spin_box_ = new QDoubleSpinBox(this);
  tool_radius_spin_box_->setRange(0.0, 10000.0);
  tool_radius_spin_box_->setDecimals(1);
  tool_radius_spin_box_->setSingleStep(1.0);
  tool_radius_spin_box_->setValue(kDefaultToolRadiusMm);
  tool_radius_spin_box_->setSuffix(" mm");
  tool_radius_spin_box_->setToolTip(
      "Tool radius: the selected regions are eroded by this radius (the disc would overhang onto "
      "neighbors within it). For a Ø125 mm disc, use 62.5.");

  // Check box to enable/disable the tool-radius erosion altogether
  tool_radius_enabled_check_box_ = new QCheckBox("Tool radius:", this);
  tool_radius_enabled_check_box_->setChecked(true);
  tool_radius_enabled_check_box_->setToolTip(
      "Enable/disable eroding the selection by the tool radius. When unchecked, the whole selected "
      "region is planned regardless of the tool radius value.");
  ui_->toolBar->addWidget(tool_radius_enabled_check_box_);
  ui_->toolBar->addWidget(tool_radius_spin_box_);
  connect(tool_radius_spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
    updateDiscCircles();
    updateTraceMarkers();
    render();
  });
  connect(tool_radius_enabled_check_box_, &QCheckBox::toggled, this, [this](bool enabled) {
    tool_radius_spin_box_->setEnabled(enabled);
    updateDiscCircles();
    updateTraceMarkers();
    render();
  });


  // Surfaces are validated one at a time: Add turns the work selection into a named list entry,
  // and only the list is planned. The dock keeps the entries visible and individually removable.
  add_surface_button_ = new QPushButton("Add surface", this);
  add_surface_button_->setToolTip("Validates the current (orange) selection as one surface.\n"
                                  "Refused when the selection is made of several disconnected pieces.");
  ui_->toolBar->addSeparator();
  ui_->toolBar->addWidget(add_surface_button_);
  connect(add_surface_button_, &QPushButton::clicked, this, &TPPWidget::onAddSurface);

  surface_list_ = new QListWidget(this);
  remove_surface_button_ = new QPushButton("Remove selected", this);
  connect(remove_surface_button_, &QPushButton::clicked, this, &TPPWidget::onRemoveSurface);

  connect(surface_list_, &QListWidget::currentRowChanged, this, [this](int) {
    updateTraceMarkers();
    render();
  });

  auto* surfaces_panel = new QWidget(this);
  auto* surfaces_layout = new QVBoxLayout(surfaces_panel);
  surfaces_layout->addWidget(surface_list_);
  surfaces_layout->addWidget(remove_surface_button_);

  auto* surfaces_dock = new QDockWidget("Surfaces", this);
  surfaces_dock->setWidget(surfaces_panel);
  addDockWidget(Qt::RightDockWidgetArea, surfaces_dock);

  buildTreatmentDock();
  buildPathEditDock();
  // La configuration du pipeline ne sert qu'a l'approche et au retrait : son dock reste cache,
  // une configuration se charge par le menu
  ui_->dock->hide();

  // Set visibility of the actors based on the default state of the check boxes
  mesh_actor_->SetVisibility(ui_->action_show_unmodified_mesh->isChecked());
  mesh_fragment_actor_->SetVisibility(ui_->action_show_modified_mesh->isChecked());
  unmodified_tool_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path->isChecked());
  tool_path_actor_->SetVisibility(ui_->action_show_modified_tool_path->isChecked());
  unmodified_connected_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path_lines->isChecked());
  connected_path_actor_->SetVisibility(ui_->action_show_modified_tool_path_lines->isChecked());

  // Connect signals
  connect(ui_->action_load_mesh, &QAction::triggered, this, &TPPWidget::onLoadMesh);
  connect(ui_->action_clear_selection, &QAction::triggered, this, &TPPWidget::onClearSelection);
  connect(ui_->action_execute_pipeline, &QAction::triggered, [this](const bool) { plan(); });
  connect(ui_->action_save_modified_mesh, &QAction::triggered, this, &TPPWidget::onSaveModifiedMeshes);
  connect(ui_->action_save_toolpath, &QAction::triggered, this, &TPPWidget::onSaveToolPaths);
  connect(ui_->action_show_unmodified_mesh, &QAction::triggered, this, &TPPWidget::showOriginalMesh);
  connect(ui_->action_show_modified_mesh, &QAction::triggered, this, &TPPWidget::showModifiedMesh);
  connect(ui_->action_show_unmodified_tool_path, &QAction::triggered, this, &TPPWidget::showUnmodifiedToolPath);
  connect(ui_->action_show_modified_tool_path, &QAction::triggered, this, &TPPWidget::showModifiedToolPath);
  connect(
      ui_->action_show_unmodified_tool_path_lines, &QAction::triggered, this, &TPPWidget::showUnmodifiedConnectedPath);
  connect(ui_->action_show_modified_tool_path_lines, &QAction::triggered, this, &TPPWidget::showModifiedConnectedPath);
  connect(ui_->check_box_show_axes, &QCheckBox::toggled, this, &TPPWidget::showAxes);
  connect(ui_->action_load_config,
          &QAction::triggered,
          pipeline_widget_,
          &ConfigurableTPPPipelineWidget::onLoadConfiguration);
  connect(ui_->action_save_config,
          &QAction::triggered,
          pipeline_widget_,
          &ConfigurableTPPPipelineWidget::onSaveConfiguration);
  connect(ui_->double_spin_box_axis_size, &QDoubleSpinBox::editingFinished, this, [this]() {
    axes_->SetScaleFactor(ui_->double_spin_box_axis_size->value());
    tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
    render();
  });
  connect(ui_->double_spin_box_origin_size, &QDoubleSpinBox::editingFinished, this, [this]() {
    axes_actor_->SetTotalLength(ui_->double_spin_box_origin_size->value(),
                                ui_->double_spin_box_origin_size->value(),
                                ui_->double_spin_box_origin_size->value());
    render();
  });
}
void TPPWidget::render()
{
  // Call render twice
  render_widget_->renderWindow()->Render();
  render_widget_->renderWindow()->Render();
}

void TPPWidget::showOriginalMesh(const bool checked)
{
  mesh_actor_->SetVisibility(checked);
  // The original and segmented meshes share the same geometry, so showing both just z-fights.
  // Make the two views mutually exclusive: showing the raw mesh hides the colored regions.
  if (checked)
  {
    mesh_fragment_actor_->SetVisibility(false);
    ui_->action_show_modified_mesh->setChecked(false);
  }
  render();
}

void TPPWidget::showModifiedMesh(const bool checked)
{
  mesh_fragment_actor_->SetVisibility(checked);
  // Mutually exclusive with the raw mesh view (see showOriginalMesh)
  if (checked)
  {
    mesh_actor_->SetVisibility(false);
    ui_->action_show_unmodified_mesh->setChecked(false);
  }
  render();
}

void TPPWidget::showUnmodifiedConnectedPath(const bool checked)
{
  unmodified_connected_path_actor_->SetVisibility(checked);
  render();
}

void TPPWidget::showUnmodifiedToolPath(const bool checked)
{
  unmodified_tool_path_actor_->SetVisibility(checked);
  render();
}

void TPPWidget::showModifiedConnectedPath(const bool checked)
{
  connected_path_actor_->SetVisibility(checked);
  render();
}

void TPPWidget::showModifiedToolPath(const bool checked)
{
  tool_path_actor_->SetVisibility(checked);
  render();
}

void TPPWidget::showAxes(const bool checked)
{
  axes_actor_->SetVisibility(checked);
  render();
}

void TPPWidget::setMeshFile(const QString& file)
{
  if (!file.endsWith(".ply") && !file.endsWith(".stl"))
    return;

  mesh_file_ = file.toStdString();

  // Load the mesh with PCL so the same face ordering is used for display, picking, and planning
  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFile(mesh_file_, mesh) < 1)
  {
    QMessageBox::warning(this, "Error", "Failed to load mesh from file");
    return;
  }
  mesh_ = mesh;

  // Un autre maillage perime la planification : la garder afficherait, et laisserait
  // enregistrer, un chemin qui n'appartient plus a la piece a l'ecran.
  clearPlannedPaths();

  // Build the VTK poly data, adjacency, colors, and reset the selection
  buildSelectionData();

  // Zoom out to the extents
  renderer_->ResetCamera();

  // Render
  render();
}

void TPPWidget::buildSelectionData()
{
  // VTK representation for display and picking; mesh2vtk preserves face order so cell i == polygon i
  selection_poly_ = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh_, selection_poly_);
  selection_poly_->BuildLinks();

  const std::size_t num_faces = mesh_.polygons.size();

  // Per-cell RGB color array carrying the selection highlight (white = unselected)
  cell_colors_ = vtkSmartPointer<vtkUnsignedCharArray>::New();
  cell_colors_->SetNumberOfComponents(3);
  cell_colors_->SetName("region_colors");
  cell_colors_->SetNumberOfTuples(static_cast<vtkIdType>(num_faces));
  selection_poly_->GetCellData()->SetScalars(cell_colors_);

  // Face graph and per-face geometry, computed by noether_tpp so they can be unit tested without
  // a window. Every selection tool works on these two tables and nothing else.
  face_adjacency_ = buildFaceAdjacency(mesh_);
  const FaceGeometry geometry = computeFaceGeometry(mesh_);
  face_normals_ = geometry.normals;
  face_centroids_ = geometry.centroids;

  // Reset the selection
  face_region_.assign(num_faces, -1);
  pending_faces_.assign(num_faces, 0);
  surface_traces_.clear();
  trace_preview_active_ = false;
  next_region_id_ = 0;
  fragments_.clear();
  updateSelectionColors();

  // Display the selection poly data
  mesh_mapper_->SetInputData(selection_poly_);
}

std::vector<int> TPPWidget::growRegion(int seed_face) const
{
  std::vector<int> region;
  if (seed_face < 0 || seed_face >= static_cast<int>(face_adjacency_.size()))
    return region;

  const double max_angle_rad = flatness_angle_spin_box_->value() * M_PI / 180.0;
  const double min_dot = std::cos(max_angle_rad);

  // Simple region growing: flood-fill across edge-adjacent faces while the angle between a face and
  // its neighbor stays within the flatness threshold. The region follows a gently curving surface
  // and stops at edges sharper than the threshold.
  std::vector<char> visited(face_adjacency_.size(), 0);
  std::queue<int> to_visit;
  to_visit.push(seed_face);
  visited[static_cast<std::size_t>(seed_face)] = 1;

  while (!to_visit.empty())
  {
    const int f = to_visit.front();
    to_visit.pop();
    region.push_back(f);

    const Eigen::Vector3d& nf = face_normals_[static_cast<std::size_t>(f)];
    for (int neighbor : face_adjacency_[static_cast<std::size_t>(f)])
    {
      // Faces owned by an added surface are not up for grabs: growth flows around them.
      if (visited[static_cast<std::size_t>(neighbor)] || face_region_[static_cast<std::size_t>(neighbor)] >= 0)
        continue;

      const Eigen::Vector3d& nn = face_normals_[static_cast<std::size_t>(neighbor)];
      const double dot = nf.dot(nn);
      if (dot >= min_dot)
      {
        visited[static_cast<std::size_t>(neighbor)] = 1;
        to_visit.push(neighbor);
      }
    }
  }
  return region;
}

std::vector<char> TPPWidget::erodeSelection() const
{
  const std::size_t n = face_region_.size();
  std::vector<char> kept(n, 0);
  if (n == 0)
    return kept;

  const bool erosion_enabled = tool_radius_enabled_check_box_ && tool_radius_enabled_check_box_->isChecked();
  const double radius_m = tool_radius_spin_box_->value() / 1000.0;  // mm -> m
  if (!erosion_enabled || radius_m <= 0.0)
  {
    // Erosion disabled (or radius 0): every selected face is kept
    for (std::size_t f = 0; f < n; ++f)
      kept[f] = (face_region_[f] >= 0) ? 1 : 0;
    return kept;
  }

  // Multi-source Dijkstra over the face graph: distance of each selected face to the boundary of
  // its own region, following face centroids. Growth stays within a single region.
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> dist(n, inf);
  using Node = std::pair<double, int>;  // (distance, face)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  for (std::size_t f = 0; f < n; ++f)
  {
    const int r = face_region_[f];
    if (r < 0)
      continue;
    // Only erode next to an actual obstacle: an edge-neighbor face outside this region that RISES
    // above the region surface (a relief sticking up, which the disc would hit). Neighbors that
    // step down (side walls, recesses) do not obstruct a disc resting on top, and free/open edges
    // have no neighbor at all -- neither is an erosion source.
    const Eigen::Vector3d& cf = face_centroids_[f];
    const Eigen::Vector3d& nf = face_normals_[f];
    bool obstacle_boundary = false;
    for (int nb : face_adjacency_[f])
    {
      if (face_region_[static_cast<std::size_t>(nb)] == r)
        continue;  // same region: not a boundary
      const Eigen::Vector3d& cg = face_centroids_[static_cast<std::size_t>(nb)];
      const double rise = (cg - cf).dot(nf);
      if (rise > 1e-6)  // neighbor is above the surface -> a rising relief -> obstacle
      {
        obstacle_boundary = true;
        break;
      }
    }
    if (obstacle_boundary)
    {
      dist[f] = 0.0;
      pq.push(Node(0.0, static_cast<int>(f)));
    }
  }

  while (!pq.empty())
  {
    const Node top = pq.top();
    pq.pop();
    const int u = top.second;
    if (top.first > dist[static_cast<std::size_t>(u)])
      continue;
    const int r = face_region_[static_cast<std::size_t>(u)];
    const Eigen::Vector3d& cu = face_centroids_[static_cast<std::size_t>(u)];
    for (int v : face_adjacency_[static_cast<std::size_t>(u)])
    {
      if (face_region_[static_cast<std::size_t>(v)] != r)
        continue;  // stay within the region
      const Eigen::Vector3d& cv = face_centroids_[static_cast<std::size_t>(v)];
      const double nd = dist[static_cast<std::size_t>(u)] + (cu - cv).norm();
      if (nd < dist[static_cast<std::size_t>(v)])
      {
        dist[static_cast<std::size_t>(v)] = nd;
        pq.push(Node(nd, v));
      }
    }
  }

  for (std::size_t f = 0; f < n; ++f)
  {
    kept[f] = (face_region_[f] >= 0 && dist[f] >= radius_m) ? 1 : 0;
  }
  return kept;
}
void TPPWidget::updateSelectionColors()
{
  if (!cell_colors_)
    return;

  auto color_series = vtkSmartPointer<vtkColorSeries>::New();
  color_series->SetColorScheme(vtkColorSeries::ColorSchemes::BREWER_QUALITATIVE_SET1);

  const std::vector<char> kept = erodeSelection();

  const unsigned char white[3] = { 220, 220, 220 };
  const unsigned char pending_rgb[3] = { 245, 166, 35 };  // work selection: not yet a surface
  for (vtkIdType i = 0; i < cell_colors_->GetNumberOfTuples(); ++i)
  {
    const std::size_t f = static_cast<std::size_t>(i);
    const int region = (f < face_region_.size()) ? face_region_[f] : -1;
    if (f < pending_faces_.size() && pending_faces_[f])
    {
      cell_colors_->SetTypedTuple(i, pending_rgb);
    }
    else if (region < 0)
    {
      cell_colors_->SetTypedTuple(i, white);
    }
    else
    {
      const vtkColor3ub c = color_series->GetColorRepeating(region);
      if (f < kept.size() && kept[f])
      {
        // Kept by erosion: full, vivid region color
        const unsigned char rgb[3] = { c.GetRed(), c.GetGreen(), c.GetBlue() };
        cell_colors_->SetTypedTuple(i, rgb);
      }
      else
      {
        // Selected but eroded away by the tool radius: a pale tint (not planned)
        const unsigned char rgb[3] = { static_cast<unsigned char>((c.GetRed() + 3 * 220) / 4),
                                       static_cast<unsigned char>((c.GetGreen() + 3 * 220) / 4),
                                       static_cast<unsigned char>((c.GetBlue() + 3 * 220) / 4) };
        cell_colors_->SetTypedTuple(i, rgb);
      }
    }
  }
  cell_colors_->Modified();
  if (selection_poly_)
    selection_poly_->Modified();
}

void TPPWidget::onCellClicked(int cell_id)
{
  if (cell_id < 0 || cell_id >= static_cast<int>(pending_faces_.size()))
    return;

  const std::size_t face = static_cast<std::size_t>(cell_id);
  if (face_region_[face] >= 0)
    return;  // faces owned by an added surface are managed through the surface list

  if (pending_faces_[face])
  {
    // Clicking the work selection removes the connected piece under the cursor
    for (const std::vector<int>& component : pendingComponents())
    {
      if (std::find(component.begin(), component.end(), cell_id) != component.end())
      {
        for (int f : component)
          pending_faces_[static_cast<std::size_t>(f)] = 0;
        break;
      }
    }
  }
  else
  {
    for (int f : growRegion(cell_id))
      pending_faces_[static_cast<std::size_t>(f)] = 1;
  }

  updateSelectionColors();
  render();
}

void TPPWidget::paintPending(const std::vector<int>& faces, const bool erase)
{
  for (int f : faces)
  {
    const std::size_t i = static_cast<std::size_t>(f);
    if (erase)
      pending_faces_[i] = 0;
    else if (face_region_[i] < 0)  // faces owned by an added surface stay with it
      pending_faces_[i] = 1;
  }
}

std::vector<std::vector<int>> TPPWidget::pendingComponents() const
{
  // Plain flood fill over the face graph, restricted to the work selection. The component count is
  // what Add checks: one component is a surface, several are a refused selection.
  std::vector<std::vector<int>> components;
  std::vector<char> visited(pending_faces_.size(), 0);
  for (std::size_t seed = 0; seed < pending_faces_.size(); ++seed)
  {
    if (!pending_faces_[seed] || visited[seed])
      continue;

    std::vector<int> component;
    std::queue<int> to_visit;
    to_visit.push(static_cast<int>(seed));
    visited[seed] = 1;
    while (!to_visit.empty())
    {
      const int f = to_visit.front();
      to_visit.pop();
      component.push_back(f);
      for (int neighbor : face_adjacency_[static_cast<std::size_t>(f)])
      {
        const std::size_t n = static_cast<std::size_t>(neighbor);
        if (pending_faces_[n] && !visited[n])
        {
          visited[n] = 1;
          to_visit.push(neighbor);
        }
      }
    }
    components.push_back(std::move(component));
  }
  return components;
}

int TPPWidget::pickFace(const int x, const int y) const
{
  cell_picker_->Pick(x, y, 0, renderer_);

  // Only a hit on the selection mesh counts. The tool paths, the axes and the fragment actors share
  // the scene, and a ray landing on any of them would otherwise be reported as a face.
  if (cell_picker_->GetActor() != mesh_actor_.Get())
  {
    return -1;
  }

  const vtkIdType cell_id = cell_picker_->GetCellId();
  if (cell_id < 0 || static_cast<std::size_t>(cell_id) >= face_region_.size())
  {
    return -1;
  }
  return static_cast<int>(cell_id);
}

bool TPPWidget::pickAnyPoint(const int x, const int y, Eigen::Vector3d& point) const
{
  cell_picker_->Pick(x, y, 0, renderer_);
  if (cell_picker_->GetCellId() < 0)
  {
    return false;
  }
  double picked[3] = { 0.0, 0.0, 0.0 };
  cell_picker_->GetPickPosition(picked);
  point = Eigen::Vector3d(picked[0], picked[1], picked[2]);
  return true;
}

int TPPWidget::pickPoint(const int x, const int y, Eigen::Vector3d& point) const
{
  const int face = pickFace(x, y);
  if (face >= 0)
  {
    // pickFace vient de lancer le rayon : la position touchee est encore dans le picker
    double picked[3] = { 0.0, 0.0, 0.0 };
    cell_picker_->GetPickPosition(picked);
    point = Eigen::Vector3d(picked[0], picked[1], picked[2]);
  }
  return face;
}

void TPPWidget::paintBrushDab(const int x, const int y, const bool erase)
{
  const int seed = pickFace(x, y);
  if (seed < 0)
  {
    return;  // the cursor is off the mesh: a dab in the air paints nothing
  }

  // The spin box carries a diameter in millimetres, the mesh is in metres.
  const double diameter_mm =
      (brush_diameter_spin_box_ != nullptr) ? brush_diameter_spin_box_->value() : kDefaultBrushDiameterMm;
  const std::vector<int> dab = facesWithinGeodesicRadius(face_adjacency_, face_centroids_, seed, diameter_mm / 2000.0);
  paintPending(dab, erase || tool_ == SelectionTool::Eraser);
}

void TPPWidget::paintBrushAlongSegment(const int x, const int y, const bool erase)
{
  const Eigen::Vector2d target(x, y);
  const Eigen::Vector2d start = drag_position_;
  drag_position_ = target;

  // A move event can jump tens of pixels. Painting a single dab per event would leave a dotted
  // trail, so the segment is sampled; the ceiling keeps a fast flick from casting hundreds of rays.
  const double span = (target - start).norm();
  const int samples = std::min(kMaxBrushSamplesPerMove, 1 + static_cast<int>(span / kBrushSampleSpacingPx));

  for (int i = 1; i <= samples; ++i)
  {
    const Eigen::Vector2d sample = start + (target - start) * (static_cast<double>(i) / samples);
    paintBrushDab(static_cast<int>(std::lround(sample.x())), static_cast<int>(std::lround(sample.y())), erase);
  }
}

std::vector<int> TPPWidget::facesInsideOutline() const
{
  std::vector<int> enclosed;
  if (!outline_.canClose() || face_centroids_.empty())
  {
    return enclosed;
  }

  const std::pair<Eigen::Vector2d, Eigen::Vector2d> bounds = outlineBounds(outline_.vertices());
  const int* window_size = renderer_->GetRenderWindow()->GetSize();

  // Hardware selection renders the scene once and returns the cells actually VISIBLE inside the
  // outline's bounding box, depth buffer included. Faces hidden behind the front surface never
  // enter the candidate set, so outlining a panel no longer grabs the near-parallel sheet or
  // flange behind it (which faces the camera too and used to pass the old normal-only test).
  vtkNew<vtkHardwareSelector> selector;
  selector->SetRenderer(renderer_);
  selector->SetFieldAssociation(vtkDataObject::FIELD_ASSOCIATION_CELLS);
  const unsigned int x_min = static_cast<unsigned int>(std::max(0.0, std::floor(bounds.first.x())));
  const unsigned int y_min = static_cast<unsigned int>(std::max(0.0, std::floor(bounds.first.y())));
  const unsigned int x_max =
      static_cast<unsigned int>(std::min(static_cast<double>(window_size[0] - 1), std::ceil(bounds.second.x())));
  const unsigned int y_max =
      static_cast<unsigned int>(std::min(static_cast<double>(window_size[1] - 1), std::ceil(bounds.second.y())));
  selector->SetArea(x_min, y_min, x_max, y_max);

  vtkSmartPointer<vtkSelection> selection;
  selection.TakeReference(selector->Select());
  if (selection == nullptr)
  {
    return enclosed;
  }

  for (unsigned int n = 0; n < selection->GetNumberOfNodes(); ++n)
  {
    vtkSelectionNode* node = selection->GetNode(n);
    // The tool paths, the axes and the fragment actors share the scene: only the selection mesh counts.
    if (node->GetProperties()->Has(vtkSelectionNode::PROP()) &&
        node->GetProperties()->Get(vtkSelectionNode::PROP()) != mesh_actor_.Get())
    {
      continue;
    }
    auto* ids = vtkIdTypeArray::SafeDownCast(node->GetSelectionList());
    if (ids == nullptr)
    {
      continue;
    }

    for (vtkIdType i = 0; i < ids->GetNumberOfValues(); ++i)
    {
      const vtkIdType cell = ids->GetValue(i);
      if (cell < 0 || static_cast<std::size_t>(cell) >= face_centroids_.size())
      {
        continue;
      }

      // Same projection convention as before: VTK display coordinates share their origin with the
      // interactor event positions the outline was built from.
      const Eigen::Vector3d& centroid = face_centroids_[static_cast<std::size_t>(cell)];
      renderer_->SetWorldPoint(centroid.x(), centroid.y(), centroid.z(), 1.0);
      renderer_->WorldToDisplay();
      double display[3] = { 0.0, 0.0, 0.0 };
      renderer_->GetDisplayPoint(display);
      if (pointInPolygon(Eigen::Vector2d(display[0], display[1]), outline_.vertices()))
      {
        enclosed.push_back(static_cast<int>(cell));
      }
    }
  }
  return enclosed;
}

void TPPWidget::onOutlineClick(const int x, const int y, const bool erase, const bool double_click)
{
  if (double_click)
  {
    // The first click of the pair has already placed its vertex: the second one only closes
    if (outline_.canClose())
    {
      closeOutline(erase);
    }
    return;
  }
  if (outline_.addVertex(Eigen::Vector2d(x, y)))
  {
    closeOutline(erase);
    return;
  }
  updateOutlineOverlay();
  render();
}

/** @brief Vrai pour les deux touches Entree du clavier, la principale et celle du pave numerique */
bool isEnterKey(const std::string& key) { return key == "Return" || key == "KP_Enter"; }

void TPPWidget::onKeyPressed(const std::string& key, const bool shift)
{
  if (tracing())
  {
    onTraceKey(key);
    return;
  }
  if (tool_ != SelectionTool::Polygon)
  {
    return;
  }
  if (isEnterKey(key))
  {
    if (outline_.canClose())
    {
      closeOutline(shift);
    }
    return;
  }
  if (key == "BackSpace")
  {
    outline_.removeLastVertex();
  }
  else if (key == "Escape")
  {
    outline_.clear();
  }
  else
  {
    return;
  }
  updateOutlineOverlay();
  render();
}

/**
 * @brief A closed polyline over `count` points: the chord from the last point back to the first is
 * drawn because the selection uses it too. Leaving it out would let the operator believe an open
 * curve was traced, then wonder why the selection cut straight across.
 */
vtkSmartPointer<vtkCellArray> closedPolyline(const vtkIdType count)
{
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(count + 1);
  for (vtkIdType i = 0; i < count; ++i)
  {
    lines->InsertCellPoint(i);
  }
  lines->InsertCellPoint(0);
  return lines;
}

void TPPWidget::updateOutlineOverlay()
{
  const std::vector<Eigen::Vector2d>& outline_points = outline_.vertices();
  const vtkIdType count = static_cast<vtkIdType>(outline_points.size());
  if (count < 2)
  {
    outline_actor_->SetVisibility(false);
    return;
  }

  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(count);
  for (vtkIdType i = 0; i < count; ++i)
  {
    const Eigen::Vector2d& point = outline_points[static_cast<std::size_t>(i)];
    points->SetPoint(i, point.x(), point.y(), 0.0);
  }

  outline_poly_->SetPoints(points);
  outline_poly_->SetLines(closedPolyline(count));
  outline_poly_->Modified();
  outline_actor_->SetVisibility(true);
}

void TPPWidget::updateToolControls()
{
  // Greying out the readings that do not apply says which parameter governs the active tool.
  flatness_angle_spin_box_->setEnabled(tool_ == SelectionTool::Region);
  brush_diameter_spin_box_->setEnabled(tool_ == SelectionTool::Brush || tool_ == SelectionTool::Eraser);
  if (trace_step_spin_box_ != nullptr)
  {
    trace_step_spin_box_->setEnabled(tracing());
  }
}
void TPPWidget::closeOutline(const bool erase)
{
  const std::vector<int> enclosed = facesInsideOutline();
  outline_.clear();
  updateOutlineOverlay();

  if (!enclosed.empty())
  {
    paintPending(enclosed, erase);
  }

  updateSelectionColors();
  render();
}

bool TPPWidget::checkPendingIsOneSurface(const std::size_t piece_count)
{
  if (piece_count == 0)
  {
    QMessageBox::information(this, "Nothing to add", "Select faces first (click, brush or polygon).");
    return false;
  }
  if (piece_count > 1)
  {
    // One surface entry means one continuous surface: a selection in several pieces is refused
    // rather than silently split, so what the list shows is what was deliberately built.
    QMessageBox::warning(this,
                         "Selection not continuous",
                         QString("The selection is made of %1 disconnected pieces. A surface must be "
                                 "one continuous patch: erase the strays (Shift) or add them as "
                                 "separate surfaces.")
                             .arg(piece_count));
    return false;
  }
  return true;
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

  auto* item = new QListWidgetItem();
  item->setData(Qt::UserRole, surface_id);
  item->setData(Qt::UserRole + 1, static_cast<int>(components.front().size()));
  relabelSurface(item);
  surface_list_->addItem(item);

  updateSelectionColors();
  render();
}
QString TPPWidget::describeTrace(const int surface_id) const
{
  const auto found = surface_traces_.find(surface_id);
  if (found == surface_traces_.end() || found->second.empty())
  {
    return QString();
  }
  return QString(" - trace manuel (%1 sommets)").arg(found->second.size());
}
void TPPWidget::relabelSurface(QListWidgetItem* item) const
{
  const int surface_id = item->data(Qt::UserRole).toInt();
  const int face_count = item->data(Qt::UserRole + 1).toInt();
  item->setText(QString("surface %1 - %2 faces%3").arg(surface_id + 1).arg(face_count).arg(describeTrace(surface_id)));
}

void TPPWidget::onRemoveSurface()
{
  QListWidgetItem* item = surface_list_->currentItem();
  if (item == nullptr)
  {
    return;
  }
  const int surface_id = item->data(Qt::UserRole).toInt();
  for (int& r : face_region_)
  {
    if (r == surface_id)
    {
      r = -1;
    }
  }
  surface_traces_.erase(surface_id);
  trace_preview_active_ = false;
  delete surface_list_->takeItem(surface_list_->row(item));
  updateTraceMarkers();
  updateSelectionColors();
  render();
}

void TPPWidget::clearSelection()
{
  // An outline being traced is part of the selection gesture: clearing one clears the other.
  outline_.clear();
  updateOutlineOverlay();
  std::fill(face_region_.begin(), face_region_.end(), -1);
  std::fill(pending_faces_.begin(), pending_faces_.end(), 0);
  if (surface_list_ != nullptr)
  {
    surface_list_->clear();
  }
  surface_traces_.clear();
  trace_preview_active_ = false;
  updateTraceMarkers();
  next_region_id_ = 0;
  updateSelectionColors();
  render();
}

bool TPPWidget::tracing() const { return trace_button_ != nullptr && trace_button_->isChecked(); }

void TPPWidget::buildTreatmentDock()
{
  // Le traitement, c'est le trace a la main : un bouton a bascule qui prend le tirage de la vue,
  // le pas des points, et de quoi corriger le trace sans le clavier
  trace_button_ = new QPushButton("Trace manuel (drag)", this);
  trace_button_->setCheckable(true);
  trace_button_->setToolTip("Enfonce : selectionnez une surface dans la liste, cliquez dessus pour le depart, "
                            "puis tirez : chaque tirage ajoute un segment droit, un point tous les X mm, avec "
                            "le cercle du disque autour de chaque point.");
  connect(trace_button_, &QPushButton::toggled, this, [this](bool checked) { onTraceToggled(checked); });

  trace_step_spin_box_ = new QDoubleSpinBox(this);
  trace_step_spin_box_->setRange(0.1, 10000.0);
  trace_step_spin_box_->setDecimals(1);
  trace_step_spin_box_->setSingleStep(5.0);
  trace_step_spin_box_->setValue(kDefaultTraceStepMm);
  trace_step_spin_box_->setSuffix(" mm");
  trace_step_spin_box_->setToolTip("Pas du trace : un point du chemin tous les X mm le long de chaque segment.");
  connect(trace_step_spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
    updateTraceMarkers();
    render();
  });

  approach_height_spin_box_ = new QDoubleSpinBox(this);
  approach_height_spin_box_->setRange(1.0, 1000.0);
  approach_height_spin_box_->setDecimals(0);
  approach_height_spin_box_->setSingleStep(10.0);
  approach_height_spin_box_->setValue(kDefaultApproachHeightMm);
  approach_height_spin_box_->setSuffix(" mm");
  approach_height_spin_box_->setToolTip("Hauteur du point d'approche au-dessus du premier point du trace, et du "
                                        "point de retrait au-dessus du dernier, le long de l'axe outil. "
                                        "Appliquee a la generation.");

  auto* dock = new QDockWidget("Traitement", this);
  dock->setWidget(buildTreatmentPanel());
  addDockWidget(Qt::RightDockWidgetArea, dock);
}

QWidget* TPPWidget::buildTreatmentPanel()
{
  auto* undo_vertex_button = new QPushButton("Retirer le dernier sommet", this);
  connect(undo_vertex_button, &QPushButton::clicked, this, [this](bool) { onTraceKey("BackSpace"); });
  auto* clear_trace_button = new QPushButton("Effacer le trace", this);
  connect(clear_trace_button, &QPushButton::clicked, this, [this](bool) { onTraceKey("Escape"); });

  auto* step_row = new QWidget(this);
  auto* step_layout = new QHBoxLayout(step_row);
  step_layout->setContentsMargins(0, 0, 0, 0);
  step_layout->addWidget(new QLabel("Pas du trace :", this));
  step_layout->addWidget(trace_step_spin_box_);
  auto* height_row = new QWidget(this);
  auto* height_layout = new QHBoxLayout(height_row);
  height_layout->setContentsMargins(0, 0, 0, 0);
  height_layout->addWidget(new QLabel("Hauteur d'approche :", this));
  height_layout->addWidget(approach_height_spin_box_);

  auto* panel = new QWidget(this);
  auto* layout = new QVBoxLayout(panel);
  layout->addWidget(trace_button_);
  layout->addWidget(step_row);
  layout->addWidget(height_row);
  layout->addWidget(undo_vertex_button);
  layout->addWidget(clear_trace_button);
  return panel;
}

void TPPWidget::onTraceToggled(const bool checked)
{
  // Le trace et le deplacement de points se partagent le tirage : un seul a la fois
  if (checked && move_points_button_ != nullptr)
  {
    move_points_button_->setChecked(false);
  }
  trace_preview_active_ = false;
  updateToolControls();
  updateTraceMarkers();
  statusBar()->showMessage(checked ? "Trace manuel : selectionnez une surface dans la liste, cliquez dessus pour "
                                     "le depart, puis tirez." :
                                     "Trace manuel : termine.");
  render();
}

void TPPWidget::buildPathEditDock()
{
  // La retouche porte sur le resultat de la generation : ordre des passes, suppression, points
  // deplaces. Les autres retouches (sens, soudure, enchainement) n'ont pas cours en tout-manuel.
  path_edit_ = new PathEditWidget(kMinPassPoses, this);
  path_edit_->hideAdvancedControls();
  path_edit_->on_changed = [this]() { onPathEdited(); };
  path_edit_->on_selection_changed = [this]() { updatePathHighlight(); };

  move_points_button_ = new QPushButton("Deplacer des points (drag)", this);
  move_points_button_->setCheckable(true);
  move_points_button_->setToolTip("Enfonce, le tirage dans la vue saisit le point du chemin le plus proche du "
                                  "clic et le pose ou la souris relache, sur la piece. Annuler defait un tirage.");
  connect(move_points_button_, &QPushButton::toggled, this, [this](bool checked) { onMovePointsToggled(checked); });
  validate_layout_button_ = new QPushButton("Valider la nouvelle disposition", this);
  validate_layout_button_->setToolTip("Sort du mode deplacement. Les points deplaces sont ceux du chemin livre : "
                                      "Save tool paths les ecrit tels quels.");
  connect(validate_layout_button_, &QPushButton::clicked, this, [this](bool) { validateLayout(); });

  load_tool_paths_button_ = new QPushButton("Charger un chemin...", this);
  load_tool_paths_button_->setToolTip("Relit un chemin enregistre (tool_path_plan.yaml) pour le retoucher : ordre, "
                                      "suppression, deplacement de points. Save tool paths reecrit le resultat.");
  connect(load_tool_paths_button_, &QPushButton::clicked, this, [this](bool) { onLoadToolPaths(); });

  auto* panel = new QWidget(this);
  auto* layout = new QVBoxLayout(panel);
  layout->addWidget(load_tool_paths_button_);
  layout->addWidget(path_edit_);
  layout->addWidget(move_points_button_);
  layout->addWidget(validate_layout_button_);
  auto* dock = new QDockWidget("Trajectoire", this);
  dock->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, dock);
}

void TPPWidget::onMovePointsToggled(const bool checked)
{
  grab_active_ = false;
  if (checked && trace_button_ != nullptr)
  {
    trace_button_->setChecked(false);
  }
  updateGrabMarker();
  updateTraceMarkers();
  statusBar()->showMessage(checked ? "Deplacer des points : le tirage dans la vue saisit un point du chemin." :
                                     "Deplacer des points : termine.");
  render();
}

void TPPWidget::plan()
{
  try
  {
    if (!confirmDiscardPathEdits())
    {
      return;
    }
    if (std::any_of(pending_faces_.begin(), pending_faces_.end(), [](char p) { return p != 0; }))
    {
      QMessageBox::warning(this,
                           "Selection non ajoutee",
                           "La selection orange n'appartient a aucune surface. Ajoutez-la a la liste, ou "
                           "effacez-la, puis generez.");
      return;
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    tool_paths_.clear();
    fragments_.clear();
    std::vector<ToolPaths> unmodified_tool_paths;
    const std::vector<int> untraced = planTracedSurfaces(unmodified_tool_paths);
    QApplication::restoreOverrideCursor();
    showPlannedPaths(unmodified_tool_paths);
    reportUntracedSurfaces(untraced);
  }
  catch (const std::exception& ex)
  {
    QApplication::restoreOverrideCursor();
    // Les surfaces deja planifiees sont dans tool_paths_, celles d'apres non : garder ce
    // resultat partiel laisserait afficher, et enregistrer, un chemin ampute en silence
    clearPlannedPaths();
    std::stringstream ss;
    printException(ex, ss);
    QMessageBox::warning(this, "Erreur de generation", QString::fromStdString(ss.str()));
  }
}

std::vector<int> TPPWidget::planTracedSurfaces(std::vector<ToolPaths>& unmodified)
{
  // Les faces de chaque surface, dans l'ordre des surfaces ; une surface sans trace n'a rien a
  // generer et est rapportee
  std::map<int, std::vector<int>> region_faces;
  for (std::size_t f = 0; f < face_region_.size(); ++f)
  {
    if (face_region_[f] >= 0)
    {
      region_faces[face_region_[f]].push_back(static_cast<int>(f));
    }
  }
  std::vector<int> untraced;
  for (const auto& entry : region_faces)
  {
    if (surfaceHasTrace(entry.first))
    {
      planTracedSurface(entry.first, entry.second, unmodified);
    }
    else
    {
      untraced.push_back(entry.first);
    }
  }
  return untraced;
}

void TPPWidget::showPlannedPaths(const std::vector<ToolPaths>& unmodified)
{
  renderer_->RemoveActor(mesh_fragment_actor_);
  mesh_fragment_actor_ = createMeshActors(fragments_);
  renderer_->AddActor(mesh_fragment_actor_);
  mesh_fragment_actor_->SetVisibility(ui_->action_show_modified_mesh->isChecked());

  renderer_->RemoveActor(unmodified_tool_path_actor_);
  unmodified_tool_path_actor_ = createToolPathActors(unmodified, tube_filter_->GetOutputPort());
  renderer_->AddActor(unmodified_tool_path_actor_);
  unmodified_tool_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path->isChecked());

  renderer_->RemoveActor(unmodified_connected_path_actor_);
  unmodified_connected_path_actor_ = createToolPathPolylineActor(unmodified, tube_filter_->GetOutputPort());
  renderer_->AddActor(unmodified_connected_path_actor_);
  unmodified_connected_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path_lines->isChecked());

  // Le dock Trajectoire recoit les passes ; c'est lui qui dessine le chemin livre a partir d'ici
  path_edit_->setGenerated(flattenToolPaths(tool_paths_));
  render();
}

void TPPWidget::reportUntracedSurfaces(const std::vector<int>& untraced)
{
  if (untraced.empty())
  {
    statusBar()->showMessage(QString("%1 surface(s) generee(s) depuis leur trace.").arg(tool_paths_.size()));
    return;
  }
  QStringList names;
  for (const int surface_id : untraced)
  {
    names << QString::number(surface_id + 1);
  }
  QMessageBox::information(this,
                           "Surfaces sans trace",
                           QString("Les surfaces %1 n'ont pas de trace manuel : rien n'est genere pour elles. "
                                   "Onglet Traitement, Trace manuel, puis tracez dessus.")
                               .arg(names.join(", ")));
}

void TPPWidget::onDragStart(const int x, const int y, const bool erase)
{
  drag_position_ = Eigen::Vector2d(x, y);
  // Le deplacement de points prime sur l'outil de selection tant que son bouton est enfonce
  if (movingPoints())
  {
    onPointDragStart(x, y);
    return;
  }
  if (tracing())
  {
    onTraceDragStart(x, y);
    return;
  }

  switch (tool_)
  {
    case SelectionTool::Brush:
    case SelectionTool::Eraser:
      paintBrushDab(x, y, erase);
      updateSelectionColors();
      render();
      break;
    case SelectionTool::Region:
    case SelectionTool::Polygon:
      break;  // the click path handles them; neither consumes a drag
  }
}

void TPPWidget::onDragMove(const int x, const int y, const bool erase)
{
  if (movingPoints())
  {
    onPointDragMove(x, y);
    return;
  }
  if (tracing())
  {
    onTraceDragMove(x, y);
    return;
  }
  switch (tool_)
  {
    case SelectionTool::Brush:
    case SelectionTool::Eraser:
      paintBrushAlongSegment(x, y, erase);
      updateSelectionColors();
      render();
      break;
    case SelectionTool::Region:
    case SelectionTool::Polygon:
      break;
  }
}

void TPPWidget::onDragEnd(const bool /*erase*/)
{
  // The brush has already committed every dab, and the polygon works by clicks: only the trace
  // has a segment left to commit when a drag ends.
  if (movingPoints())
  {
    onPointDragEnd();
  }
  else if (tracing())
  {
    onTraceDragEnd();
  }
}

bool TPPWidget::movingPoints() const { return move_points_button_ != nullptr && move_points_button_->isChecked(); }

void TPPWidget::onPointDragStart(const int x, const int y)
{
  grab_active_ = false;
  Eigen::Vector3d clicked;
  if (!pickAnyPoint(x, y, clicked) || path_edit_->passes().empty())
  {
    statusBar()->showMessage("Cliquez pres d'un point du chemin genere pour le saisir.");
    return;
  }
  // La pose est cherchee dans la generation deplacee : c'est elle que la recette assemble, et
  // ses indices ne bougent pas quand les passes sont reordonnees
  if (!locateContactPose(clicked, grabbed_))
  {
    statusBar()->showMessage("Aucun point de contact du chemin a moins de 50 mm du clic.");
    return;
  }
  grab_active_ = true;
  grab_origin_ = grabbed_.pose.translation();
  grab_segment_before_ = path_edit_->movedGenerated()[grabbed_.segment];
  path_edit_->beginPoseMoves();
  // Le cercle cyan de la pose saisie s'efface : c'est le cercle rose qui la suit
  updateDiscCircles();
  updateGrabMarker();
  statusBar()->showMessage("Point saisi : tirez-le ou vous voulez sur la piece, relachez pour le poser.");
  render();
}

void TPPWidget::onPointDragMove(const int x, const int y)
{
  if (!grab_active_)
  {
    return;
  }
  Eigen::Vector3d point;
  const int face = pickPoint(x, y, point);
  if (face < 0)
  {
    return;
  }
  // La pose suit le curseur sur la piece : z le long de la face touchee, x garde son sens de
  // marche rendu orthogonal a la nouvelle normale
  const Eigen::Vector3d z = face_normals_[static_cast<std::size_t>(face)];
  Eigen::Vector3d x_axis = grabbed_.pose.linear().col(0);
  x_axis -= x_axis.dot(z) * z;
  if (x_axis.norm() < 1e-9)
  {
    x_axis = z.unitOrthogonal();
  }
  x_axis.normalize();
  grabbed_.pose.linear().col(0) = x_axis;
  grabbed_.pose.linear().col(1) = z.cross(x_axis);
  grabbed_.pose.linear().col(2) = z;
  grabbed_.pose.translation() = point;
  updateGrabMarker();
  render();
}

void TPPWidget::onPointDragEnd()
{
  if (!grab_active_)
  {
    return;
  }
  grab_active_ = false;
  // Un seul recalcul du chemin et des cercles, au relachement ; l'approche ou le retrait suit
  path_edit_->movePose(grabbed_);
  moveSkirtPoses();
  updateGrabMarker();
  statusBar()->showMessage(QString("Point pose. %1 point(s) deplace(s) a la main ; Annuler les defait un tirage a la fois.")
                               .arg(path_edit_->moves().size()));
  render();
}

void TPPWidget::validateLayout()
{
  grab_active_ = false;
  move_points_button_->setChecked(false);
  onPathEdited();
  const std::size_t moved = path_edit_->moves().size();
  statusBar()->showMessage(QString("Disposition validee : %1 point(s) deplace(s). Save tool paths ecrit ces positions "
                                   "dans le chemin, et la retouche a cote.")
                               .arg(moved));
  render();
}

bool TPPWidget::locateContactPose(const Eigen::Vector3d& clicked, PoseMove& found) const
{
  // Les poses d'approche et de retrait sont hors contact : on ne les propose pas a la saisie
  std::vector<ToolPathSegment> contact_only = path_edit_->movedGenerated();
  for (ToolPathSegment& pass : contact_only)
  {
    if (pass.size() > 2 * kSkirtPoses)
    {
      pass.erase(pass.end() - static_cast<std::ptrdiff_t>(kSkirtPoses), pass.end());
      pass.erase(pass.begin(), pass.begin() + static_cast<std::ptrdiff_t>(kSkirtPoses));
    }
  }
  if (!locatePose(contact_only, clicked, kGrabToleranceM, found))
  {
    return false;
  }
  // Retour aux indices de la passe complete, approche comprise
  if (contact_only[found.segment].size() + 2 * kSkirtPoses == path_edit_->movedGenerated()[found.segment].size())
  {
    found.index += kSkirtPoses;
  }
  return true;
}

void TPPWidget::moveSkirtPoses()
{
  const ToolPathSegment& before = grab_segment_before_;
  if (before.size() <= 2 * kSkirtPoses || grabbed_.index >= before.size())
  {
    return;
  }
  const Eigen::Isometry3d origin_pose = before[grabbed_.index];
  // Approche devant la premiere pose de contact, retrait derriere la derniere
  std::vector<std::size_t> skirts;
  if (grabbed_.index == kSkirtPoses)
  {
    skirts.push_back(0);
  }
  if (grabbed_.index + kSkirtPoses + 1 == before.size())
  {
    skirts.push_back(before.size() - 1);
  }
  for (const std::size_t skirt : skirts)
  {
    // La place de l'approche dans le repere de la pose de contact ne change pas : elle reste a la
    // meme hauteur, le long du nouvel axe outil
    PoseMove follower;
    follower.segment = grabbed_.segment;
    follower.index = skirt;
    follower.pose = grabbed_.pose * (origin_pose.inverse() * before[skirt]);
    path_edit_->movePose(follower);
  }
}

void TPPWidget::updateGrabMarker()
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto vertices = vtkSmartPointer<vtkCellArray>::New();
  if (grab_active_)
  {
    const Eigen::Vector3d& t = grabbed_.pose.translation();
    const vtkIdType id = points->InsertNextPoint(t.x(), t.y(), t.z());
    vertices->InsertNextCell(1, &id);
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(vertices);
  grab_actor_ = replaceActor(*renderer_, grab_actor_, poly);
  grab_actor_->GetProperty()->SetColor(kGrabRgb[0] / 255.0, kGrabRgb[1] / 255.0, kGrabRgb[2] / 255.0);
  grab_actor_->GetProperty()->SetPointSize(22.0f);
  grab_actor_->GetProperty()->SetRenderPointsAsSpheres(true);

  // Le cercle du disque suit la pose saisie, dans la meme couleur qu'elle
  auto circle_points = vtkSmartPointer<vtkPoints>::New();
  auto circle_lines = vtkSmartPointer<vtkCellArray>::New();
  const bool radius_enabled = tool_radius_enabled_check_box_->isChecked() && tool_radius_spin_box_->value() > 0.0;
  if (grab_active_ && radius_enabled)
  {
    appendDiscCircle(grabbed_.pose, tool_radius_spin_box_->value() / 1000.0, *circle_points, *circle_lines);
  }
  auto circle = vtkSmartPointer<vtkPolyData>::New();
  circle->SetPoints(circle_points);
  circle->SetLines(circle_lines);
  grab_disc_actor_ = replaceActor(*renderer_, grab_disc_actor_, circle);
  grab_disc_actor_->GetProperty()->SetColor(kGrabRgb[0] / 255.0, kGrabRgb[1] / 255.0, kGrabRgb[2] / 255.0);
  grab_disc_actor_->GetProperty()->SetLineWidth(3.0f);
}

int TPPWidget::selectedSurfaceId() const
{
  QListWidgetItem* item = surface_list_ != nullptr ? surface_list_->currentItem() : nullptr;
  return item != nullptr ? item->data(Qt::UserRole).toInt() : -1;
}

void TPPWidget::onTraceDragStart(const int x, const int y)
{
  const int surface_id = selectedSurfaceId();
  if (surface_id < 0)
  {
    statusBar()->showMessage("Selectionnez d'abord une surface dans la liste : le trace lui appartient.");
    return;
  }
  Eigen::Vector3d point;
  const int face = pickPoint(x, y, point);
  if (face < 0 || face_region_[static_cast<std::size_t>(face)] != surface_id)
  {
    statusBar()->showMessage("Le trace commence sur la surface selectionnee : cliquez dessus.");
    return;
  }
  // Un trace vide prend ce point pour depart ; sinon le segment part du dernier sommet
  if (surface_traces_[surface_id].empty())
  {
    surface_traces_[surface_id].push_back(point);
    statusBar()->showMessage("Depart du trace fixe. Tirez pour continuer le chemin.");
  }
  trace_preview_end_ = point;
  trace_preview_active_ = true;
  updateTraceMarkers();
  render();
}

void TPPWidget::onTraceDragMove(const int x, const int y)
{
  if (!trace_preview_active_)
  {
    return;
  }
  Eigen::Vector3d point;
  const int face = pickPoint(x, y, point);
  // Hors de la surface, la fin provisoire reste ou elle etait : le trace ne la quitte pas
  if (face < 0 || face_region_[static_cast<std::size_t>(face)] != selectedSurfaceId())
  {
    return;
  }
  trace_preview_end_ = point;
  updateTraceMarkers();
  render();
}

void TPPWidget::onTraceDragEnd()
{
  if (!trace_preview_active_)
  {
    return;
  }
  trace_preview_active_ = false;
  const int surface_id = selectedSurfaceId();
  if (surface_id < 0)
  {
    return;
  }
  std::vector<Eigen::Vector3d>& vertices = surface_traces_[surface_id];
  // Un tirage qui n'a pas quitte le dernier sommet n'ajoute rien
  if (!vertices.empty() && (trace_preview_end_ - vertices.back()).norm() > 1e-6)
  {
    vertices.push_back(trace_preview_end_);
  }
  if (QListWidgetItem* item = surface_list_->currentItem())
  {
    relabelSurface(item);
  }
  statusBar()->showMessage(QString("Trace : %1 sommets. Tirez encore pour continuer, Retour arriere pour "
                                   "retirer le dernier, Echap pour effacer.")
                               .arg(vertices.size()));
  updateTraceMarkers();
  render();
}

void TPPWidget::onTraceKey(const std::string& key)
{
  const int surface_id = selectedSurfaceId();
  if (surface_id < 0 || surface_traces_.count(surface_id) == 0)
  {
    return;
  }
  if (key == "BackSpace" && !surface_traces_[surface_id].empty())
  {
    surface_traces_[surface_id].pop_back();
  }
  else if (key == "Escape")
  {
    surface_traces_.erase(surface_id);
  }
  else
  {
    return;
  }
  trace_preview_active_ = false;
  if (QListWidgetItem* item = surface_list_->currentItem())
  {
    relabelSurface(item);
  }
  updateTraceMarkers();
  render();
}

std::vector<Eigen::Vector3d> TPPWidget::traceVerticesWithPreview(const int surface_id) const
{
  std::vector<Eigen::Vector3d> vertices;
  const auto found = surface_traces_.find(surface_id);
  if (found != surface_traces_.end())
  {
    vertices = found->second;
  }
  if (trace_preview_active_)
  {
    vertices.push_back(trace_preview_end_);
  }
  return vertices;
}

std::vector<Eigen::Vector3d> TPPWidget::projectedTraceSamples(const int surface_id,
                                                              std::vector<Eigen::Vector3d>& normals) const
{
  normals.clear();
  std::vector<Eigen::Vector3d> samples;
  // Le trace ne se montre que pendant qu'on le trace : apres generation, ses points et ses
  // cercles se confondraient avec ceux du chemin, et ne suivraient pas un point deplace
  if (surface_id < 0 || trace_step_spin_box_ == nullptr || !tracing() || movingPoints())
  {
    return samples;
  }
  // Dessines et evalues la ou l'outil ira vraiment : sur la surface, pas sur la corde
  samples = sampleTrace(traceVerticesWithPreview(surface_id), trace_step_spin_box_->value() / 1000.0);
  projectOnSurface(surface_id, samples, normals);
  return samples;
}

/// @brief Points colores du trace : depart vert, arrivee rouge, les autres cyan
vtkSmartPointer<vtkPolyData> traceMarkerPoints(const std::vector<Eigen::Vector3d>& samples)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto vertices = vtkSmartPointer<vtkCellArray>::New();
  auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  for (std::size_t k = 0; k < samples.size(); ++k)
  {
    const vtkIdType id = points->InsertNextPoint(samples[k].x(), samples[k].y(), samples[k].z());
    vertices->InsertNextCell(1, &id);
    colors->InsertNextTypedTuple(k == 0 ? kStartRgb : (k + 1 == samples.size() ? kEndRgb : kTraceRgb));
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(vertices);
  poly->GetPointData()->SetScalars(colors);
  return poly;
}

/// @brief Ligne brisee passant par les points du trace, dans l'ordre
vtkSmartPointer<vtkPolyData> traceLine(const std::vector<Eigen::Vector3d>& samples)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto line = vtkSmartPointer<vtkPolyLine>::New();
  line->GetPointIds()->SetNumberOfIds(static_cast<vtkIdType>(samples.size()));
  for (std::size_t k = 0; k < samples.size(); ++k)
  {
    points->InsertNextPoint(samples[k].x(), samples[k].y(), samples[k].z());
    line->GetPointIds()->SetId(static_cast<vtkIdType>(k), static_cast<vtkIdType>(k));
  }
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  if (samples.size() >= 2)
  {
    lines->InsertNextCell(line);
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetLines(lines);
  return poly;
}

/// @brief Acteur d'un nuage ou d'une ligne, remplace dans le rendu
vtkSmartPointer<vtkActor> replaceActor(vtkRenderer& renderer, vtkSmartPointer<vtkActor> previous, vtkPolyData* poly)
{
  renderer.RemoveActor(previous);
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(poly);
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  renderer.AddActor(actor);
  return actor;
}

void TPPWidget::updateTraceMarkers()
{
  // Seul le trace de la surface selectionnee est dessine, pour que l'operateur voie ce qu'il edite
  std::vector<Eigen::Vector3d> normals;
  const std::vector<Eigen::Vector3d> samples = projectedTraceSamples(selectedSurfaceId(), normals);
  trace_actor_ = replaceActor(*renderer_, trace_actor_, traceMarkerPoints(samples));
  trace_actor_->GetProperty()->SetPointSize(16.0f);
  trace_actor_->GetProperty()->SetRenderPointsAsSpheres(true);
  trace_line_actor_ = replaceActor(*renderer_, trace_line_actor_, traceLine(samples));
  trace_line_actor_->GetProperty()->SetColor(kTraceRgb[0] / 255.0, kTraceRgb[1] / 255.0, kTraceRgb[2] / 255.0);
  trace_line_actor_->GetProperty()->SetLineWidth(3.0f);
  // L'empreinte du disque des le trace, pour tirer le trait a l'ecart des bords tout de suite
  trace_disc_actor_ = replaceActor(*renderer_, trace_disc_actor_, flatDiscCircles(samples, normals));
  trace_disc_actor_->GetProperty()->SetColor(kDiscRgb[0] / 255.0, kDiscRgb[1] / 255.0, kDiscRgb[2] / 255.0);
  trace_disc_actor_->GetProperty()->SetLineWidth(1.5f);
}

vtkSmartPointer<vtkPolyData> TPPWidget::flatDiscCircles(const std::vector<Eigen::Vector3d>& points,
                                                        const std::vector<Eigen::Vector3d>& normals) const
{
  auto vtk_points = vtkSmartPointer<vtkPoints>::New();
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  const bool radius_enabled = tool_radius_enabled_check_box_->isChecked() && tool_radius_spin_box_->value() > 0.0;
  if (radius_enabled)
  {
    const double radius_m = tool_radius_spin_box_->value() / 1000.0;
    for (std::size_t k = 0; k < points.size(); ++k)
    {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normals[k]).toRotationMatrix();
      pose.translation() = points[k];
      appendDiscCircle(pose, radius_m, *vtk_points, *lines);
    }
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(vtk_points);
  poly->SetLines(lines);
  return poly;
}

/// @brief Un cercle de rayon donne dans le plan (x, y) de chaque pose, ajoute au nuage et aux lignes
void appendDiscCircle(const Eigen::Isometry3d& pose, const double radius, vtkPoints& points, vtkCellArray& lines)
{
  auto circle = vtkSmartPointer<vtkPolyLine>::New();
  circle->GetPointIds()->SetNumberOfIds(kCircleSegments + 1);
  for (int i = 0; i <= kCircleSegments; ++i)
  {
    // Le dernier point reprend le premier : le cercle est ferme
    const double angle = 2.0 * M_PI * static_cast<double>(i % kCircleSegments) / kCircleSegments;
    const Eigen::Vector3d rim = pose * Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), 0.0);
    circle->GetPointIds()->SetId(i, points.InsertNextPoint(rim.x(), rim.y(), rim.z()));
  }
  lines.InsertNextCell(circle);
}

void TPPWidget::updateDiscCircles()
{
  renderer_->RemoveActor(disc_actor_);
  disc_actor_ = nullptr;
  const bool radius_enabled = tool_radius_enabled_check_box_->isChecked() && tool_radius_spin_box_->value() > 0.0;
  if (path_edit_ == nullptr || !radius_enabled)
  {
    return;
  }
  // Les poses d'approche et de retrait, hors contact, n'ont pas d'empreinte
  const double radius_m = tool_radius_spin_box_->value() / 1000.0;
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  for (const ToolPathSegment& pass : path_edit_->passes())
  {
    for (std::size_t k = kSkirtPoses; k + kSkirtPoses < pass.size(); ++k)
    {
      // La pose en cours de deplacement garde son cercle rose, pas son cercle cyan d'origine
      if (grab_active_ && (pass[k].translation() - grab_origin_).norm() < 1e-9)
      {
        continue;
      }
      appendDiscCircle(pass[k], radius_m, *points, *lines);
    }
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetLines(lines);
  disc_actor_ = replaceActor(*renderer_, disc_actor_, poly);
  disc_actor_->GetProperty()->SetColor(kDiscRgb[0] / 255.0, kDiscRgb[1] / 255.0, kDiscRgb[2] / 255.0);
  disc_actor_->GetProperty()->SetLineWidth(2.0f);
}

void TPPWidget::projectOnSurface(const int surface_id,
                                 std::vector<Eigen::Vector3d>& points,
                                 std::vector<Eigen::Vector3d>& normals) const
{
  // Sous-maillage de la surface, dans l'ordre de ses faces : la cellule k est la face faces[k]
  std::vector<int> faces;
  for (std::size_t f = 0; f < face_region_.size(); ++f)
  {
    if (face_region_[f] == surface_id)
    {
      faces.push_back(static_cast<int>(f));
    }
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(extractSubMeshFromFaces(mesh_, faces), poly);
  auto locator = vtkSmartPointer<vtkCellLocator>::New();
  locator->SetDataSet(poly);
  locator->BuildLocator();

  normals.clear();
  for (Eigen::Vector3d& point : points)
  {
    double closest[3] = { 0.0, 0.0, 0.0 };
    vtkIdType cell = -1;
    int sub_id = 0;
    double squared_distance = 0.0;
    locator->FindClosestPoint(point.data(), closest, cell, sub_id, squared_distance);
    point = Eigen::Vector3d(closest[0], closest[1], closest[2]);
    normals.push_back(face_normals_[static_cast<std::size_t>(faces[static_cast<std::size_t>(cell)])]);
  }
}

bool TPPWidget::surfaceHasTrace(const int surface_id) const
{
  const auto found = surface_traces_.find(surface_id);
  return found != surface_traces_.end() && found->second.size() >= 2;
}

ToolPaths TPPWidget::tracedToolPaths(const int surface_id) const
{
  // Les points de la corde sont ramenes sur la surface, avec la normale de la face touchee :
  // l'outil reste pose dessus, meme quand la corde passe sous une surface courbe
  std::vector<Eigen::Vector3d> points =
      sampleTrace(surface_traces_.at(surface_id), trace_step_spin_box_->value() / 1000.0);
  std::vector<Eigen::Vector3d> normals;
  projectOnSurface(surface_id, points, normals);
  return { ToolPath{ tracePoses(points, normals) } };
}
void TPPWidget::onClearSelection(const bool /*checked*/) { clearSelection(); }

void TPPWidget::onLoadMesh(const bool /*checked*/)
{
  const QString file = QFileDialog::getOpenFileName(this, "Load mesh file", "", "Mesh files (*.ply *.stl)");
  if (!file.isNull())
    setMeshFile(file);
}

void TPPWidget::configure(const QString& file) { pipeline_widget_->configure(file); }

vtkSmartPointer<vtkTransform> toVTK(const Eigen::Isometry3d& mat)
{
  auto t = vtkSmartPointer<vtkTransform>::New();
  t->Translate(mat.translation().data());
  Eigen::AngleAxisd aa(mat.rotation());
  t->RotateWXYZ(aa.angle() * 180.0 / M_PI, aa.axis().data());
  return t;
}

vtkSmartPointer<vtkPropAssembly> createToolPathActors(const std::vector<ToolPaths>& tool_paths,
                                                      vtkAlgorithmOutput* waypoint_shape_output_port)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();

  for (const ToolPaths& fragment : tool_paths)
  {
    for (const ToolPath& tool_path : fragment)
    {
      for (const ToolPathSegment& segment : tool_path)
      {
        for (const Eigen::Isometry3d& w : segment)
        {
          auto transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
          transform_filter->SetTransform(toVTK(w));
          transform_filter->SetInputConnection(waypoint_shape_output_port);

          auto map = vtkSmartPointer<vtkPolyDataMapper>::New();
          map->SetInputConnection(transform_filter->GetOutputPort());

          auto actor = vtkSmartPointer<vtkActor>::New();
          actor->SetMapper(map);

          assembly->AddPart(actor);
        }
      }
    }
  }

  return assembly;
}

vtkSmartPointer<vtkLeaderActor2D> createLineActor(const Eigen::Isometry3d& point1,
                                                  const Eigen::Isometry3d& point2,
                                                  LineStyle lineStyle,
                                                  bool include_arrow)
{
  vtkNew<vtkLeaderActor2D> arrow;

  if (include_arrow)
    arrow->SetArrowPlacementToPoint2();
  else
    arrow->SetArrowPlacementToNone();

  vtkSmartPointer<vtkCoordinate> p1 = arrow->GetPositionCoordinate();
  p1->SetCoordinateSystemToWorld();
  p1->SetValue(point1.translation().data());

  vtkSmartPointer<vtkCoordinate> p2 = arrow->GetPosition2Coordinate();
  p2->SetCoordinateSystemToWorld();
  p2->SetValue(point2.translation().data());

  vtkNew<vtkProperty2D> property;
  switch (lineStyle)
  {
    case LineStyle::INTRA_SEGMENT:
      property->SetLineWidth(1.0);
      property->SetColor(0.27, 0.74, 0.2);
      break;
    case LineStyle::INTER_SEGMENT:
      property->SetLineWidth(0.5);
      property->SetColor(0, 0, 1);
      break;
    case LineStyle::INTER_PATH:
      property->SetLineWidth(0.5);
      property->SetColor(1, 0.271, 0.043);
      break;
  }

  arrow->SetProperty(property);

  return arrow;
}

/*
 *  Create a polyline between all points of the tool path
 *
 *  @param tool_paths The tool paths to create the polyline from
 *  @param waypoint_shape_output_port The output port of the waypoint shape
 *  @return The assembly containing the polyline
 */
vtkSmartPointer<vtkPropAssembly> createToolPathPolylineActor(const std::vector<ToolPaths>& tool_paths,
                                                             vtkAlgorithmOutput* waypoint_shape_output_port)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();

  for (const ToolPaths& fragment : tool_paths)
  {
    for (int i = 0; i < fragment.size(); i++)
    {
      const ToolPath& tool_path = fragment[i];

      for (int j = 0; j < tool_path.size(); j++)
      {
        const ToolPathSegment& segment = tool_path[j];

        for (int k = 0; k < segment.size() - 1; k++)
        {
          const Eigen::Isometry3d& point = segment[k];
          const Eigen::Isometry3d& next_point = segment[k + 1];
          const bool include_arrow = (k + 2 == segment.size());
          auto actor = createLineActor(point, next_point, LineStyle::INTRA_SEGMENT, include_arrow);
          assembly->AddPart(actor);
        }

        if (j < tool_path.size() - 1)
        {
          const ToolPathSegment& next_segment = tool_path[j + 1];
          const Eigen::Isometry3d& w1 = segment.back();
          const Eigen::Isometry3d& w2 = next_segment.front();

          auto actor = createLineActor(w1, w2, LineStyle::INTER_SEGMENT, true);
          assembly->AddPart(actor);
        }
      }

      if (i < fragment.size() - 1)
      {
        const ToolPath& next_tool_path = fragment[i + 1];
        const Eigen::Isometry3d& w1 = tool_path.back().back();
        const Eigen::Isometry3d& w2 = next_tool_path.front().front();

        auto actor = createLineActor(w1, w2, LineStyle::INTER_PATH, true);
        assembly->AddPart(actor);
      }
    }
  }

  return assembly;
}

/**
 * @brief Builds the polyline running through the waypoints of one pass.
 */
vtkSmartPointer<vtkPolyData> buildPassPolyline(const ToolPathSegment& pass)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto line = vtkSmartPointer<vtkPolyLine>::New();
  line->GetPointIds()->SetNumberOfIds(static_cast<vtkIdType>(pass.size()));
  for (std::size_t i = 0; i < pass.size(); ++i)
  {
    const Eigen::Vector3d& position = pass[i].translation();
    points->InsertNextPoint(position.x(), position.y(), position.z());
    line->GetPointIds()->SetId(static_cast<vtkIdType>(i), static_cast<vtkIdType>(i));
  }

  auto cells = vtkSmartPointer<vtkCellArray>::New();
  cells->InsertNextCell(line);

  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetLines(cells);
  return poly;
}

/**
 * @brief Draws one pass as a single thick line, to mark it out in the viewer.
 * @details One actor for the whole pass rather than one per waypoint: this is rebuilt every time
 * the operator picks another row in the Trajectory dock.
 */
vtkSmartPointer<vtkPropAssembly> createPassHighlightActor(const ToolPathSegment& pass)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();
  if (pass.size() < 2)
  {
    return assembly;
  }

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(buildPassPolyline(pass));

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(1.0, 0.9, 0.0);
  actor->GetProperty()->SetLineWidth(5.0);
  assembly->AddPart(actor);
  return assembly;
}

vtkSmartPointer<vtkPropAssembly> createMeshActors(const std::vector<pcl::PolygonMesh>& meshes)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();

  // Create a color series to differentiate the meshes
  auto color_series = vtkSmartPointer<vtkColorSeries>::New();
  color_series->SetColorScheme(vtkColorSeries::ColorSchemes::BREWER_QUALITATIVE_SET1);
  color_series->SetNumberOfColors(meshes.size());

  for (std::size_t i = 0; i < meshes.size(); ++i)
  {
    vtkSmartPointer<vtkPolyData> mesh_poly_data = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(meshes[i], mesh_poly_data);

    auto map = vtkSmartPointer<vtkPolyDataMapper>::New();
    map->SetInputData(mesh_poly_data);
    map->SetScalarVisibility(false);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(map);

    // Set the color of the actor from the color series
    vtkTuple<double, 3> color = color_series->GetColorRepeating(i).Cast<double>();
    double r = color[0] / 255.0;
    double g = color[1] / 255.0;
    double b = color[2] / 255.0;
    actor->GetProperty()->SetColor(r, g, b);

    assembly->AddPart(actor);
  }

  return assembly;
}

void TPPWidget::planTracedSurface(const int surface_id,
                                  const std::vector<int>& faces,
                                  std::vector<ToolPaths>& unmodified)
{
  const ToolPaths path = tracedToolPaths(surface_id);
  fragments_.push_back(extractSubMeshFromFaces(mesh_, faces));
  unmodified.push_back(path);
  // Le chemin livre : le trace, une approche devant, un retrait derriere, rien d'autre
  const double height_m = approach_height_spin_box_->value() / 1000.0;
  ToolPaths delivered;
  for (const ToolPath& tool_path : path)
  {
    ToolPath with_skirts;
    for (const ToolPathSegment& segment : tool_path)
    {
      with_skirts.push_back(withApproachAndRetract(segment, height_m));
    }
    delivered.push_back(with_skirts);
  }
  tool_paths_.push_back(delivered);
}
void TPPWidget::saveModifiedMeshes(const QDir& save_dir)
{
  // Extract the submeshes from the actor that holds the modified meshes
  vtkPropCollection* parts = mesh_fragment_actor_->GetParts();
  if (parts->GetNumberOfItems() == 0)
  {
    QMessageBox::warning(this, "Error", "No modified meshes found; please plan a tool path first.");
    return;
  }

  for (vtkIdType i = 0; i < parts->GetNumberOfItems(); i++)
  {
    auto actor = vtkActor::SafeDownCast(parts->GetItemAsObject(i));
    if (!actor)
      continue;

    auto map = vtkPolyDataMapper::SafeDownCast(actor->GetMapper());
    if (!map)
      continue;

    auto mesh_poly_data = vtkPolyData::SafeDownCast(map->GetInput());
    if (!mesh_poly_data)
      continue;

    QString file_name;
    QTextStream ss(&file_name);
    ss << "modified_mesh_" << i << ".ply";
    QFileInfo file_info(save_dir, file_name);

    auto writer = vtkSmartPointer<vtkPLYWriter>::New();
    // Write the modified mesh to a file
    writer->SetInputData(mesh_poly_data);
    writer->SetFileName(file_info.absoluteFilePath().toLocal8Bit().data());
    writer->Write();
  }
}

void TPPWidget::onSaveModifiedMeshes(const bool /*checked*/)
{
  QDir save_dir(QFileDialog::getExistingDirectory(this, "Save modified mesh(es)"));
  if (!save_dir.exists())
    return;

  saveModifiedMeshes(save_dir);
}

std::vector<ToolPaths> TPPWidget::editedToolPaths() const
{
  // Aucun repli sur tool_paths_. Ce membre peut porter un resultat PARTIEL, quand une region
  // ulterieure a fait echouer la planification, et le dock serait alors vide : le repli
  // ecrirait ce resultat partiel sans que la validation de la recette ne l'ait jamais vu.
  if (path_edit_ == nullptr)
  {
    return std::vector<ToolPaths>();
  }
  return nestToolPath(path_edit_->passes());
}

void TPPWidget::onPathEdited()
{
  const std::vector<ToolPaths> edited = editedToolPaths();

  renderer_->RemoveActor(tool_path_actor_);
  tool_path_actor_ = createToolPathActors(edited, tube_filter_->GetOutputPort());
  renderer_->AddActor(tool_path_actor_);
  tool_path_actor_->SetVisibility(ui_->action_show_modified_tool_path->isChecked());

  renderer_->RemoveActor(connected_path_actor_);
  connected_path_actor_ = createToolPathPolylineActor(edited, tube_filter_->GetOutputPort());
  renderer_->AddActor(connected_path_actor_);
  connected_path_actor_->SetVisibility(ui_->action_show_modified_tool_path_lines->isChecked());

  updateDiscCircles();
  updatePathHighlight();
}

void TPPWidget::updatePathHighlight()
{
  renderer_->RemoveActor(highlight_actor_);
  highlight_actor_ = vtkSmartPointer<vtkPropAssembly>::New();

  const int row = (path_edit_ == nullptr) ? -1 : path_edit_->selectedPass();
  if (row >= 0 && row < static_cast<int>(path_edit_->passes().size()))
  {
    highlight_actor_ = createPassHighlightActor(path_edit_->passes()[static_cast<std::size_t>(row)]);
  }

  renderer_->AddActor(highlight_actor_);
  render();
}

bool TPPWidget::confirmDiscardPathEdits()
{
  if (path_edit_ == nullptr || !path_edit_->isRetouched())
  {
    return true;
  }
  const QMessageBox::StandardButton answer =
      QMessageBox::question(this,
                            "Retouche en cours",
                            "Le dock Trajectoire porte une retouche. Une nouvelle planification la "
                            "perd, parce qu'elle renumerote les passes.\n\nPlanifier quand meme ?",
                            QMessageBox::Yes | QMessageBox::No,
                            QMessageBox::No);
  return answer == QMessageBox::Yes;
}

void TPPWidget::clearPlannedPaths()
{
  tool_paths_.clear();
  fragments_.clear();
  if (path_edit_ != nullptr)
  {
    path_edit_->setGenerated(std::vector<ToolPathSegment>());
  }
  renderer_->RemoveActor(disc_actor_);
  disc_actor_ = nullptr;

  renderer_->RemoveActor(mesh_fragment_actor_);
  renderer_->RemoveActor(unmodified_tool_path_actor_);
  renderer_->RemoveActor(unmodified_connected_path_actor_);
  mesh_fragment_actor_ = vtkSmartPointer<vtkPropAssembly>::New();
  unmodified_tool_path_actor_ = vtkSmartPointer<vtkPropAssembly>::New();
  unmodified_connected_path_actor_ = vtkSmartPointer<vtkPropAssembly>::New();
  renderer_->AddActor(mesh_fragment_actor_);
  renderer_->AddActor(unmodified_tool_path_actor_);
  renderer_->AddActor(unmodified_connected_path_actor_);
}

QString TPPWidget::editsFileName(const QString& tool_path_file)
{
  const QFileInfo info(tool_path_file);
  return info.dir().filePath(info.completeBaseName() + ".edits.yaml");
}

void TPPWidget::saveToolPaths(const QString& file)
{
  if (!reportRetouchFaults())
  {
    return;
  }

  // Open output file
  std::ofstream out(file.toStdString());
  if (!out)
  {
    QMessageBox::warning(this, "Save Error", "Failed to open file for writing: " + file);
    return;
  }

  // Write all tool paths at once using YAML serialization
  out << YAML::Node(editedToolPaths());

  savePathEdits(file);
}

bool TPPWidget::reportRetouchFaults()
{
  // C'est le dock qui fait foi, pas tool_paths_ : il n'est alimente que par une planification
  // menee jusqu'au bout, et lui seul porte la recette que la validation controle.
  if (path_edit_ == nullptr || path_edit_->generated().empty())
  {
    QMessageBox::warning(this, "Error", "No tool paths found; please plan a tool path first.");
    return false;
  }

  const std::vector<std::string> faults = path_edit_->faults();
  if (faults.empty())
  {
    return true;
  }

  QString message = "La retouche ne peut pas etre livree :";
  for (std::size_t i = 0; i < faults.size(); ++i)
  {
    message += "\n- " + QString::fromStdString(faults[i]);
  }
  QMessageBox::warning(this, "Retouche invalide", message);
  return false;
}

void TPPWidget::savePathEdits(const QString& tool_path_file)
{
  // La recette part a cote du chemin de poses : celui-ci reste une donnee derivee, qui se
  // regenere, et ce que le mainteneur a retouche a la main reste lisible et rejouable.
  RecipeFile content;
  content.recipe = path_edit_->recipe();
  content.source = fingerprintSegments(path_edit_->generated());
  content.moves = path_edit_->moves();

  const QString file = editsFileName(tool_path_file);
  try
  {
    writeRecipeFile(file.toStdString(), content);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this,
                         "Retouche non enregistree",
                         QString("Le chemin est ecrit, mais la retouche n'a pas pu l'etre dans %1 : %2")
                             .arg(file)
                             .arg(QString::fromStdString(ex.what())));
  }
}
void TPPWidget::loadToolPaths(const QString& file)
{
  std::vector<ToolPaths> loaded;
  try
  {
    loaded = YAML::LoadFile(file.toStdString()).as<std::vector<ToolPaths>>();
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error("Le fichier " + file.toStdString() + " ne se lit pas comme un chemin d'outil : " +
                             ex.what());
  }
  // Le chemin charge devient la generation : les fragments de la vue n'ont plus de sens, le dock
  // Trajectoire repart d'une recette identite sur ces passes
  clearPlannedPaths();
  tool_paths_ = loaded;
  showPlannedPaths(loaded);
  std::size_t pose_count = 0;
  for (const ToolPathSegment& pass : path_edit_->passes())
  {
    pose_count += pass.size();
  }
  statusBar()->showMessage(QString("Chemin charge : %1 passe(s), %2 poses. Retouchez, puis Save tool paths.")
                               .arg(path_edit_->passes().size())
                               .arg(pose_count));
}

void TPPWidget::onLoadToolPaths()
{
  if (!confirmDiscardPathEdits())
  {
    return;
  }
  const QString file = QFileDialog::getOpenFileName(this, "Charger un chemin", "", "YAML files (*.yaml)");
  if (file.isEmpty())
  {
    return;
  }
  try
  {
    loadToolPaths(file);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Chemin non charge", QString::fromStdString(ex.what()));
  }
}

void TPPWidget::onSaveToolPaths(const bool /*checked*/)
{
  QString file = QFileDialog::getSaveFileName(this, "Save trajectory", "", "YAML files (*.yaml)");
  if (file.isEmpty())
    return;

  if (!file.endsWith(".yaml"))
    file = file.append(".yaml");

  saveToolPaths(file);
}

}  // namespace noether
