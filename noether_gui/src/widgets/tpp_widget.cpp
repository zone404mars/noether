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

/** @brief Couleur des faces planifiees que le disque n'atteint pas le long du chemin livre */
const unsigned char kUncoveredRgb[3] = { 40, 40, 40 };

/** @brief Rayon d'outil a l'ouverture, en millimetres : le disque de 125 mm de la cellule */
const double kDefaultToolRadiusMm = 62.5;

/** @brief Pas du trace manuel a l'ouverture, en millimetres */
const double kDefaultTraceStepMm = 20.0;

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

/** @brief Donnee du selecteur de direction : la surface suit la configuration du pipeline */
const int kDirectionFromPipeline = -1;

/** @brief Donnee du selecteur de direction : direction tapee a la main */
const int kDirectionCustom = 3;

/**
 * @brief Norme en dessous de laquelle une direction de passe ne designe plus rien
 * @details Le generateur normalise la direction qu'on lui donne : un vecteur nul en ferait sortir
 * des NaN, et le chemin produit serait vide ou aberrant sans qu'aucune erreur ne soit levee.
 */
const double kMinDirectionNorm = 1e-6;

/** @brief Les trois axes du repere du maillage, dans l'ordre du selecteur */
const Eigen::Vector3d kAxisDirections[3] = { Eigen::Vector3d::UnitX(),
                                             Eigen::Vector3d::UnitY(),
                                             Eigen::Vector3d::UnitZ() };

/// @brief Entree du selecteur qui correspond a une direction.
/// @param direction Direction de la surface, ignoree quand elle n'en porte pas
/// @param has_own La surface porte une direction a elle
/// @return L'entree a afficher : celle du pipeline, celle d'un axe, ou celle du choix libre
int directionEntry(const Eigen::Vector3d& direction, const bool has_own)
{
  if (!has_own)
  {
    return kDirectionFromPipeline;
  }
  for (int axis = 0; axis < 3; ++axis)
  {
    if (direction.isApprox(kAxisDirections[axis]))
    {
      return axis;
    }
  }
  return kDirectionCustom;
}

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
    if (endpoint_pick_ != EndpointPick::None)
    {
      onEndpointPicked(pickFace(x, y));
      return;
    }
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
    return movingPoints() || tool_ == SelectionTool::Brush || tool_ == SelectionTool::Eraser ||
           tool_ == SelectionTool::Trace;
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
  tool_combo_box_->addItem("Trace manuel (drag)", static_cast<int>(SelectionTool::Trace));
  tool_combo_box_->setToolTip(
      "Selection tool. Shift while dragging, or while closing an outline, erases instead of "
      "selecting; Alt while dragging orbits the camera without leaving the brush.\n"
      "Polygon: click to place vertices, then click the first one, press Enter or double-click to "
      "close. Backspace removes the last vertex, Escape cancels the outline.\n"
      "Trace manuel : selectionnez une surface dans la liste ; le premier clic est le depart, chaque "
      "tirage ajoute un segment droit, un point tous les X mm. Retour arriere retire le dernier sommet, "
      "Echap efface le trace.");
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
    updateCoverage();
    updateDiscCircles();
    updateTraceMarkers();
    render();
  });
  connect(tool_radius_enabled_check_box_, &QCheckBox::toggled, this, [this](bool enabled) {
    tool_radius_spin_box_->setEnabled(enabled);
    updateCoverage();
    updateDiscCircles();
    updateTraceMarkers();
    render();
  });

  buildTraceControls();

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
    onSurfaceSelected();
    updateTraceMarkers();
    render();
  });

  auto* surfaces_panel = new QWidget(this);
  auto* surfaces_layout = new QVBoxLayout(surfaces_panel);
  surfaces_layout->addWidget(surface_list_);
  surfaces_layout->addWidget(buildSurfaceDirectionPanel());
  surfaces_layout->addWidget(buildSurfaceEndpointPanel());
  surfaces_layout->addWidget(remove_surface_button_);

  auto* surfaces_dock = new QDockWidget("Surfaces", this);
  surfaces_dock->setWidget(surfaces_panel);
  addDockWidget(Qt::RightDockWidgetArea, surfaces_dock);

  buildPathEditDock();

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

void TPPWidget::buildPathEditDock()
{
  // Retouching bears on the result of the planning, not on the selection, so it lives in its own
  // dock. Planning feeds it; from then on it owns what the "modified" path actors show.
  path_edit_ = new PathEditWidget(kMinPassPoses, this);
  path_edit_->on_changed = [this]() { onPathEdited(); };
  path_edit_->on_selection_changed = [this]() { updatePathHighlight(); };

  load_edits_button_ = new QPushButton("Charger une retouche...", this);
  load_edits_button_->setToolTip("Relit une retouche enregistree.\n"
                                 "Elle est refusee si elle a ete ecrite sur une autre generation.");
  connect(load_edits_button_, &QPushButton::clicked, this, &TPPWidget::onLoadPathEdits);

  buildCoverageControls();

  auto* panel = new QWidget(this);
  auto* layout = new QVBoxLayout(panel);
  layout->addWidget(path_edit_);
  layout->addWidget(auto_chain_check_box_);
  layout->addWidget(move_points_button_);
  layout->addWidget(validate_layout_button_);
  layout->addWidget(show_discs_check_box_);
  layout->addWidget(coverage_label_);
  layout->addWidget(load_edits_button_);

  auto* dock = new QDockWidget("Trajectoire", this);
  dock->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, dock);
}

void TPPWidget::buildCoverageControls()
{
  auto_chain_check_box_ = new QCheckBox("Enchainer automatiquement apres generation", this);
  auto_chain_check_box_->setChecked(false);
  auto_chain_check_box_->setToolTip("Des la generation, soude toutes les passes en une seule a partir de la "
                                    "premiere, du plus proche au plus proche. Annuler defait l'enchainement.");

  move_points_button_ = new QPushButton("Deplacer des points (drag)", this);
  move_points_button_->setCheckable(true);
  move_points_button_->setToolTip("Enfonce, le tirage dans la vue saisit le point du chemin le plus proche du "
                                  "clic et le pose ou la souris relache, sur la piece. Annuler defait un tirage. "
                                  "Les points deplaces sont enregistres avec la retouche.");
  connect(move_points_button_, &QPushButton::toggled, this, [this](bool checked) {
    grab_active_ = false;
    updateGrabMarker();
    updateTraceMarkers();
    statusBar()->showMessage(checked ? "Deplacer des points : le tirage dans la vue saisit un point du chemin. "
                                       "Relachez le bouton, ou choisissez un outil, pour tracer ou selectionner." :
                                       "Deplacer des points : termine.");
    render();
  });

  validate_layout_button_ = new QPushButton("Valider la nouvelle disposition", this);
  validate_layout_button_->setToolTip("Sort du mode deplacement. Les points deplaces sont ceux du chemin livre : "
                                      "Save tool paths les ecrit tels quels.");
  connect(validate_layout_button_, &QPushButton::clicked, this, [this](bool) { validateLayout(); });

  show_discs_check_box_ = new QCheckBox("Cercle du disque autour de chaque point", this);
  show_discs_check_box_->setChecked(true);
  show_discs_check_box_->setToolTip("Dessine, apres generation, l'empreinte du disque (rayon d'outil) autour de "
                                    "chaque point de contact du chemin livre, pour voir un point trop pres d'un bord.");
  connect(show_discs_check_box_, &QCheckBox::toggled, this, [this](bool) {
    updateDiscCircles();
    updateTraceMarkers();
    render();
  });

  coverage_label_ = new QLabel(this);
  coverage_label_->setWordWrap(true);
  coverage_label_->setToolTip("Part des faces planifiees dont le centroide passe a moins du rayon d'outil "
                              "d'un troncon du chemin livre. Les faces hors portee sont en noir dans la vue.");
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

int TPPWidget::nearestKeptFace(const int surface_id, const Eigen::Vector3d& point, const std::vector<char>& kept) const
{
  int nearest = -1;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t f = 0; f < face_region_.size(); ++f)
  {
    if (face_region_[f] != surface_id || f >= kept.size() || kept[f] == 0)
    {
      continue;
    }
    const double distance = (face_centroids_[f] - point).squaredNorm();
    if (distance < best)
    {
      best = distance;
      nearest = static_cast<int>(f);
    }
  }
  return nearest;
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
      if (f < face_uncovered_.size() && face_uncovered_[f])
      {
        // Planned, but the disc never reaches it along the delivered path
        cell_colors_->SetTypedTuple(i, kUncoveredRgb);
      }
      else if (f < kept.size() && kept[f])
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
  if (tool_ == SelectionTool::Trace)
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
    trace_step_spin_box_->setEnabled(tool_ == SelectionTool::Trace);
  }
}

void TPPWidget::buildTraceControls()
{
  trace_step_spin_box_ = new QDoubleSpinBox(this);
  trace_step_spin_box_->setRange(0.1, 10000.0);
  trace_step_spin_box_->setDecimals(1);
  trace_step_spin_box_->setSingleStep(5.0);
  trace_step_spin_box_->setValue(kDefaultTraceStepMm);
  trace_step_spin_box_->setSuffix(" mm");
  trace_step_spin_box_->setToolTip("Pas du trace manuel : un point du chemin tous les X mm le long de chaque segment.");
  ui_->toolBar->addWidget(new QLabel("Pas du trace:", this));
  ui_->toolBar->addWidget(trace_step_spin_box_);
  connect(trace_step_spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
    updateTraceMarkers();
    render();
  });
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

QWidget* TPPWidget::buildSurfaceDirectionPanel()
{
  // La direction de passe est une propriete DE LA SURFACE, pas du pipeline : une tole se ponce
  // dans le sens de sa longueur, celle d'a cote en travers. Le pipeline n'en porte plus qu'un
  // defaut, et chaque surface peut s'en ecarter.
  populateDirectionSelector();

  auto* panel = new QWidget(this);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(surface_direction_combo_box_);

  layout->addLayout(buildDirectionComponents());

  connect(surface_direction_combo_box_,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          this,
          [this](int) { onSurfaceDirectionEdited(); });

  return panel;
}

void TPPWidget::populateDirectionSelector()
{
  surface_direction_combo_box_ = new QComboBox(this);
  surface_direction_combo_box_->addItem("Direction : celle du pipeline", kDirectionFromPipeline);
  surface_direction_combo_box_->addItem("Direction : X de la piece", 0);
  surface_direction_combo_box_->addItem("Direction : Y de la piece", 1);
  surface_direction_combo_box_->addItem("Direction : Z de la piece", 2);
  surface_direction_combo_box_->addItem("Direction : au choix", kDirectionCustom);
  surface_direction_combo_box_->setToolTip("Sens des lignes de balayage sur la surface selectionnee,\n"
                                           "exprime dans le repere du maillage, celui des axes affiches.");
}

QLayout* TPPWidget::buildDirectionComponents()
{
  auto* components = new QHBoxLayout();
  QDoubleSpinBox** boxes[3] = { &direction_x_spin_box_, &direction_y_spin_box_, &direction_z_spin_box_ };
  const char* labels[3] = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i)
  {
    *boxes[i] = new QDoubleSpinBox(this);
    (*boxes[i])->setRange(-1.0, 1.0);
    (*boxes[i])->setDecimals(3);
    (*boxes[i])->setSingleStep(0.1);
    components->addWidget(new QLabel(labels[i], this));
    components->addWidget(*boxes[i]);
    connect(*boxes[i],
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this](double) { onSurfaceDirectionEdited(); });
  }
  return components;
}

void TPPWidget::showDirection(const Eigen::Vector3d& direction, const int entry)
{
  // Les controles refletent la surface choisie : les regler ne doit rien reecrire dans la
  // surface precedente, d'ou le blocage des signaux le temps de la mise a jour.
  const bool blocked = surface_direction_combo_box_->blockSignals(true);
  surface_direction_combo_box_->setCurrentIndex(surface_direction_combo_box_->findData(entry));
  surface_direction_combo_box_->blockSignals(blocked);

  QDoubleSpinBox* boxes[3] = { direction_x_spin_box_, direction_y_spin_box_, direction_z_spin_box_ };
  for (int i = 0; i < 3; ++i)
  {
    boxes[i]->blockSignals(true);
    boxes[i]->setValue(direction[i]);
    boxes[i]->blockSignals(false);
    boxes[i]->setEnabled(entry == kDirectionCustom);
  }
}

void TPPWidget::onSurfaceSelected()
{
  QListWidgetItem* item = surface_list_->currentItem();
  if (item == nullptr)
  {
    return;
  }

  Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
  const bool has_own = surfaceDirection(item->data(Qt::UserRole).toInt(), direction);
  showDirection(direction, directionEntry(direction, has_own));
}

void TPPWidget::storeSurfaceDirection(const int surface_id, const int entry)
{
  if (entry == kDirectionFromPipeline)
  {
    surface_directions_.erase(surface_id);
  }
  else if (entry == kDirectionCustom)
  {
    surface_directions_[surface_id] =
        Eigen::Vector3d(direction_x_spin_box_->value(), direction_y_spin_box_->value(), direction_z_spin_box_->value());
  }
  else
  {
    surface_directions_[surface_id] = kAxisDirections[entry];
  }
}

QWidget* TPPWidget::buildSurfaceEndpointPanel()
{
  // Le depart et l'arrivee sont des proprietes DE LA SURFACE, comme sa direction : la ou l'outil
  // pose et la ou il quitte, pour que la surface suivante s'enchaine sans decoller
  pick_start_button_ = new QPushButton("Choisir le depart (clic)", this);
  pick_end_button_ = new QPushButton("Choisir l'arrivee (clic)", this);
  clear_endpoints_button_ = new QPushButton("Effacer depart et arrivee", this);
  pick_start_button_->setToolTip("Le prochain clic sur le maillage fixe le point ou le chemin de la surface "
                                 "selectionnee commence. Il faut aussi une arrivee pour que cela s'applique.");
  pick_end_button_->setToolTip("Le prochain clic sur le maillage fixe le point ou le chemin finit. L'arrivee "
                               "n'est pas toujours atteignable exactement : l'ecart est affiche apres generation.");
  connectEndpointButtons();

  auto* panel = new QWidget(this);
  auto* layout = new QHBoxLayout(panel);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(pick_start_button_);
  layout->addWidget(pick_end_button_);
  layout->addWidget(clear_endpoints_button_);
  return panel;
}

void TPPWidget::connectEndpointButtons()
{
  connect(pick_start_button_, &QPushButton::clicked, this, [this](bool) {
    endpoint_pick_ = EndpointPick::Start;
    statusBar()->showMessage("Cliquez la face ou le chemin de la surface selectionnee commence.");
  });
  connect(pick_end_button_, &QPushButton::clicked, this, [this](bool) {
    endpoint_pick_ = EndpointPick::End;
    statusBar()->showMessage("Cliquez la face ou le chemin de la surface selectionnee finit.");
  });
  connect(clear_endpoints_button_, &QPushButton::clicked, this, [this](bool) { clearSelectedSurfaceEndpoints(); });
}

void TPPWidget::clearSelectedSurfaceEndpoints()
{
  QListWidgetItem* item = surface_list_->currentItem();
  if (item == nullptr)
  {
    return;
  }
  const int surface_id = item->data(Qt::UserRole).toInt();
  surface_starts_.erase(surface_id);
  surface_ends_.erase(surface_id);
  relabelSurface(item);
  updateEndpointMarkers();
  render();
}

/// @brief Phrase du bandeau d'etat apres un depart ou une arrivee fixe.
/// @param is_start Vrai pour le depart, faux pour l'arrivee
/// @param shift_mm Distance entre la face cliquee et la face retenue, en millimetres
static QString endpointMessage(const bool is_start, const double shift_mm)
{
  QString message = is_start ? "Depart fixe." : "Arrivee fixee.";
  if (shift_mm > 0.5)
  {
    message += QString(" Deplace de %1 mm : le disque deborderait de la surface a l'endroit clique.")
                   .arg(shift_mm, 0, 'f', 0);
  }
  return message;
}

void TPPWidget::onEndpointPicked(const int face)
{
  const EndpointPick target = endpoint_pick_;
  endpoint_pick_ = EndpointPick::None;
  QListWidgetItem* item = surface_list_->currentItem();
  if (face < 0 || item == nullptr || static_cast<std::size_t>(face) >= face_centroids_.size())
  {
    statusBar()->showMessage("Aucune face sous le clic, ou aucune surface selectionnee : rien n'est fixe.");
    return;
  }
  // Le disque doit tenir sur la surface au point demande : la face cliquee est remplacee par la
  // face planifiable (non erodee) la plus proche, et l'operateur lit de combien il a ete deplace
  const int surface_id = item->data(Qt::UserRole).toInt();
  const Eigen::Vector3d clicked = face_centroids_[static_cast<std::size_t>(face)];
  const int reachable = nearestKeptFace(surface_id, clicked, erodeSelection());
  if (reachable < 0)
  {
    statusBar()->showMessage("Le disque ne tient sur aucune face de cette surface : rien n'est fixe.");
    return;
  }
  std::map<int, Eigen::Vector3d>& store = (target == EndpointPick::Start) ? surface_starts_ : surface_ends_;
  store[surface_id] = face_centroids_[static_cast<std::size_t>(reachable)];
  const double shift_mm = (store[surface_id] - clicked).norm() * 1000.0;
  statusBar()->showMessage(endpointMessage(target == EndpointPick::Start, shift_mm));
  relabelSurface(item);
  updateEndpointMarkers();
  render();
}

bool TPPWidget::surfaceEndpoints(const int surface_id, Eigen::Vector3d& start, Eigen::Vector3d& end) const
{
  const auto found_start = surface_starts_.find(surface_id);
  const auto found_end = surface_ends_.find(surface_id);
  if (found_start == surface_starts_.end() || found_end == surface_ends_.end())
  {
    return false;
  }
  start = found_start->second;
  end = found_end->second;
  return true;
}

vtkSmartPointer<vtkPolyData> TPPWidget::endpointMarkerPoints() const
{
  // Un point par extremite fixee : vert au depart, rouge a l'arrivee
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto vertices = vtkSmartPointer<vtkCellArray>::New();
  auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  const unsigned char green[3] = { 40, 200, 60 };
  const unsigned char red[3] = { 220, 40, 40 };
  for (const std::map<int, Eigen::Vector3d>* store : { &surface_starts_, &surface_ends_ })
  {
    for (const auto& entry : *store)
    {
      const vtkIdType id = points->InsertNextPoint(entry.second.x(), entry.second.y(), entry.second.z());
      vertices->InsertNextCell(1, &id);
      colors->InsertNextTypedTuple(store == &surface_starts_ ? green : red);
    }
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(vertices);
  poly->GetPointData()->SetScalars(colors);
  return poly;
}

void TPPWidget::updateEndpointMarkers()
{
  // Rendus comme des spheres, pour se voir a toute distance de camera
  renderer_->RemoveActor(endpoint_actor_);
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(endpointMarkerPoints());
  endpoint_actor_ = vtkSmartPointer<vtkActor>::New();
  endpoint_actor_->SetMapper(mapper);
  endpoint_actor_->GetProperty()->SetPointSize(18.0f);
  endpoint_actor_->GetProperty()->SetRenderPointsAsSpheres(true);
  renderer_->AddActor(endpoint_actor_);
}

QString TPPWidget::describeEndpointGaps(const int surface_id, const ToolPaths& delivered) const
{
  Eigen::Vector3d start;
  Eigen::Vector3d end;
  if (!surfaceEndpoints(surface_id, start, end) || delivered.empty() || delivered.front().empty())
  {
    return QString();
  }
  // La premiere et la derniere poses sont l'approche et le retrait, hors contact : on lit celles d'a cote
  const ToolPathSegment& first = delivered.front().front();
  const ToolPathSegment& last = delivered.back().back();
  if (first.size() <= kSkirtPoses || last.size() <= kSkirtPoses)
  {
    return QString();
  }
  const double start_gap = (first[kSkirtPoses].translation() - start).norm();
  const double end_gap = (last[last.size() - 1 - kSkirtPoses].translation() - end).norm();
  return QString("Surface %1 : depart a %2 mm du point demande, arrivee a %3 mm.  ")
      .arg(surface_id + 1)
      .arg(start_gap * 1000.0, 0, 'f', 0)
      .arg(end_gap * 1000.0, 0, 'f', 0);
}

void TPPWidget::onSurfaceDirectionEdited()
{
  QListWidgetItem* item = surface_list_->currentItem();
  if (item == nullptr)
  {
    return;
  }

  const int surface_id = item->data(Qt::UserRole).toInt();
  const int entry = surface_direction_combo_box_->currentData().toInt();
  storeSurfaceDirection(surface_id, entry);

  // Les controles suivent ce qui vient d'etre enregistre : choisir un axe remplit les
  // composantes et les grise, revenir au pipeline les grise aussi.
  Eigen::Vector3d stored = Eigen::Vector3d::UnitX();
  showDirection(surfaceDirection(surface_id, stored) ? stored : Eigen::Vector3d::UnitX(), entry);
  relabelSurface(item);
}

bool TPPWidget::surfaceDirection(const int surface_id, Eigen::Vector3d& direction) const
{
  const std::map<int, Eigen::Vector3d>::const_iterator found = surface_directions_.find(surface_id);
  if (found == surface_directions_.end())
  {
    return false;
  }
  direction = found->second;
  return true;
}

void TPPWidget::relabelSurface(QListWidgetItem* item) const
{
  const int surface_id = item->data(Qt::UserRole).toInt();
  const int face_count = item->data(Qt::UserRole + 1).toInt();

  Eigen::Vector3d direction;
  QString reading = "pipeline";
  if (surfaceDirection(surface_id, direction))
  {
    reading = "au choix";
    const char* names[3] = { "X", "Y", "Z" };
    for (int axis = 0; axis < 3; ++axis)
    {
      if (direction.isApprox(kAxisDirections[axis]))
      {
        reading = names[axis];
      }
    }
  }
  Eigen::Vector3d start;
  Eigen::Vector3d end;
  const QString endpoints = surfaceEndpoints(surface_id, start, end) ? " - depart et arrivee fixes" : "";
  item->setText(QString("surface %1 - %2 faces - direction %3%4%5")
                    .arg(surface_id + 1)
                    .arg(face_count)
                    .arg(reading)
                    .arg(endpoints)
                    .arg(describeTrace(surface_id)));
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
  surface_directions_.erase(surface_id);
  surface_starts_.erase(surface_id);
  surface_ends_.erase(surface_id);
  surface_traces_.erase(surface_id);
  trace_preview_active_ = false;
  updateEndpointMarkers();
  delete surface_list_->takeItem(surface_list_->row(item));

  updateTraceMarkers();
  updateSelectionColors();
  render();
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

  switch (tool_)
  {
    case SelectionTool::Brush:
    case SelectionTool::Eraser:
      paintBrushDab(x, y, erase);
      updateSelectionColors();
      render();
      break;
    case SelectionTool::Trace:
      onTraceDragStart(x, y);
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
  switch (tool_)
  {
    case SelectionTool::Brush:
    case SelectionTool::Eraser:
      paintBrushAlongSegment(x, y, erase);
      updateSelectionColors();
      render();
      break;
    case SelectionTool::Trace:
      onTraceDragMove(x, y);
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
  else if (tool_ == SelectionTool::Trace)
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
  if (!locatePose(path_edit_->movedGenerated(), clicked, kGrabToleranceM, grabbed_))
  {
    statusBar()->showMessage("Aucun point du chemin a moins de 50 mm du clic.");
    return;
  }
  grab_active_ = true;
  grab_origin_ = grabbed_.pose.translation();
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
  // Un seul recalcul du chemin, de la couverture et des cercles, au relachement
  path_edit_->movePose(grabbed_);
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
  if (surface_id < 0 || trace_step_spin_box_ == nullptr || tool_ != SelectionTool::Trace || movingPoints())
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
  if (radius_enabled && show_discs_check_box_->isChecked())
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
  if (path_edit_ == nullptr || !radius_enabled || !show_discs_check_box_->isChecked())
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
  surface_directions_.clear();
  surface_starts_.clear();
  surface_ends_.clear();
  surface_traces_.clear();
  trace_preview_active_ = false;
  updateEndpointMarkers();
  updateTraceMarkers();
  next_region_id_ = 0;
  updateSelectionColors();
  render();
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
  const ToolPathModifier::ConstPtr modifier = pipeline_widget_->createToolPathModifierWithoutOrganization();
  const ToolPaths path = tracedToolPaths(surface_id);
  fragments_.push_back(extractSubMeshFromFaces(mesh_, faces));
  unmodified.push_back(path);
  try
  {
    tool_paths_.push_back(modifier->modify(path));
  }
  catch (const std::exception&)
  {
    std::stringstream ss;
    ss << "Erreur des modificateurs de chemin sur le trace de la surface " << (surface_id + 1) << ".";
    std::throw_with_nested(std::runtime_error(ss.str()));
  }
}

bool TPPWidget::anySurfaceNeedsPlanner(const std::map<int, std::vector<int>>& region_faces) const
{
  for (const auto& entry : region_faces)
  {
    if (!surfaceHasTrace(entry.first))
    {
      return true;
    }
  }
  return false;
}

ToolPathPlannerPipeline TPPWidget::surfacePipeline(const bool has_direction,
                                                   const bool has_endpoints,
                                                   const Eigen::Vector3d& direction,
                                                   const Eigen::Vector3d& start,
                                                   const Eigen::Vector3d& end) const
{
  if (has_endpoints)
  {
    return pipeline_widget_->createPipeline(has_direction ? &direction : nullptr, start, end);
  }
  return pipeline_widget_->createPipeline(direction);
}

void TPPWidget::plan()
{
  try
  {
    // Une nouvelle generation renumerote les passes : la retouche en cours ne designerait
    // plus rien et est donc perdue. La perdre en silence sur un clic est un piege.
    if (!confirmDiscardPathEdits())
    {
      return;
    }

    if (std::any_of(pending_faces_.begin(), pending_faces_.end(), [](char p) { return p != 0; }))
    {
      QMessageBox::warning(this,
                           "Selection not added",
                           "The orange work selection is not part of any surface. Add it to the "
                           "list, or erase it, then Plan.");
      return;
    }

    // Collect the added surfaces as lists of face indices, keyed (and ordered) by surface id,
    // keeping only the faces that survive erosion by the tool radius
    const std::vector<char> kept = erodeSelection();
    std::map<int, std::vector<int>> region_faces;
    bool any_selected = false;
    for (std::size_t f = 0; f < face_region_.size(); ++f)
    {
      const int region = face_region_[f];
      if (region < 0)
        continue;
      any_selected = true;
      // Une surface tracee a la main est planifiee par son trace : ses faces servent au dessin
      if ((f < kept.size() && kept[f]) || surfaceHasTrace(region))
      {
        region_faces[region].push_back(static_cast<int>(f));
      }
    }

    if (region_faces.empty())
    {
      const QString msg = any_selected ?
                              "All selected regions are narrower than the tool radius, so they were "
                              "fully eroded (a disc of that radius would overhang). Reduce the tool "
                              "radius or select wider regions." :
                              "No surface in the list. Select faces in the viewer, then Add surface.";
      QMessageBox::warning(this, "Nothing to plan", msg);
      return;
    }

    // Construit avant la boucle pour que la configuration soit refusee tout de suite, et non
    // apres avoir fait attendre sur la premiere surface. Pas construit du tout quand toutes les
    // surfaces sont tracees a la main : le planificateur ne sert alors a rien, et sa
    // configuration peut rester incomplete.
    std::unique_ptr<ToolPathPlannerPipeline> default_pipeline;
    if (anySurfaceNeedsPlanner(region_faces))
    {
      default_pipeline.reset(new ToolPathPlannerPipeline(pipeline_widget_->createPipeline()));
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);

    tool_paths_.clear();
    std::vector<ToolPaths> unmodified_tool_paths;
    fragments_.clear();

    std::size_t skipped_regions = 0;
    QString endpoint_report;
    std::size_t region_index = 0;
    for (const auto& entry : region_faces)
    {
      // Une surface qui porte sa propre direction de passe est planifiee par un pipeline bati
      // pour elle. Les autres partagent celui de la configuration, qui ne se rebatit pas.
      // Une surface tracee a la main ne passe pas par le planificateur : le trace fixe lui-meme
      // le depart, l'arrivee et le sens de marche
      if (surfaceHasTrace(entry.first))
      {
        planTracedSurface(entry.first, entry.second, unmodified_tool_paths);
        ++region_index;
        continue;
      }
      std::unique_ptr<ToolPathPlannerPipeline> own_pipeline;
      Eigen::Vector3d direction;
      Eigen::Vector3d start;
      Eigen::Vector3d end;
      const bool has_direction = surfaceDirection(entry.first, direction);
      const bool has_endpoints = surfaceEndpoints(entry.first, start, end);
      if (has_direction && direction.norm() < kMinDirectionNorm)
      {
        std::stringstream ss;
        ss << "La surface " << (entry.first + 1) << " porte une direction de passe nulle.";
        throw std::runtime_error(ss.str());
      }
      if (has_direction || has_endpoints)
      {
        try
        {
          // Depart et arrivee fixes : l'organisateur de la configuration cede la place au notre
          own_pipeline.reset(
              new ToolPathPlannerPipeline(surfacePipeline(has_direction, has_endpoints, direction, start, end)));
        }
        catch (const std::exception&)
        {
          std::stringstream ss;
          ss << "Error building the pipeline for the direction of selected region " << region_index << ".";
          std::throw_with_nested(std::runtime_error(ss.str()));
        }
      }
      const ToolPathPlannerPipeline& pipeline = own_pipeline ? *own_pipeline : *default_pipeline;

      // Extract the selected region as a standalone submesh
      const pcl::PolygonMesh region_mesh = extractSubMeshFromFaces(mesh_, entry.second);

      // Apply the mesh modifier stage (e.g. NormalsFromMeshFaces); it may yield multiple meshes
      std::vector<pcl::PolygonMesh> modified_meshes;
      try
      {
        modified_meshes = pipeline.mesh_modifier->modify(region_mesh);
      }
      catch (const std::exception&)
      {
        std::stringstream ss;
        ss << "Error invoking mesh modifier on selected region " << region_index << ".";
        std::throw_with_nested(std::runtime_error(ss.str()));
      }

      for (const pcl::PolygonMesh& mesh : modified_meshes)
      {
        // Plan the tool path
        ToolPaths path;
        try
        {
          path = pipeline.planner->plan(mesh);
        }
        catch (const std::exception&)
        {
          std::stringstream ss;
          ss << "Error invoking tool path planner on selected region " << region_index << ".";
          std::throw_with_nested(std::runtime_error(ss.str()));
        }

        // Skip regions the planner could not turn into a usable path (e.g. a tiny stray selection
        // or a region smaller than the raster spacing). Applying tool path modifiers to an empty
        // path would otherwise throw (out-of-range access).
        bool has_waypoints = false;
        for (const ToolPath& tp : path)
        {
          for (const ToolPathSegment& seg : tp)
          {
            if (!seg.empty())
            {
              has_waypoints = true;
              break;
            }
          }
          if (has_waypoints)
            break;
        }
        if (!has_waypoints)
        {
          ++skipped_regions;
          continue;
        }

        fragments_.push_back(mesh);
        unmodified_tool_paths.push_back(path);

        try
        {
          tool_paths_.push_back(pipeline.tool_path_modifier->modify(path));
          if (has_endpoints)
          {
            endpoint_report += describeEndpointGaps(entry.first, tool_paths_.back());
          }
        }
        catch (const std::exception&)
        {
          std::stringstream ss;
          ss << "Error invoking tool path modifier for selected region " << region_index << ".";
          std::throw_with_nested(std::runtime_error(ss.str()));
        }
      }

      ++region_index;
    }

    // Show the planned submeshes as colored fragments (togglable via "Show modified mesh" and saved
    // by "Save modified mesh")
    renderer_->RemoveActor(mesh_fragment_actor_);
    mesh_fragment_actor_ = createMeshActors(fragments_);
    renderer_->AddActor(mesh_fragment_actor_);
    mesh_fragment_actor_->SetVisibility(ui_->action_show_modified_mesh->isChecked());

    QApplication::restoreOverrideCursor();

    // Render the unmodified tool paths
    {
      renderer_->RemoveActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_ = createToolPathActors(unmodified_tool_paths, tube_filter_->GetOutputPort());
      renderer_->AddActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path->isChecked());
    }

    // Render the unmodified connected paths
    {
      renderer_->RemoveActor(unmodified_connected_path_actor_);
      unmodified_connected_path_actor_ =
          createToolPathPolylineActor(unmodified_tool_paths, tube_filter_->GetOutputPort());
      renderer_->AddActor(unmodified_connected_path_actor_);
      unmodified_connected_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path_lines->isChecked());
    }

    // Hand the planned passes to the Trajectory dock. Its recipe starts as the identity, so what
    // it delivers is the generation itself until the operator retouches it. Drawing the modified
    // tool path and the connected path is its callback's job from here on.
    path_edit_->setGenerated(flattenToolPaths(tool_paths_));
    if (!endpoint_report.isEmpty())
    {
      statusBar()->showMessage(endpoint_report.trimmed());
    }
    if (auto_chain_check_box_->isChecked() && path_edit_->passes().size() > 1)
    {
      path_edit_->selectPass(0);
      path_edit_->chainFromSelected();
    }
    updateCoverage();

    // Render
    render();

    // Let the user know if some selected regions produced no usable tool path
    if (skipped_regions > 0)
    {
      QMessageBox::information(
          this,
          "Some regions skipped",
          QString("%1 selected region(s) produced no tool path and were skipped.\n\nThis usually means "
                  "the region is too small for the planner's raster spacing, or a stray single-face "
                  "selection. Adjust the region or the planner settings, or clear and reselect.")
              .arg(skipped_regions));
    }
  }
  catch (const std::exception& ex)
  {
    QApplication::restoreOverrideCursor();

    // Les regions deja planifiees sont dans tool_paths_, celles d'apres non. Garder ce
    // resultat partiel laisserait afficher, et enregistrer, un chemin ampute en silence.
    clearPlannedPaths();

    std::stringstream ss;
    printException(ex, ss);
    QMessageBox::warning(this, "Tool Path Planning Error", QString::fromStdString(ss.str()));
  }
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

  updateCoverage();
  updateDiscCircles();
  updatePathHighlight();
}

/** @brief Phrase affichee sous le dock pour une mesure de couverture */
QString describeCoverage(const CoverageResult& coverage)
{
  if (coverage.uncovered_faces.empty())
  {
    return QString("Couverture du disque : 100 % des %1 faces planifiees.").arg(coverage.selected_count);
  }
  return QString("Couverture du disque : %1 % ; %2 faces sur %3 hors portee, en noir dans la vue.")
      .arg(coverage.ratio * 100.0, 0, 'f', 1)
      .arg(coverage.uncovered_faces.size())
      .arg(coverage.selected_count);
}

std::vector<int> TPPWidget::plannedFaces() const
{
  // Les faces planifiees sont celles des surfaces, moins ce que l'erosion par le rayon a retire
  const std::vector<char> kept = erodeSelection();
  std::vector<int> planned;
  for (std::size_t f = 0; f < face_region_.size(); ++f)
  {
    if (face_region_[f] >= 0 && f < kept.size() && kept[f])
    {
      planned.push_back(static_cast<int>(f));
    }
  }
  return planned;
}

void TPPWidget::updateCoverage()
{
  face_uncovered_.assign(face_region_.size(), 0);
  if (path_edit_ == nullptr || path_edit_->passes().empty())
  {
    coverage_label_->clear();
    updateSelectionColors();
    return;
  }
  const bool radius_enabled = tool_radius_enabled_check_box_->isChecked() && tool_radius_spin_box_->value() > 0.0;
  if (!radius_enabled)
  {
    coverage_label_->setText("Couverture du disque : activez un rayon d'outil non nul pour la mesurer.");
    updateSelectionColors();
    return;
  }

  const CoverageResult coverage = computeCoverage(
      path_edit_->passes(), face_centroids_, plannedFaces(), tool_radius_spin_box_->value() / 1000.0, kSkirtPoses);
  for (const int face : coverage.uncovered_faces)
  {
    face_uncovered_[static_cast<std::size_t>(face)] = 1;
  }
  coverage_label_->setText(describeCoverage(coverage));
  updateSelectionColors();
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
  face_uncovered_.clear();
  if (coverage_label_ != nullptr)
  {
    coverage_label_->clear();
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

void TPPWidget::onLoadPathEdits(const bool /*checked*/)
{
  if (path_edit_->generated().empty())
  {
    QMessageBox::warning(this, "Aucune generation", "Planifiez un chemin avant de charger une retouche.");
    return;
  }

  const QString file = QFileDialog::getOpenFileName(this, "Charger une retouche", "", "YAML files (*.yaml)");
  if (file.isEmpty())
  {
    return;
  }

  try
  {
    path_edit_->applyRecipeFile(readRecipeFile(file.toStdString()), kFingerprintTolerance);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Retouche refusee", QString::fromStdString(ex.what()));
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
