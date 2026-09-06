#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <pcl/io/vtk_lib_io.h>
#include <QColorDialog>
#include <QFileDialog>
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
#include <vtkCellPicker.h>
#include <vtkCoordinate.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkIdList.h>
#include <vtkColorSeries.h>

#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QToolBar>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
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
/** @brief Minimum screen spacing between two recorded lasso points, in pixels */
constexpr double kLassoMinPointSpacingPx = 3.0;

/** @brief Axis-aligned bounds of a 2D outline, as (lower, upper) */
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
 * @brief Whether a face turns toward the eye.
 * @details The direction from the eye to the face is constant only under a parallel projection; a
 * perspective camera needs it recomputed per face. A degenerate face carries a zero normal, whose
 * null dot product drops it here as well.
 */
bool facesTheCamera(const Eigen::Vector3d& normal,
                    const Eigen::Vector3d& centroid,
                    const Eigen::Vector3d& eye,
                    const Eigen::Vector3d& projection_direction,
                    const bool parallel)
{
  const Eigen::Vector3d toward = parallel ? projection_direction : (centroid - eye);
  return normal.dot(toward) < 0.0;
}

/** @brief Lowest region id among a set of faces, or -1 when none of them is selected */
int lowestRegion(const std::vector<int>& face_region, const std::vector<int>& faces)
{
  int lowest = -1;
  for (const int face : faces)
  {
    const int region = face_region[static_cast<std::size_t>(face)];
    if (region >= 0 && (lowest < 0 || region < lowest))
    {
      lowest = region;
    }
  }
  return lowest;
}

/**
 * @brief Interactor style that shares the left mouse button between the camera and the selection
 * tools.
 * @details With no drag tool active the behavior is that of the trackball camera, plus reporting of
 * clicks that did not drag (@ref on_click) so a click can pick a region. With a drag tool active
 * (brush, lasso) the left button feeds @ref on_drag_start / @ref on_drag_move / @ref on_drag_end
 * instead of orbiting, unless Alt is held: the orbit then stays reachable without leaving the tool.
 * Holding Shift marks the drag as an erase, which is reported back to the tool.
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
  /** @brief Called with the display (x, y) of a click that did not drag */
  std::function<void(int, int)> on_click;
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
      on_click(pos[0], pos[1]);
    }

    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
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
};
vtkStandardNewMacro(SelectionInteractorStyle);

}  // namespace

namespace noether
{
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
  , axes_(vtkSmartPointer<vtkAxes>::New())
  , axes_actor_(vtkSmartPointer<vtkAxesActor>::New())
  , tube_filter_(vtkSmartPointer<vtkTubeFilter>::New())
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
  // region, a drag paints or lassos when one of those tools is active.
  cell_picker_ = vtkSmartPointer<vtkCellPicker>::New();
  cell_picker_->SetTolerance(0.0005);
  auto interactor_style = vtkSmartPointer<SelectionInteractorStyle>::New();
  interactor_style->on_click = [this](int x, int y) {
    const int face = pickFace(x, y);
    if (face >= 0)
    {
      onCellClicked(face);
    }
  };
  interactor_style->drag_selects = [this]() { return tool_ != SelectionTool::Region; };
  interactor_style->on_drag_start = [this](int x, int y, bool erase) { onDragStart(x, y, erase); };
  interactor_style->on_drag_move = [this](int x, int y, bool erase) { onDragMove(x, y, erase); };
  interactor_style->on_drag_end = [this](bool erase) { onDragEnd(erase); };
  render_widget_->interactor()->SetInteractorStyle(interactor_style);
  render_widget_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  // Outline overlay for the lasso. Its points live in display coordinates, so the transform
  // coordinate is set explicitly rather than relying on the mapper's default interpretation.
  lasso_poly_ = vtkSmartPointer<vtkPolyData>::New();
  lasso_mapper_ = vtkSmartPointer<vtkPolyDataMapper2D>::New();
  lasso_mapper_->SetInputData(lasso_poly_);
  {
    auto display_coordinate = vtkSmartPointer<vtkCoordinate>::New();
    display_coordinate->SetCoordinateSystemToDisplay();
    lasso_mapper_->SetTransformCoordinate(display_coordinate);
  }
  lasso_actor_ = vtkSmartPointer<vtkActor2D>::New();
  lasso_actor_->SetMapper(lasso_mapper_);
  lasso_actor_->GetProperty()->SetColor(1.0, 1.0, 0.0);
  lasso_actor_->GetProperty()->SetLineWidth(2.0);
  lasso_actor_->SetVisibility(false);
  renderer_->AddActor2D(lasso_actor_);

  // Selection tool selector. The three tools fill the same face_region_ table, so switching is free
  // and leaves what is already selected untouched.
  tool_combo_box_ = new QComboBox(this);
  tool_combo_box_->addItem("Region (click)", static_cast<int>(SelectionTool::Region));
  tool_combo_box_->addItem("Brush (drag)", static_cast<int>(SelectionTool::Brush));
  tool_combo_box_->addItem("Lasso (drag)", static_cast<int>(SelectionTool::Lasso));
  tool_combo_box_->setToolTip(
      "Selection tool. Shift while dragging erases instead of selecting; Alt while dragging orbits "
      "the camera without leaving the tool.");
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
    // An outline left over from an interrupted drag must not survive a tool change.
    lasso_points_.clear();
    updateLassoOverlay();
    updateToolControls();
    render();
  });
  updateToolControls();

  // Toolbar control for the tool radius (mm) used to erode the selected regions. A selected strip
  // narrower than the tool diameter is eroded away (the disc would overhang onto its neighbors).
  tool_radius_spin_box_ = new QDoubleSpinBox(this);
  tool_radius_spin_box_->setRange(0.0, 10000.0);
  tool_radius_spin_box_->setDecimals(1);
  tool_radius_spin_box_->setSingleStep(1.0);
  tool_radius_spin_box_->setValue(0.0);
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
    updateSelectionColors();
    render();
  });
  connect(tool_radius_enabled_check_box_, &QCheckBox::toggled, this, [this](bool enabled) {
    tool_radius_spin_box_->setEnabled(enabled);
    updateSelectionColors();
    render();
  });

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
      if (visited[static_cast<std::size_t>(neighbor)])
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
    kept[f] = (face_region_[f] >= 0 && dist[f] >= radius_m) ? 1 : 0;
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
  for (vtkIdType i = 0; i < cell_colors_->GetNumberOfTuples(); ++i)
  {
    const std::size_t f = static_cast<std::size_t>(i);
    const int region = (f < face_region_.size()) ? face_region_[f] : -1;
    if (region < 0)
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
  if (cell_id < 0 || cell_id >= static_cast<int>(face_region_.size()))
    return;

  if (face_region_[static_cast<std::size_t>(cell_id)] >= 0)
  {
    // Clicking an already-selected region deselects that whole region
    const int region = face_region_[static_cast<std::size_t>(cell_id)];
    for (int& r : face_region_)
      if (r == region)
        r = -1;
  }
  else
  {
    // Grow a new region from the clicked face and give it a fresh id (for a distinct highlight color)
    const std::vector<int> region = growRegion(cell_id);
    const int region_id = next_region_id_++;
    for (int f : region)
      face_region_[static_cast<std::size_t>(f)] = region_id;
  }

  updateSelectionColors();
  render();
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

  if (erase)
  {
    clearFacesFromRegions(face_region_, dab);
    return;
  }

  if (stroke_region_ < 0)
  {
    // A stroke starting on an existing surface extends it; one starting on bare metal opens a new
    // surface. Without this, releasing and painting the same panel again would yield two regions,
    // therefore two tool paths, each paying its own approach and departure.
    const int existing = face_region_[static_cast<std::size_t>(seed)];
    stroke_region_ = (existing >= 0) ? existing : next_region_id_++;
  }
  assignFacesToRegion(face_region_, dab, stroke_region_);
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

std::vector<int> TPPWidget::facesInsideLasso() const
{
  std::vector<int> enclosed;
  vtkCamera* camera = renderer_->GetActiveCamera();
  if (lasso_points_.size() < 3 || face_centroids_.empty() || camera == nullptr)
  {
    return enclosed;
  }

  const bool parallel = camera->GetParallelProjection() != 0;
  const Eigen::Vector3d eye(camera->GetPosition());
  const Eigen::Vector3d projection_direction(camera->GetDirectionOfProjection());

  // Rejecting a face on four comparisons against the outline's bounding box is far cheaper than
  // running the crossing count against an outline that may hold hundreds of points.
  const std::pair<Eigen::Vector2d, Eigen::Vector2d> bounds = outlineBounds(lasso_points_);

  for (std::size_t f = 0; f < face_centroids_.size(); ++f)
  {
    const Eigen::Vector3d& centroid = face_centroids_[f];
    if (!facesTheCamera(face_normals_[f], centroid, eye, projection_direction, parallel))
    {
      continue;
    }

    // VTK reports display coordinates from the bottom-left corner, which is also the origin of the
    // interactor event positions the outline was built from: the two are directly comparable.
    renderer_->SetWorldPoint(centroid.x(), centroid.y(), centroid.z(), 1.0);
    renderer_->WorldToDisplay();
    double display[3] = { 0.0, 0.0, 0.0 };
    renderer_->GetDisplayPoint(display);
    const Eigen::Vector2d screen(display[0], display[1]);

    if ((screen.array() < bounds.first.array()).any() || (screen.array() > bounds.second.array()).any())
    {
      continue;
    }
    if (pointInPolygon(screen, lasso_points_))
    {
      enclosed.push_back(static_cast<int>(f));
    }
  }
  return enclosed;
}

bool TPPWidget::extendLasso(const int x, const int y)
{
  const Eigen::Vector2d point(x, y);

  // Recording every event would pile up thousands of nearly identical points, and the crossing
  // count runs once per face against the whole outline.
  if (!lasso_points_.empty() && (point - lasso_points_.back()).norm() < kLassoMinPointSpacingPx)
  {
    return false;
  }
  lasso_points_.push_back(point);
  return true;
}

void TPPWidget::updateLassoOverlay()
{
  const vtkIdType count = static_cast<vtkIdType>(lasso_points_.size());
  if (count < 2)
  {
    lasso_actor_->SetVisibility(false);
    return;
  }

  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(count);
  for (vtkIdType i = 0; i < count; ++i)
  {
    const Eigen::Vector2d& point = lasso_points_[static_cast<std::size_t>(i)];
    points->SetPoint(i, point.x(), point.y(), 0.0);
  }

  // A closed polyline: the chord from the last point back to the first is drawn because the
  // selection uses it too. Leaving it out would let the operator believe an open curve was traced,
  // then wonder why the selection cut straight across.
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(count + 1);
  for (vtkIdType i = 0; i < count; ++i)
  {
    lines->InsertCellPoint(i);
  }
  lines->InsertCellPoint(0);

  lasso_poly_->SetPoints(points);
  lasso_poly_->SetLines(lines);
  lasso_poly_->Modified();
  lasso_actor_->SetVisibility(true);
}

void TPPWidget::updateToolControls()
{
  // Greying out the readings that do not apply says which parameter governs the active tool.
  flatness_angle_spin_box_->setEnabled(tool_ == SelectionTool::Region);
  brush_diameter_spin_box_->setEnabled(tool_ == SelectionTool::Brush);
}

void TPPWidget::closeLasso(const bool erase)
{
  const std::vector<int> enclosed = facesInsideLasso();
  lasso_points_.clear();
  updateLassoOverlay();

  if (!enclosed.empty())
  {
    if (erase)
    {
      clearFacesFromRegions(face_region_, enclosed);
    }
    else
    {
      // The lowest region the outline covers absorbs the others, so an outline drawn across two
      // surfaces fuses them; one landing only on bare metal opens a new surface.
      const int covered = lowestRegion(face_region_, enclosed);
      assignFacesToRegion(face_region_, enclosed, (covered >= 0) ? covered : next_region_id_++);
    }
  }

  updateSelectionColors();
  render();
}

void TPPWidget::onDragStart(const int x, const int y, const bool erase)
{
  // Allocated lazily by the first dab that lands, so a stroke may begin off the mesh.
  stroke_region_ = -1;
  drag_position_ = Eigen::Vector2d(x, y);

  switch (tool_)
  {
    case SelectionTool::Brush:
      paintBrushDab(x, y, erase);
      updateSelectionColors();
      render();
      break;
    case SelectionTool::Lasso:
      lasso_points_.clear();
      lasso_points_.push_back(Eigen::Vector2d(x, y));
      updateLassoOverlay();
      break;
    case SelectionTool::Region:
      break;  // the click path handles it; Region never consumes a drag
  }
}

void TPPWidget::onDragMove(const int x, const int y, const bool erase)
{
  switch (tool_)
  {
    case SelectionTool::Brush:
      paintBrushAlongSegment(x, y, erase);
      updateSelectionColors();
      render();
      break;
    case SelectionTool::Lasso:
      // Redraw only when the point survived decimation, not on every mouse event.
      if (extendLasso(x, y))
      {
        updateLassoOverlay();
        render();
      }
      break;
    case SelectionTool::Region:
      break;
  }
}

void TPPWidget::onDragEnd(const bool erase)
{
  if (tool_ == SelectionTool::Lasso)
  {
    closeLasso(erase);
  }
  // The brush has already committed every dab, so it has nothing left to do here.
  stroke_region_ = -1;
}

void TPPWidget::clearSelection()
{
  // An outline being traced is part of the selection gesture: clearing one clears the other.
  lasso_points_.clear();
  updateLassoOverlay();

  std::fill(face_region_.begin(), face_region_.end(), -1);
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

void TPPWidget::plan()
{
  try
  {
    // Collect the selected regions as lists of face indices, keyed (and ordered) by region id,
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
      if (f < kept.size() && kept[f])
        region_faces[region].push_back(static_cast<int>(f));
    }

    if (region_faces.empty())
    {
      const QString msg = any_selected ?
                              "All selected regions are narrower than the tool radius, so they were "
                              "fully eroded (a disc of that radius would overhang). Reduce the tool "
                              "radius or select wider regions." :
                              "No mesh regions are selected. Click a region of the part in the viewer "
                              "to select it, then Plan.";
      QMessageBox::warning(this, "Nothing to plan", msg);
      return;
    }

    const ToolPathPlannerPipeline pipeline = pipeline_widget_->createPipeline();
    QApplication::setOverrideCursor(Qt::WaitCursor);

    tool_paths_.clear();
    std::vector<ToolPaths> unmodified_tool_paths;
    fragments_.clear();

    std::size_t skipped_regions = 0;
    std::size_t region_index = 0;
    for (const auto& entry : region_faces)
    {
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

    // Render the modified tool paths
    {
      renderer_->RemoveActor(tool_path_actor_);
      tool_path_actor_ = createToolPathActors(tool_paths_, tube_filter_->GetOutputPort());
      renderer_->AddActor(tool_path_actor_);
      tool_path_actor_->SetVisibility(ui_->action_show_modified_tool_path->isChecked());
    }

    // Render the modified connected paths
    {
      renderer_->RemoveActor(connected_path_actor_);
      connected_path_actor_ = createToolPathPolylineActor(tool_paths_, tube_filter_->GetOutputPort());
      renderer_->AddActor(connected_path_actor_);
      connected_path_actor_->SetVisibility(ui_->action_show_modified_tool_path_lines->isChecked());
    }

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

void TPPWidget::saveToolPaths(const QString& file)
{
  if (tool_paths_.empty())
  {
    QMessageBox::warning(this, "Error", "No tool paths found; please plan a tool path first.");
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
  out << YAML::Node(tool_paths_);
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
