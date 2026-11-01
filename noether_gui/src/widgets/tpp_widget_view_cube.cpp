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
 * @file tpp_widget_view_cube.cpp
 * @brief Cube de navigation (spec §22) : dessin en haut a droite de la vue, clics sur ses faces,
 * aretes et coins, fleches vers les faces voisines, quarts de tour et bouton maison.
 */

#include <noether_gui/widgets/tpp_widget.h>

#include <QToolButton>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkAnnotatedCubeActor.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkOpenGLRenderer.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>

namespace noether
{
namespace
{
/** @brief Cote de la zone du cube et de ses boutons, en pixels logiques */
constexpr int kCubeAreaPx = 130;
/** @brief Ecart entre la zone du cube et les bords de la vue, en pixels logiques */
constexpr int kCubeMarginPx = 8;
/** @brief Cote d'un bouton autour du cube, en pixels logiques */
constexpr int kCubeButtonPx = 22;
/** @brief Demi-hauteur vue par la camera du cube : le cube de cote 1 tient dans la zone avec ses boutons */
constexpr double kCubeParallelScale = 1.1;
/** @brief Distance de la camera du cube a son centre, hors du cube */
constexpr double kCubeCameraDistance = 3.0;
}  // namespace

void TPPWidget::buildViewCube()
{
  buildViewCubeActor();
  view_cube_renderer_ = vtkSmartPointer<vtkRenderer>::New();
  // Couche du dessus, non interactive : rotation et zoom restent au renderer principal
  view_cube_renderer_->SetLayer(1);
  view_cube_renderer_->InteractiveOff();
  view_cube_renderer_->AddActor(view_cube_actor_);
  view_cube_renderer_->GetActiveCamera()->ParallelProjectionOn();
  view_cube_renderer_->GetActiveCamera()->SetParallelScale(kCubeParallelScale);
  vtkRenderWindow* window = render_widget_->renderWindow();
  window->SetNumberOfLayers(2);
  window->AddRenderer(view_cube_renderer_);
  view_cube_observer_ = renderer_->AddObserver(vtkCommand::StartEvent, this, &TPPWidget::syncViewCube);
  buildViewCubeButtons();
  translator_.bind([this]() { labelViewCubeFaces(); });
}

void TPPWidget::buildViewCubeActor()
{
  view_cube_actor_ = vtkSmartPointer<vtkAnnotatedCubeActor>::New();
  view_cube_actor_->SetFaceTextScale(0.14);
  view_cube_actor_->GetCubeProperty()->SetColor(0.85, 0.86, 0.9);
  // Lettres pleines et sombres, sans contour : un contour vu de profil deborde du cube
  view_cube_actor_->SetTextEdgesVisibility(false);
  for (vtkProperty* face : { view_cube_actor_->GetXPlusFaceProperty(), view_cube_actor_->GetXMinusFaceProperty(),
                             view_cube_actor_->GetYPlusFaceProperty(), view_cube_actor_->GetYMinusFaceProperty(),
                             view_cube_actor_->GetZPlusFaceProperty(), view_cube_actor_->GetZMinusFaceProperty() })
  {
    face->SetColor(0.15, 0.15, 0.2);
  }
  // Texte du dessus lisible dans la vue Dessus, haut de l'ecran +Y
  view_cube_actor_->SetZFaceTextRotation(90.0);
}

void TPPWidget::buildViewCubeButtons()
{
  addViewCubeArrow(CubeArrow::Up, "▲", { "Face voisine en haut", "Upper neighbour face" });
  addViewCubeArrow(CubeArrow::Down, "▼", { "Face voisine en bas", "Lower neighbour face" });
  addViewCubeArrow(CubeArrow::Left, "◀", { "Face voisine à gauche", "Left neighbour face" });
  addViewCubeArrow(CubeArrow::Right, "▶", { "Face voisine à droite", "Right neighbour face" });
  view_cube_roll_ccw_ = addViewCubeButton("↺", { "Quart de tour, sens inverse", "Quarter turn, counterclockwise" },
                                          [this]() { onViewCubeRoll(false); });
  view_cube_roll_cw_ = addViewCubeButton("↻", { "Quart de tour, sens horaire", "Quarter turn, clockwise" },
                                         [this]() { onViewCubeRoll(true); });
  view_cube_home_ = addViewCubeButton("⌂", { "Vue isométrique", "Isometric view" },
                                      [this]() { setCameraView(kHomeViewDirection, Eigen::Vector3d::UnitZ()); });
}

void TPPWidget::addViewCubeArrow(const CubeArrow arrow, const QString& symbol, const Bilingual& tip)
{
  QToolButton* button = addViewCubeButton(symbol, tip, [this, arrow]() { onViewCubeArrow(arrow); });
  view_cube_arrows_.emplace_back(arrow, button);
}

void TPPWidget::labelViewCubeFaces()
{
  // Le texte des faces est dessine par vtkVectorText, qui ne connait que l'ASCII : pas d'accent
  const bool french = translator_.language() == Language::FRENCH;
  view_cube_actor_->SetXPlusFaceText(french ? "DROITE" : "RIGHT");
  view_cube_actor_->SetXMinusFaceText(french ? "GAUCHE" : "LEFT");
  view_cube_actor_->SetYPlusFaceText(french ? "ARRIERE" : "BACK");
  view_cube_actor_->SetYMinusFaceText(french ? "FACE" : "FRONT");
  view_cube_actor_->SetZPlusFaceText(french ? "DESSUS" : "TOP");
  view_cube_actor_->SetZMinusFaceText(french ? "DESSOUS" : "BOTTOM");
}

QToolButton* TPPWidget::addViewCubeButton(const QString& symbol, const Bilingual& tip,
                                          const std::function<void()>& action)
{
  auto* button = new QToolButton(render_widget_);
  button->setText(symbol);
  button->setAutoRaise(true);
  button->setFixedSize(kCubeButtonPx, kCubeButtonPx);
  // Symbole clair : le fond de la vue est gris fonce
  button->setStyleSheet("QToolButton { color: #e8e8ee; font-size: 14px; }");
  translator_.bindToolTip(button, tip);
  connect(button, &QToolButton::clicked, this, [action](bool) { action(); });
  return button;
}

void TPPWidget::syncViewCube()
{
  // La camera du cube regarde son centre depuis le meme cote et avec le meme haut que la vue
  const Eigen::Vector3d position = cameraSide() * kCubeCameraDistance;
  const Eigen::Vector3d up = cameraUp();
  vtkCamera* camera = view_cube_renderer_->GetActiveCamera();
  camera->SetFocalPoint(0.0, 0.0, 0.0);
  camera->SetPosition(position.x(), position.y(), position.z());
  camera->SetViewUp(up.x(), up.y(), up.z());
  view_cube_renderer_->ResetCameraClippingRange();

  // Zone carree de taille fixe a l'ecran : le viewport VTK se donne en fraction de la fenetre
  const int* size = render_widget_->renderWindow()->GetSize();
  if (size[0] > 0 && size[1] > 0)
  {
    const double dpr = render_widget_->devicePixelRatioF();
    const double area = kCubeAreaPx * dpr;
    const double margin = kCubeMarginPx * dpr;
    view_cube_renderer_->SetViewport(std::max(0.0, 1.0 - (area + margin) / size[0]),
                                     std::max(0.0, 1.0 - (area + margin) / size[1]), 1.0 - margin / size[0],
                                     1.0 - margin / size[1]);
  }
  // Les boutons Qt se deplacent hors du dessin VTK, apres cette image
  QMetaObject::invokeMethod(this, [this]() { updateViewCubeButtons(); }, Qt::QueuedConnection);
}

void TPPWidget::updateViewCubeButtons()
{
  // Coordonnees Qt, en pixels logiques, origine en haut a gauche
  const int left = render_widget_->width() - kCubeMarginPx - kCubeAreaPx;
  const int top = kCubeMarginPx;
  const int middle = (kCubeAreaPx - kCubeButtonPx) / 2;
  const int far = kCubeAreaPx - kCubeButtonPx;
  const bool aligned = alignedFace(cameraSide()).has_value();
  for (const auto& entry : view_cube_arrows_)
  {
    const int dx = entry.first == CubeArrow::Left ? 0 : (entry.first == CubeArrow::Right ? far : middle);
    const int dy = entry.first == CubeArrow::Up ? 0 : (entry.first == CubeArrow::Down ? far : middle);
    entry.second->move(left + dx, top + dy);
    entry.second->setVisible(aligned);
  }
  view_cube_roll_ccw_->move(left, top);
  view_cube_roll_ccw_->setVisible(aligned);
  view_cube_roll_cw_->move(left + far, top);
  view_cube_roll_cw_->setVisible(aligned);
  view_cube_home_->move(left, top + far);
  view_cube_home_->show();
}

Eigen::Vector3d TPPWidget::cameraSide() const
{
  vtkCamera* camera = renderer_->GetActiveCamera();
  Eigen::Vector3d position;
  Eigen::Vector3d focal_point;
  camera->GetPosition(position.data());
  camera->GetFocalPoint(focal_point.data());
  return (position - focal_point).normalized();
}

Eigen::Vector3d TPPWidget::cameraUp() const
{
  Eigen::Vector3d up;
  renderer_->GetActiveCamera()->GetViewUp(up.data());
  return up;
}

std::optional<Eigen::Vector3d> TPPWidget::viewCubePoint(const int x, const int y) const
{
  if (!view_cube_renderer_ || !view_cube_renderer_->IsInViewport(x, y))
  {
    return std::nullopt;
  }
  // Rayon du pixel : du plan proche au plan lointain de la camera du cube
  Eigen::Vector3d ends[2];
  for (int i = 0; i < 2; ++i)
  {
    view_cube_renderer_->SetDisplayPoint(x, y, static_cast<double>(i));
    view_cube_renderer_->DisplayToWorld();
    const double* world = view_cube_renderer_->GetWorldPoint();
    ends[i] = Eigen::Vector3d(world[0], world[1], world[2]) / world[3];
  }
  return cubeHit(ends[0], ends[1] - ends[0]);
}

void TPPWidget::onViewCubeClick(const int x, const int y)
{
  const std::optional<Eigen::Vector3d> point = viewCubePoint(x, y);
  if (!point)
  {
    return;
  }
  const Eigen::Vector3d direction = cubeZoneDirection(*point);
  setCameraView(direction, defaultViewUp(direction));
}

void TPPWidget::onViewCubeArrow(const CubeArrow arrow)
{
  const std::optional<Eigen::Vector3d> face = alignedFace(cameraSide());
  if (!face)
  {
    return;
  }
  const Eigen::Vector3d next = neighborFace(*face, cameraUp(), arrow);
  setCameraView(next, defaultViewUp(next));
}

void TPPWidget::onViewCubeRoll(const bool clockwise)
{
  const std::optional<Eigen::Vector3d> face = alignedFace(cameraSide());
  if (!face)
  {
    return;
  }
  setCameraView(*face, rolledUp(*face, cameraUp(), clockwise));
}

}  // namespace noether
