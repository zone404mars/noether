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
#include <noether_gui/widgets/tpp_widget.h>
#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/path_edit_widget.h>

#include <QApplication>
#include <QAbstractButton>
#include <QDoubleSpinBox>
#include <QShortcut>
#include <QTest>
#include <QToolButton>
#include <QTreeWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <gtest/gtest.h>
#include <vtkCellPicker.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>

/** @brief La fenetre complete se construit : tous les panneaux et le cube sont branches sans erreur */
TEST(TPPWidget, TheWindowBuilds)
{
  noether::TPPWidget widget(std::make_shared<noether::WidgetFactory>());
  SUCCEED();
}

/** @brief Acces aux membres proteges de la fenetre, pour piloter l'outil Editer les points */
class EditPointsWindow : public noether::TPPWidget
{
public:
  EditPointsWindow() : noether::TPPWidget(std::make_shared<noether::WidgetFactory>())
  {
    loadToolPaths(TOOL_PATH_DIR "/generated_tool_path.yaml");
    show();
    QApplication::setActiveWindow(this);
    edit_points_button_->setChecked(true);
  }

  /** @brief Nombre de poses de la premiere passe du chemin livre */
  std::size_t poseCount() const { return path_edit_->passes().front().size(); }

  /** @brief Choisit une pose de contact de la premiere passe, comme un clic sur elle */
  void choosePose(const std::size_t index)
  {
    point_selection_.click(noether::PoseRef{ 0, index }, false);
    onPointSelectionChanged();
  }

  /** @brief Quitte l'outil Editer les points comme l'IHM : l'outil de selection reprend la souris */
  void leaveEditing() { releaseDragTools(); }

  /** @brief Vrai quand l'outil Editer les points est actif */
  bool editing() const { return editingPoints(); }

  /** @brief Vrai quand des points sont choisis */
  bool hasSelection() const { return !point_selection_.empty(); }

  /** @brief Arbre de la fenetre, vue 3D et champ Pas cible : trois cibles du focus clavier */
  QTreeWidget* tree() const { return tree_; }
  QWidget* view() const { return render_widget_; }
  QWidget* stepField() const { return densify_step_spin_box_; }

  /** @brief Passe a l'outil Polygone et y pose trois sommets */
  void startPolygon()
  {
    buttonOfTool(noether::SelectionTool::Polygon)->click();
    outline_.addVertex(Eigen::Vector2d(10.0, 10.0));
    outline_.addVertex(Eigen::Vector2d(100.0, 10.0));
    outline_.addVertex(Eigen::Vector2d(100.0, 100.0));
  }

  /** @brief Nombre de sommets du contour du Polygone */
  std::size_t polygonVertexCount() const { return outline_.vertices().size(); }

  /** @brief Ouvre la piece et cadre la camera dessus */
  void openPart(const std::string& file)
  {
    setMeshFile(QString::fromStdString(file));
    render_widget_->renderWindow()->SetSize(800, 600);
    renderer_->ResetCamera();
  }

  /**
   * @brief Pixel de la vue ou le rayon touche la piece, et le point touche ; faux si aucun.
   * Cherche hors du centre : la piece de test a un trou au milieu.
   */
  bool findPartPixel(int& x, int& y, Eigen::Vector3d& point) const
  {
    for (x = 100; x < 800; x += 50)
    {
      for (y = 100; y < 600; y += 50)
      {
        if (pickPoint(x, y, point) >= 0)
        {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * @brief Pose un carre pointable entre la camera et ce point de la piece, comme le chemin
   * dessine devant elle
   */
  void coverPixel(const Eigen::Vector3d& point)
  {
    Eigen::Vector3d eye;
    renderer_->GetActiveCamera()->GetPosition(eye.data());
    Eigen::Vector3d middle = 0.5 * (eye + point);
    Eigen::Vector3d normal = (eye - point).normalized();
    auto square = vtkSmartPointer<vtkPlaneSource>::New();
    square->SetCenter(middle.data());
    square->SetNormal(normal.data());
    square->Update();
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(square->GetOutputPort());
    covering_actor_->SetMapper(mapper);
    renderer_->AddActor(covering_actor_);
    // Comme a chaque rendu de l'IHM : sans cela, le carre serait devant le plan de coupe proche
    renderer_->ResetCameraClippingRange();
  }

  /** @brief Face de la piece trouvee sous le pixel, -1 si aucune */
  int faceUnder(const int x, const int y) const { return pickFace(x, y); }

  /** @brief Vrai quand le carre est le premier acteur sur le rayon du pixel */
  bool coveringActorIsHit(const int x, const int y)
  {
    auto picker = vtkSmartPointer<vtkCellPicker>::New();
    picker->Pick(x, y, 0, renderer_);
    return picker->GetActor() == covering_actor_.Get();
  }

private:
  /** @brief Carre pointable dessine devant la piece */
  vtkSmartPointer<vtkActor> covering_actor_ = vtkSmartPointer<vtkActor>::New();
};

/** @brief Les raccourcis de l'outil marchent quand l'arbre a le focus, pas seulement la vue */
TEST(TPPWidget, EditShortcutsWorkWhenTheTreeHasFocus)
{
  EditPointsWindow window;
  const std::size_t before = window.poseCount();
  window.tree()->setFocus();
  ASSERT_TRUE(window.tree()->hasFocus());

  window.choosePose(5);
  QTest::keyClick(window.tree(), Qt::Key_Delete);
  EXPECT_EQ(window.poseCount(), before - 1);
  QTest::keyClick(window.tree(), Qt::Key_Z, Qt::ControlModifier);
  EXPECT_EQ(window.poseCount(), before);
  QTest::keyClick(window.tree(), Qt::Key_Y, Qt::ControlModifier);
  EXPECT_EQ(window.poseCount(), before - 1);

  window.choosePose(5);
  QTest::keyClick(window.tree(), Qt::Key_Escape);
  EXPECT_FALSE(window.hasSelection());
}

/**
 * @brief Vue 3D au focus : les touches marchent toujours, et un Ctrl+Z defait une seule action.
 * Garde de non-regression : la vue ignore les touches de l'outil, que les raccourcis portent.
 */
TEST(TPPWidget, EditShortcutsWorkWhenTheViewHasFocus)
{
  EditPointsWindow window;
  const std::size_t before = window.poseCount();
  window.view()->setFocus();
  ASSERT_TRUE(window.view()->hasFocus());

  window.choosePose(5);
  QTest::keyClick(window.view(), Qt::Key_Delete);
  window.choosePose(5);
  QTest::keyClick(window.view(), Qt::Key_Delete);
  EXPECT_EQ(window.poseCount(), before - 2);
  QTest::keyClick(window.view(), Qt::Key_Z, Qt::ControlModifier);
  EXPECT_EQ(window.poseCount(), before - 1);
}

/** @brief Le champ Pas cible du panneau garde son Ctrl+Z et son Suppr : le chemin ne change pas */
TEST(TPPWidget, EditShortcutsLeaveTheFieldsAlone)
{
  EditPointsWindow window;
  const std::size_t before = window.poseCount();
  window.choosePose(5);
  window.tree()->setFocus();
  QTest::keyClick(window.tree(), Qt::Key_Delete);
  ASSERT_EQ(window.poseCount(), before - 1);

  window.stepField()->setFocus();
  ASSERT_TRUE(window.stepField()->hasFocus());
  QTest::keyClick(window.stepField(), Qt::Key_Z, Qt::ControlModifier);
  EXPECT_EQ(window.poseCount(), before - 1);
  window.choosePose(5);
  QTest::keyClick(window.stepField(), Qt::Key_Delete);
  EXPECT_EQ(window.poseCount(), before - 1);
}

/** @brief Outil ferme : Ctrl+Z ne touche plus au chemin, Echap reste au Polygone */
TEST(TPPWidget, EditShortcutsAreOffWithAnotherTool)
{
  EditPointsWindow window;
  const std::size_t before = window.poseCount();
  window.choosePose(5);
  window.tree()->setFocus();
  QTest::keyClick(window.tree(), Qt::Key_Delete);
  ASSERT_EQ(window.poseCount(), before - 1);

  window.leaveEditing();
  ASSERT_FALSE(window.editing());
  QTest::keyClick(window.tree(), Qt::Key_Z, Qt::ControlModifier);
  EXPECT_EQ(window.poseCount(), before - 1);

  window.startPolygon();
  ASSERT_EQ(window.polygonVertexCount(), 3u);
  window.view()->setFocus();
  QTest::keyClick(window.view(), Qt::Key_Escape);
  EXPECT_EQ(window.polygonVertexCount(), 0u);
}

/** @brief Un acteur dessine devant la piece n'arrete pas le rayon : la face de la piece est trouvee */
TEST(TPPWidget, ThePickerSeesThePartBehindTheToolPath)
{
  EditPointsWindow window;
  window.openPart(MESH_DIR "/wavy_mesh_with_hole.ply");
  int x = 0;
  int y = 0;
  Eigen::Vector3d point;
  ASSERT_TRUE(window.findPartPixel(x, y, point));
  window.coverPixel(point);
  // Sans ce controle, un carre mal place ferait passer le test sans rien prouver
  ASSERT_TRUE(window.coveringActorIsHit(x, y));
  EXPECT_GE(window.faceUnder(x, y), 0);
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
