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
#include <noether_gui/widgets/path_edit_widget.h>

#include <QApplication>
#include <gtest/gtest.h>

using namespace noether;

namespace
{
/** @brief Nombre minimal de poses d'une passe livree, comme z404_sdg_planning le lit */
const std::size_t kMinPoses = 4;

/** @brief Passe droite le long de x, une pose tous les 10 mm a partir de @p x0, sans surface */
EditedPass straightPass(const double x0, const std::size_t count)
{
  EditedPass pass;
  for (std::size_t i = 0; i < count; ++i)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x0 + 0.01 * static_cast<double>(i), 0.0, 0.0);
    pass.poses.push_back(pose);
  }
  return pass;
}

/** @brief Trois passes de dix poses, a un metre l'une de l'autre */
std::vector<EditedPass> threePasses()
{
  return { straightPass(0.0, 10), straightPass(1.0, 10), straightPass(2.0, 10) };
}

/** @brief Abscisse de la premiere pose de chaque passe livree : elle nomme la passe */
std::vector<double> passStarts(const PathEditWidget& widget)
{
  std::vector<double> starts;
  for (const ToolPathSegment& pass : widget.passes())
  {
    starts.push_back(pass.front().translation().x());
  }
  return starts;
}
}  // namespace

TEST(PathEditWidget, StartsOnTheGenerationUnchanged)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses(threePasses());
  EXPECT_EQ(widget.passes().size(), 3u);
  EXPECT_EQ(widget.passes()[0].size(), 10u);
  EXPECT_EQ(widget.selectedPass(), 0);
  EXPECT_TRUE(widget.faults().empty());
  EXPECT_FALSE(widget.isModified());
}

TEST(PathEditWidget, MovingAPassChangesOnlyTheExecutionOrder)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses(threePasses());
  widget.selectPass(2);
  widget.moveSelected(-1);
  EXPECT_EQ(passStarts(widget), std::vector<double>({ 0.0, 2.0, 1.0 }));
  // Le choix suit la passe deplacee, sinon le clic suivant porterait sur une autre
  EXPECT_EQ(widget.selectedPass(), 1);
  widget.selectPass(0);
  widget.moveSelected(-1);
  EXPECT_EQ(passStarts(widget), std::vector<double>({ 0.0, 2.0, 1.0 }));
}

TEST(PathEditWidget, RemovingDropsThePassFromTheDelivery)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses(threePasses());
  widget.selectPass(1);
  widget.removeSelected();
  EXPECT_EQ(passStarts(widget), std::vector<double>({ 0.0, 2.0 }));
  EXPECT_EQ(widget.selectedPass(), 1);
}

TEST(PathEditWidget, EveryRenumberingIsAnnouncedAndNothingElse)
{
  // Les points choisis designent des rangs de poses : chaque changement de rangs doit etre annonce
  PathEditWidget widget(kMinPoses);
  int announced = 0;
  widget.on_poses_renumbered = [&announced]() { ++announced; };
  widget.setPasses(threePasses());
  EXPECT_EQ(announced, 1);
  widget.selectPass(0);
  widget.moveSelected(-1);
  EXPECT_EQ(announced, 1);
  widget.moveSelected(1);
  widget.removeSelected();
  widget.undo();
  widget.redo();
  EXPECT_EQ(announced, 5);
  // Un glisser ne change aucun rang
  widget.beginDrag();
  widget.setPose(PoseRef{ 0, 1 }, widget.editedPasses()[0].poses[1]);
  widget.endDrag();
  EXPECT_EQ(announced, 5);
}

TEST(PathEditWidget, UndoAndRedoWalkTheActions)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses(threePasses());
  widget.selectPass(0);
  widget.removeSelected();
  widget.selectPass(1);
  widget.moveSelected(-1);
  EXPECT_EQ(passStarts(widget), std::vector<double>({ 2.0, 1.0 }));
  widget.undo();
  EXPECT_EQ(passStarts(widget), std::vector<double>({ 1.0, 2.0 }));
  widget.undo();
  EXPECT_EQ(passStarts(widget), std::vector<double>({ 0.0, 1.0, 2.0 }));
  EXPECT_FALSE(widget.isModified());
  widget.redo();
  EXPECT_EQ(passStarts(widget), std::vector<double>({ 1.0, 2.0 }));
}

TEST(PathEditWidget, ReportsAPassTooShortToDeliver)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses({ straightPass(0.0, 3), straightPass(1.0, 2), straightPass(2.0, 4) });
  // Deux passes sous le minimum, chacune nommee
  EXPECT_EQ(widget.faults().size(), 2u);
}

TEST(PathEditWidget, NotifiesItsHostOnEditAndOnSelection)
{
  PathEditWidget widget(kMinPoses);
  int edits = 0;
  int selections = 0;
  widget.on_changed = [&edits]() { ++edits; };
  widget.on_selection_changed = [&selections]() { ++selections; };
  widget.setPasses(threePasses());
  const int after_generation = edits;
  EXPECT_GT(after_generation, 0);
  widget.selectPass(2);
  EXPECT_EQ(selections, 1);
  // Choisir une ligne ne modifie pas le chemin
  EXPECT_EQ(edits, after_generation);
  widget.removeSelected();
  EXPECT_GT(edits, after_generation);
}

TEST(PathEditWidget, ANewGenerationClearsEverything)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses(threePasses());
  widget.selectPass(1);
  widget.removeSelected();
  widget.setPasses(std::vector<EditedPass>());
  EXPECT_TRUE(widget.passes().empty());
  EXPECT_EQ(widget.selectedPass(), -1);
  EXPECT_TRUE(widget.faults().empty());
  EXPECT_FALSE(widget.isModified());
  widget.undo();
  EXPECT_TRUE(widget.passes().empty());
}

TEST(PathEditWidget, ADragIsUndoneAtOnce)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses(threePasses());
  int edits = 0;
  widget.on_changed = [&edits]() { ++edits; };
  widget.beginDrag();
  for (int step = 1; step <= 5; ++step)
  {
    Eigen::Isometry3d pose = widget.editedPasses()[0].poses[3];
    pose.translation().y() = 0.001 * step;
    widget.setPose(PoseRef{ 0, 3 }, pose);
  }
  widget.endDrag();
  // L'hote ne redessine qu'au relachement
  EXPECT_EQ(edits, 1);
  EXPECT_NEAR(widget.passes()[0][3].translation().y(), 0.005, 1e-12);
  widget.undo();
  EXPECT_NEAR(widget.passes()[0][3].translation().y(), 0.0, 1e-12);
}

TEST(PathEditWidget, InsertedAndRemovedPosesAreOneActionEach)
{
  PathEditWidget widget(kMinPoses);
  widget.setPasses(threePasses());
  ToolPathSegment added(2, widget.editedPasses()[0].poses[2]);
  widget.insertPoses({ GapInsertion{ 0, 2, added } });
  EXPECT_EQ(widget.passes()[0].size(), 12u);
  widget.removePoses({ PoseRef{ 0, 4 }, PoseRef{ 0, 5 } });
  EXPECT_EQ(widget.passes()[0].size(), 10u);
  widget.undo();
  EXPECT_EQ(widget.passes()[0].size(), 12u);
  widget.undo();
  EXPECT_EQ(widget.passes()[0].size(), 10u);
  EXPECT_FALSE(widget.isModified());
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
