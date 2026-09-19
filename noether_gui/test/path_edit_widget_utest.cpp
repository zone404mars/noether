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

#include <noether_tpp/serialization.h>

#include <QApplication>
#include <gtest/gtest.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

using namespace noether;

namespace
{
/** @brief Fewest waypoints a delivered pass may carry, as z404_sdg_planning reads them */
const std::size_t kMinPoses = 4;

/** @brief Largest position difference still counted as the same generated pass, in metres */
const double kTolerance = 1e-9;

/** @brief Builds a straight run of waypoints along x, one every 10 mm, starting at @p x0 */
ToolPathSegment straightSegment(double x0, std::size_t count)
{
  ToolPathSegment segment;
  for (std::size_t i = 0; i < count; ++i)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x0 + 0.01 * static_cast<double>(i), 0.0, 0.0);
    segment.push_back(pose);
  }
  return segment;
}

/** @brief Three generated passes of ten waypoints each, a metre apart */
std::vector<ToolPathSegment> threePasses()
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 10));
  generated.push_back(straightSegment(1.0, 10));
  generated.push_back(straightSegment(2.0, 10));
  return generated;
}

/** @brief Reads a saved tool path file and flattens it into the list of passes */
std::vector<ToolPathSegment> loadFlattened(const std::string& file)
{
  const YAML::Node root = YAML::LoadFile(file);
  return flattenToolPaths(root.as<std::vector<ToolPaths>>());
}

/** @brief The generated pass indices a delivered pass takes, in order */
std::vector<std::size_t> sourcesOf(const PathRecipe& recipe, std::size_t pass)
{
  std::vector<std::size_t> indices;
  for (std::size_t i = 0; i < recipe[pass].size(); ++i)
  {
    indices.push_back(recipe[pass][i].segment);
  }
  return indices;
}

}  // namespace

TEST(PathEditWidget, StartsOnTheGenerationUnchanged)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  EXPECT_EQ(widget.recipe().size(), 3u);
  EXPECT_EQ(widget.passes().size(), 3u);
  EXPECT_EQ(widget.passes()[0].size(), 10u);
  EXPECT_EQ(widget.selectedPass(), 0);
  EXPECT_TRUE(widget.faults().empty());
}

TEST(PathEditWidget, MovingAPassChangesOnlyTheExecutionOrder)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  widget.selectPass(2);
  widget.moveSelected(-1);

  EXPECT_EQ(sourcesOf(widget.recipe(), 1), std::vector<std::size_t>({ 2u }));
  EXPECT_EQ(sourcesOf(widget.recipe(), 2), std::vector<std::size_t>({ 1u }));
  // La selection suit la passe deplacee, sinon le clic suivant porte sur une autre
  EXPECT_EQ(widget.selectedPass(), 1);

  widget.selectPass(0);
  widget.moveSelected(-1);
  EXPECT_EQ(sourcesOf(widget.recipe(), 0), std::vector<std::size_t>({ 0u }));
}

TEST(PathEditWidget, WeldingJoinsTheSelectedPassToTheOneBefore)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  widget.selectPass(1);
  widget.weldSelected();

  ASSERT_EQ(widget.recipe().size(), 2u);
  EXPECT_EQ(sourcesOf(widget.recipe(), 0), std::vector<std::size_t>({ 0u, 1u }));
  // 10 + 10, moins le retrait de la premiere et l'approche de la seconde
  EXPECT_EQ(widget.passes()[0].size(), 18u);
  EXPECT_EQ(widget.selectedPass(), 0);

  // La premiere passe n'a rien au-dessus d'elle : la soudure ne fait rien
  widget.selectPass(0);
  widget.weldSelected();
  EXPECT_EQ(widget.recipe().size(), 2u);
}

TEST(PathEditWidget, ReversingTurnsEveryPieceAndTheirOrder)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  widget.selectPass(1);
  widget.weldSelected();
  widget.reverseSelected();

  ASSERT_EQ(widget.recipe()[0].size(), 2u);
  EXPECT_EQ(sourcesOf(widget.recipe(), 0), std::vector<std::size_t>({ 1u, 0u }));
  EXPECT_TRUE(widget.recipe()[0][0].reversed);
  EXPECT_TRUE(widget.recipe()[0][1].reversed);

  // Le resultat commence ou la passe soudee finissait
  const std::vector<ToolPathSegment>& generated = widget.generated();
  EXPECT_TRUE(widget.passes()[0].front().translation().isApprox(generated[1].back().translation()));
  EXPECT_TRUE(widget.passes()[0].back().translation().isApprox(generated[0].front().translation()));
}

TEST(PathEditWidget, SplittingUndoesAWeldInPlace)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  widget.selectPass(1);
  widget.weldSelected();
  ASSERT_EQ(widget.recipe().size(), 2u);

  widget.selectPass(0);
  widget.splitSelected();

  ASSERT_EQ(widget.recipe().size(), 3u);
  EXPECT_EQ(sourcesOf(widget.recipe(), 0), std::vector<std::size_t>({ 0u }));
  EXPECT_EQ(sourcesOf(widget.recipe(), 1), std::vector<std::size_t>({ 1u }));
  EXPECT_EQ(sourcesOf(widget.recipe(), 2), std::vector<std::size_t>({ 2u }));
}

TEST(PathEditWidget, RemovingDropsThePassFromTheDelivery)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  widget.selectPass(1);
  widget.removeSelected();

  ASSERT_EQ(widget.recipe().size(), 2u);
  EXPECT_EQ(sourcesOf(widget.recipe(), 0), std::vector<std::size_t>({ 0u }));
  EXPECT_EQ(sourcesOf(widget.recipe(), 1), std::vector<std::size_t>({ 2u }));
  EXPECT_TRUE(widget.faults().empty());
}

TEST(PathEditWidget, UndoGoesBackOneChangeAtATime)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  widget.selectPass(1);
  widget.weldSelected();
  widget.selectPass(0);
  widget.reverseSelected();
  ASSERT_TRUE(widget.recipe()[0][0].reversed);

  widget.undo();
  EXPECT_FALSE(widget.recipe()[0][0].reversed);
  EXPECT_EQ(widget.recipe().size(), 2u);

  widget.undo();
  EXPECT_EQ(widget.recipe().size(), 3u);

  // Rien de plus a annuler : l'etat ne bouge plus
  widget.undo();
  EXPECT_EQ(widget.recipe().size(), 3u);
}

TEST(PathEditWidget, ResetReturnsToTheGeneration)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  widget.selectPass(1);
  widget.weldSelected();
  widget.selectPass(1);
  widget.removeSelected();
  ASSERT_EQ(widget.recipe().size(), 1u);

  widget.reset();
  EXPECT_EQ(widget.recipe().size(), 3u);
}

TEST(PathEditWidget, ReportsAPassTooShortToDeliver)
{
  std::vector<ToolPathSegment> generated;
  generated.push_back(straightSegment(0.0, 3));
  generated.push_back(straightSegment(1.0, 2));

  PathEditWidget widget(kMinPoses);
  widget.setGenerated(generated);
  EXPECT_EQ(widget.faults().size(), 2u);  // les deux passes sont sous le contrat

  widget.selectPass(1);
  widget.weldSelected();
  ASSERT_EQ(widget.passes()[0].size(), 3u);
  EXPECT_EQ(widget.faults().size(), 1u);
}

TEST(PathEditWidget, NotifiesItsHostOnEditAndOnSelection)
{
  PathEditWidget widget(kMinPoses);
  int edits = 0;
  int selections = 0;
  widget.on_changed = [&edits]() { ++edits; };
  widget.on_selection_changed = [&selections]() { ++selections; };

  widget.setGenerated(threePasses());
  const int after_generation = edits;
  EXPECT_GT(after_generation, 0);

  widget.selectPass(2);
  EXPECT_EQ(selections, 1);
  EXPECT_EQ(edits, after_generation);  // choisir une ligne ne retouche rien

  widget.reverseSelected();
  EXPECT_GT(edits, after_generation);
}

TEST(PathEditWidget, KnowsWhetherAnythingWasRetouched)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());
  EXPECT_FALSE(widget.isRetouched());

  widget.selectPass(1);
  widget.reverseSelected();
  EXPECT_TRUE(widget.isRetouched());

  widget.undo();
  EXPECT_FALSE(widget.isRetouched());

  // Un simple echange de rang compte aussi : l'ordre d'execution a change
  widget.selectPass(2);
  widget.moveSelected(-1);
  EXPECT_TRUE(widget.isRetouched());

  widget.reset();
  EXPECT_FALSE(widget.isRetouched());

  // Une generation vide n'a rien a retoucher
  PathEditWidget empty(kMinPoses);
  EXPECT_FALSE(empty.isRetouched());
}

TEST(PathEditWidget, ATakenBackGenerationClearsEverything)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());
  widget.selectPass(1);
  widget.weldSelected();

  widget.setGenerated(std::vector<ToolPathSegment>());

  EXPECT_TRUE(widget.recipe().empty());
  EXPECT_TRUE(widget.passes().empty());
  EXPECT_EQ(widget.selectedPass(), -1);
  EXPECT_TRUE(widget.faults().empty());
  EXPECT_FALSE(widget.isRetouched());
}

TEST(PathEditWidget, RefusesARecipeWrittenOnAnotherGeneration)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());

  RecipeFile content;
  content.source = fingerprintSegments(threePasses());
  content.recipe = identityRecipe(3);
  EXPECT_NO_THROW(widget.applyRecipeFile(content, kTolerance));

  std::vector<ToolPathSegment> other = threePasses();
  other[2] = straightSegment(5.0, 10);
  content.source = fingerprintSegments(other);
  EXPECT_THROW(widget.applyRecipeFile(content, kTolerance), std::runtime_error);
}

TEST(PathEditWidget, ReplaysASavedRecipeOnTheSameGeneration)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());
  widget.selectPass(1);
  widget.weldSelected();
  widget.selectPass(0);
  widget.reverseSelected();

  RecipeFile content;
  content.recipe = widget.recipe();
  content.source = fingerprintSegments(widget.generated());

  PathEditWidget replayed(kMinPoses);
  replayed.setGenerated(threePasses());
  replayed.applyRecipeFile(content, kTolerance);

  ASSERT_EQ(replayed.passes().size(), widget.passes().size());
  for (std::size_t p = 0; p < replayed.passes().size(); ++p)
  {
    ASSERT_EQ(replayed.passes()[p].size(), widget.passes()[p].size());
    for (std::size_t w = 0; w < replayed.passes()[p].size(); ++w)
    {
      EXPECT_TRUE(replayed.passes()[p][w].isApprox(widget.passes()[p][w], 1e-12));
    }
  }
}

TEST(PathEditWidget, AnEmptyGenerationDeliversNothingRatherThanPassingValidation)
{
  // Le dock vide doit rester vide : c'est ce sur quoi la fenetre s'appuie pour refuser
  // d'enregistrer apres une planification qui a echoue en cours de route.
  PathEditWidget widget(kMinPoses);
  EXPECT_TRUE(widget.generated().empty());
  EXPECT_TRUE(widget.passes().empty());
  EXPECT_TRUE(widget.recipe().empty());
}

TEST(PathEditWidget, ReachesTheHandEditOfWorkpiece5803024014WithTheButtonsAlone)
{
  const std::vector<ToolPathSegment> generated =
      loadFlattened(std::string(TOOL_PATH_DIR) + "/generated_tool_path.yaml");
  const std::vector<ToolPathSegment> expected =
      loadFlattened(std::string(TOOL_PATH_DIR) + "/hand_edited_tool_path.yaml");
  ASSERT_EQ(generated.size(), 7u);

  PathEditWidget widget(kMinPoses);
  widget.setGenerated(generated);

  // La suite de gestes qu'un operateur fait au dock pour retrouver la retouche a la main.
  // Les passes 1, 3 et 5 sont montees cote a cote, soudees dans cet ordre, puis la passe
  // soudee est inversee d'un coup : c'est ce qui donne 5, 3, 1 a l'envers.
  widget.selectPass(2);
  widget.moveSelected(-1);
  widget.weldSelected();

  widget.selectPass(5);
  for (int i = 0; i < 4; ++i)
  {
    widget.moveSelected(-1);
  }

  widget.selectPass(5);
  widget.moveSelected(-1);

  widget.selectPass(3);
  widget.weldSelected();
  widget.selectPass(3);
  widget.weldSelected();

  widget.selectPass(2);
  widget.reverseSelected();

  EXPECT_TRUE(widget.faults().empty());
  ASSERT_EQ(widget.passes().size(), expected.size());
  for (std::size_t p = 0; p < expected.size(); ++p)
  {
    ASSERT_EQ(widget.passes()[p].size(), expected[p].size()) << "passe " << p;
    for (std::size_t w = 0; w < expected[p].size(); ++w)
    {
      EXPECT_TRUE(widget.passes()[p][w].isApprox(expected[p][w], 1e-12)) << "passe " << p << ", pose " << w;
    }
  }
}

TEST(PathEditWidget, MovesAPoseAndUndoesTheWholeDragAtOnce)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());
  EXPECT_FALSE(widget.isRetouched());

  PoseMove move;
  move.segment = 1;
  move.index = 3;
  widget.beginPoseMoves();
  for (int step = 1; step <= 5; ++step)
  {
    move.pose.translation() = Eigen::Vector3d(1.03, 0.01 * step, 0.0);
    widget.movePose(move);
  }
  ASSERT_EQ(widget.moves().size(), 1u);
  EXPECT_TRUE(widget.isRetouched());
  EXPECT_NEAR(widget.passes()[1][3].translation().y(), 0.05, 1e-12);
  EXPECT_NEAR(widget.generated()[1][3].translation().y(), 0.0, 1e-12);

  widget.undo();
  EXPECT_TRUE(widget.moves().empty());
  EXPECT_FALSE(widget.isRetouched());
  EXPECT_NEAR(widget.passes()[1][3].translation().y(), 0.0, 1e-12);
}

TEST(PathEditWidget, MovedPosesFollowTheRecipe)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());
  PoseMove move;
  move.segment = 0;
  move.index = 0;
  move.pose.translation() = Eigen::Vector3d(-1.0, 0.0, 0.0);
  widget.beginPoseMoves();
  widget.movePose(move);
  // La passe 0 retournee : la pose deplacee, premiere de la generation, finit derniere
  widget.selectPass(0);
  widget.reverseSelected();
  EXPECT_NEAR(widget.passes()[0].back().translation().x(), -1.0, 1e-12);
}

TEST(PathEditWidget, ChainsEveryPassFromTheSelectedOneWithoutLifting)
{
  PathEditWidget widget(kMinPoses);
  widget.setGenerated(threePasses());
  widget.selectPass(2);
  widget.chainFromSelected();

  ASSERT_EQ(widget.recipe().size(), 1u);
  // From the pass at x = 2, the nearest end of the pass at x = 1 is its end, and so on down
  const std::vector<std::size_t> expected = { 2, 1, 0 };
  EXPECT_EQ(sourcesOf(widget.recipe(), 0), expected);
  EXPECT_FALSE(widget.recipe()[0][0].reversed);
  EXPECT_TRUE(widget.recipe()[0][1].reversed);
  EXPECT_TRUE(widget.recipe()[0][2].reversed);
  EXPECT_EQ(widget.passes().size(), 1u);
  EXPECT_TRUE(widget.isRetouched());

  widget.undo();
  EXPECT_EQ(widget.recipe().size(), 3u);
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
