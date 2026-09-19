/**
 * @file tool_path_recipe.h
 * @copyright Copyright (c) 2026, Zone 404
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <noether_tpp/core/types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <string>
#include <vector>

namespace noether
{
/**
 * @brief One generated pass, taken up into a delivered pass.
 */
struct SourceSegment
{
  /** @brief Index of the pass in the flattened generation */
  std::size_t segment = 0;
  /** @brief The pass is travelled backwards */
  bool reversed = false;
};

/**
 * @brief What every delivered pass takes from the generation, in execution order.
 * @details One entry per delivered pass; each entry lists the generated passes it concatenates.
 * A generated pass listed nowhere is dropped from the delivery.
 *
 * The form is declarative rather than a sequence of mutations on purpose: an imperative recipe
 * ("reverse 5, then join 5 and 3, then reorder") renumbers its own later steps, which makes it
 * unreadable six months on and ambiguous to replay. Here the recipe is a permutation with grouping
 * and direction, so its meaning does not depend on the order the operator obtained it in.
 *
 * Because a delivered pass is a concatenation of generated passes, every waypoint of the result
 * exists in the generation. That property holds by construction, not by test.
 */
using PathRecipe = std::vector<std::vector<SourceSegment>>;

/**
 * @brief Enough of a generated pass to tell whether a recipe still describes it.
 * @details A recipe indexes passes, and indices only mean something for the generation they were
 * written against. Replaying a recipe on a generation whose passes moved would weld the wrong ones,
 * silently. These fingerprints are what turns that into a refusal.
 */
struct SegmentFingerprint
{
  /** @brief Number of waypoints in the pass */
  std::size_t pose_count = 0;
  /** @brief Position of the first waypoint */
  Eigen::Vector3d first = Eigen::Vector3d::Zero();
  /** @brief Position of the last waypoint */
  Eigen::Vector3d last = Eigen::Vector3d::Zero();
};

/**
 * @brief A recipe together with the generation it was written against.
 */
/**
 * @brief Une pose de la generation deplacee a la main par l'operateur.
 * @details Le deplacement designe la pose par sa place dans la generation, pas dans la
 * livraison : les indices de la generation ne bougent pas quand la recette reordonne ou soude
 * les passes. La pose complete est enregistree, position et orientation.
 */
struct PoseMove
{
  /** @brief Passe de la generation, aplatie */
  std::size_t segment = 0;
  /** @brief Rang de la pose dans cette passe */
  std::size_t index = 0;
  /** @brief La pose apres deplacement */
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct RecipeFile
{
  /** @brief What every delivered pass takes from the generation */
  PathRecipe recipe;
  /** @brief Fingerprint of every generated pass, in the order the recipe indexes them */
  std::vector<SegmentFingerprint> source;
  /** @brief Poses deplacees a la main, appliquees a la generation avant la recette */
  std::vector<PoseMove> moves;
};

/**
 * @brief Distances measured across a weld, for the operator to judge it before accepting it.
 */
struct WeldMetrics
{
  /** @brief Distance between the two waypoints that become neighbours, in the mesh's length unit */
  double gap = -1.0;
  /** @brief Angle between the tool axes of those two waypoints, in radians */
  double angle = -1.0;
};

/**
 * @brief Flattens the planner's output into the list of passes the operator and the cell both see.
 * @details The planner nests its result as fragment / tool path / segment. Every consumer of the
 * saved file walks that nesting to any depth and reads the leaves in file order, so the flat list
 * is the only ordering that means the same thing to the GUI, the recipe and the cell.
 * @param tool_paths Planner output, one entry per planned surface
 * @return The passes, in file order
 */
std::vector<ToolPathSegment> flattenToolPaths(const std::vector<ToolPaths>& tool_paths);

/**
 * @brief Wraps a list of passes back into the nesting the file format expects.
 * @details One fragment holding one tool path holding the passes. The fragment level carried the
 * planned surface, and a weld across two surfaces makes that level untrue, so it is collapsed
 * rather than guessed at.
 * @param passes The passes, in execution order
 */
std::vector<ToolPaths> nestToolPath(const ToolPath& passes);

/**
 * @brief The recipe that changes nothing: every generated pass delivered alone, in order.
 * @param segment_count Number of generated passes
 */
PathRecipe identityRecipe(std::size_t segment_count);

/**
 * @brief Assembles the passes a recipe describes.
 * @details Where two generated passes are concatenated, one waypoint is dropped on each side of the
 * joint. Those two are the departure of the first and the approach of the second, both standing off
 * the surface; keeping them would place a lift and a re-approach in the middle of a pass that is
 * meant to stay in contact.
 *
 * Length is not checked here: an assembly too short to deliver must still be displayable while the
 * operator works on it. See ::validateRecipe for the check that gates delivery.
 *
 * @param generated The generated passes, flattened
 * @param recipe What every delivered pass takes from them
 * @return The delivered passes, in execution order
 * @throws std::runtime_error if the recipe indexes a pass that does not exist, uses one twice, or
 *         carries an empty entry. Those are faults of the recipe itself, not of the edit in
 *         progress.
 */
ToolPath assemblePasses(const std::vector<ToolPathSegment>& generated, const PathRecipe& recipe);

/**
 * @brief Lists what forbids delivering a recipe, one message per fault.
 * @param generated The generated passes, flattened
 * @param recipe What every delivered pass takes from them
 * @param min_poses Fewest waypoints a delivered pass may carry. No default: the figure comes from
 *                  the consumer's contract, and a wrong guess here is only found out on the cell.
 * @return One message per fault, empty when the recipe can be delivered
 */
/**
 * @brief La generation avec les poses deplacees remplacees.
 * @details Un deplacement qui designe une passe ou un rang inexistant est ignore : la generation
 * a pu changer depuis qu'il a ete fait. Le dernier deplacement d'une meme pose gagne.
 * @param generated Passes generees, aplaties
 * @param moves Deplacements a appliquer
 */
std::vector<ToolPathSegment> applyPoseMoves(const std::vector<ToolPathSegment>& generated,
                                            const std::vector<PoseMove>& moves);

/**
 * @brief Retrouve, dans la generation, la pose la plus proche d'une position.
 * @param generated Passes ou chercher, deja deplacees si des deplacements existent
 * @param position Position visee
 * @param tolerance Distance au-dela de laquelle rien n'est retrouve
 * @param found Sortie : passe, rang et pose de la plus proche
 * @return Vrai quand une pose est a moins de `tolerance`
 */
bool locatePose(const std::vector<ToolPathSegment>& generated,
                const Eigen::Vector3d& position,
                double tolerance,
                PoseMove& found);

std::vector<std::string> validateRecipe(const std::vector<ToolPathSegment>& generated,
                                        const PathRecipe& recipe,
                                        std::size_t min_poses);

/**
 * @brief Fingerprints every pass of a generation.
 * @param generated The generated passes, flattened
 */
std::vector<SegmentFingerprint> fingerprintSegments(const std::vector<ToolPathSegment>& generated);

/**
 * @brief Passes whose fingerprint no longer matches the one the recipe was written against.
 * @param recorded Fingerprints stored with the recipe
 * @param current Fingerprints of the generation at hand
 * @param tolerance Largest position difference still counted as the same pass, in the mesh's length
 *                  unit. No default: the figure depends on what the caller considers the same
 *                  generation, and silently picking one would silently accept a stale recipe.
 * @return Indices of the passes that differ, plus every index beyond the shorter of the two lists.
 *         Empty when the recipe still describes this generation.
 */
std::vector<std::size_t> staleSegments(const std::vector<SegmentFingerprint>& recorded,
                                       const std::vector<SegmentFingerprint>& current,
                                       double tolerance);

/**
 * @brief Measures the joint that welding two assembled passes would create.
 * @details The waypoints that become neighbours are the second-to-last of @p before and the second
 * of @p after, because welding drops one waypoint on each side.
 * @param before Pass that would come first
 * @param after Pass that would be appended to it
 * @return The measurements, both negative when either pass is too short to be welded
 */
WeldMetrics computeWeldMetrics(const ToolPathSegment& before, const ToolPathSegment& after);

/**
 * @brief Les memes morceaux, parcourus a l'envers : ordre inverse ET chaque morceau retourne.
 * @details Ne faire que l'un des deux casserait la continuite de la passe.
 */
std::vector<SourceSegment> reverseSources(const std::vector<SourceSegment>& sources);

/**
 * @brief Enchaine toutes les passes livrees en une seule, du plus proche au plus proche.
 * @details Part de `start_pass` dans son sens de parcours, puis prend a chaque etape la passe
 * restante dont une extremite est la plus proche de la fin courante, retournee quand c'est sa
 * fin qui est la plus proche. C'est ce qui fait un U de trois bandes sans que l'outil decolle :
 * chaque raccord est une soudure, dont l'ecart reste a juger par l'operateur, aucun seuil ne
 * s'applique ici.
 * @param generated Passes de la generation
 * @param recipe Livraison courante, dont les soudures deja faites sont conservees
 * @param start_pass Rang, dans la livraison, de la passe par laquelle on commence
 * @return Une recette a une seule passe
 * @throws std::out_of_range si `start_pass` ne designe aucune passe livree
 */
PathRecipe chainPasses(const std::vector<ToolPathSegment>& generated,
                       const PathRecipe& recipe,
                       std::size_t start_pass);

/**
 * @brief Reads a recipe file.
 * @param file Path to the YAML file
 * @throws std::runtime_error if the file is missing, malformed, or of an unknown schema version
 */
RecipeFile readRecipeFile(const std::string& file);

/**
 * @brief Writes a recipe file.
 * @param file Path to the YAML file
 * @param content The recipe and the generation it was written against
 * @throws std::runtime_error if the file cannot be opened for writing
 */
void writeRecipeFile(const std::string& file, const RecipeFile& content);

}  // namespace noether
