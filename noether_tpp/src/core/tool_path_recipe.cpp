/**
 * @file tool_path_recipe.cpp
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
#include <noether_tpp/core/tool_path_recipe.h>
#include <limits>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace noether
{
namespace
{
/**
 * @brief Schema version this code writes.
 * @details La version 1 n'avait pas de deplacements de poses ; elle se lit encore, sans
 * deplacement. Une version plus recente que celle-ci est refusee : elle porterait des champs
 * que cette version ignorerait en silence.
 */
const int kSchemaVersion = 2;
/** @brief Plus ancienne version encore lue */
const int kOldestReadableSchemaVersion = 1;

/** @brief Digits kept when writing a fingerprint position */
const int kFingerprintPrecision = 12;

/** @brief Header written at the top of a recipe file, for whoever opens it */
const char* kFileHeader =
    "# Retouche manuelle d'un chemin outil genere.\n"
    "#\n"
    "# Le fichier de poses reste une donnee DERIVEE : il se regenere. Ce fichier-ci dit\n"
    "# comment on passe de la generation a ce qui est livre, et rien d'autre.\n"
    "#\n"
    "# `passes` porte une entree par passe livree, dans l'ordre d'execution. Chaque\n"
    "# entree liste les passes de la generation qu'elle enchaine, avec leur sens de\n"
    "# parcours. Une passe generee absente d'ici est supprimee de la livraison.\n"
    "#\n"
    "# `source` est l'empreinte de la generation sur laquelle la retouche a ete definie.\n"
    "# Les indices ne designent les bonnes passes QUE pour cette generation-la : rejouer\n"
    "# la recette sur une autre est refuse plutot que devine.\n";

/// @brief Appends a piece to a pass, dropping the standoff waypoint on each side of the joint.
/// @param pass Pass being assembled, extended in place
/// @param piece Generated pass to append, already turned the right way round
void weldOnto(ToolPathSegment& pass, const ToolPathSegment& piece)
{
  if (piece.empty())
  {
    return;
  }
  if (pass.empty())
  {
    pass = piece;
    return;
  }
  // La derniere pose de la passe est son retrait, la premiere de la piece son approche :
  // toutes deux decollees de la surface. Les garder placerait un aller-retour a vide au
  // milieu d'une passe qui doit rester au contact.
  pass.pop_back();
  pass.insert(pass.end(), piece.begin() + 1, piece.end());
}

/// @brief Refuses a source that does not designate an unused generated pass.
/// @param generated_count Number of generated passes
/// @param used In/out: which generated passes a previous entry already took
/// @param source Source being checked
/// @param pass_index Index of the delivered pass, quoted in the message
/// @throws std::runtime_error if the index is out of range or already taken
void checkSource(std::size_t generated_count,
                 std::vector<char>& used,
                 const SourceSegment& source,
                 std::size_t pass_index)
{
  const std::string where = " (delivered pass " + std::to_string(pass_index + 1) + ")";
  if (source.segment >= generated_count)
  {
    throw std::runtime_error("Recipe refers to generated pass " + std::to_string(source.segment) +
                             ", but the generation holds " + std::to_string(generated_count) + where);
  }
  if (used[source.segment] != 0)
  {
    throw std::runtime_error("Generated pass " + std::to_string(source.segment) + " is used more than once" +
                             where);
  }
  used[source.segment] = 1;
}

/// @brief Assembles one delivered pass from the generated passes it lists.
/// @param generated The generated passes, flattened
/// @param sources What this pass takes from them
/// @param used In/out: which generated passes are already taken
/// @param pass_index Index of the delivered pass, quoted in refusal messages
/// @return The assembled pass
ToolPathSegment assembleOnePass(const std::vector<ToolPathSegment>& generated,
                                const std::vector<SourceSegment>& sources,
                                std::vector<char>& used,
                                std::size_t pass_index)
{
  ToolPathSegment pass;
  for (std::size_t i = 0; i < sources.size(); ++i)
  {
    checkSource(generated.size(), used, sources[i], pass_index);
    ToolPathSegment piece = generated[sources[i].segment];
    if (sources[i].reversed)
    {
      std::reverse(piece.begin(), piece.end());
    }
    weldOnto(pass, piece);
  }
  return pass;
}

/// @brief Whether two fingerprints describe the same generated pass.
/// @param recorded Fingerprint stored with the recipe
/// @param current Fingerprint of the generation at hand
/// @param tolerance Largest position difference still counted as the same pass
bool sameSegment(const SegmentFingerprint& recorded, const SegmentFingerprint& current, double tolerance)
{
  if (recorded.pose_count != current.pose_count)
  {
    return false;
  }
  return (recorded.first - current.first).norm() <= tolerance &&
         (recorded.last - current.last).norm() <= tolerance;
}

/// @brief Reads a three-number sequence into a position.
/// @param node Node expected to hold exactly three numbers
/// @param what Field name, quoted in the refusal message
/// @throws std::runtime_error if the node is not a sequence of three numbers
Eigen::Vector3d readPosition(const YAML::Node& node, const std::string& what)
{
  if (!node.IsSequence() || node.size() != 3)
  {
    throw std::runtime_error("Recipe field '" + what + "' must hold exactly three numbers.");
  }
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

/// @brief Writes one fingerprint, on a single line.
/// @param out Emitter to write to
/// @param print Fingerprint to write
void emitFingerprint(YAML::Emitter& out, const SegmentFingerprint& print)
{
  out << YAML::Flow << YAML::BeginMap;
  out << YAML::Key << "poses" << YAML::Value << print.pose_count;
  out << YAML::Key << "first" << YAML::Value << YAML::Flow << YAML::BeginSeq << print.first.x()
      << print.first.y() << print.first.z() << YAML::EndSeq;
  out << YAML::Key << "last" << YAML::Value << YAML::Flow << YAML::BeginSeq << print.last.x() << print.last.y()
      << print.last.z() << YAML::EndSeq;
  out << YAML::EndMap;
}

/// @brief Writes one delivered pass, on a single line.
/// @param out Emitter to write to
/// @param pass What the pass takes from the generation
void emitPass(YAML::Emitter& out, const std::vector<SourceSegment>& pass)
{
  out << YAML::Flow << YAML::BeginSeq;
  for (std::size_t i = 0; i < pass.size(); ++i)
  {
    out << YAML::Flow << YAML::BeginMap;
    out << YAML::Key << "segment" << YAML::Value << pass[i].segment;
    out << YAML::Key << "reversed" << YAML::Value << pass[i].reversed;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

}  // namespace

std::vector<ToolPathSegment> flattenToolPaths(const std::vector<ToolPaths>& tool_paths)
{
  std::vector<ToolPathSegment> flat;
  for (const ToolPaths& fragment : tool_paths)
  {
    for (const ToolPath& tool_path : fragment)
    {
      for (const ToolPathSegment& segment : tool_path)
      {
        flat.push_back(segment);
      }
    }
  }
  return flat;
}

std::vector<ToolPaths> nestToolPath(const ToolPath& passes)
{
  ToolPaths fragment;
  fragment.push_back(passes);
  std::vector<ToolPaths> nested;
  nested.push_back(fragment);
  return nested;
}

PathRecipe identityRecipe(std::size_t segment_count)
{
  PathRecipe recipe;
  for (std::size_t i = 0; i < segment_count; ++i)
  {
    SourceSegment source;
    source.segment = i;
    source.reversed = false;
    recipe.push_back(std::vector<SourceSegment>(1, source));
  }
  return recipe;
}

ToolPath assemblePasses(const std::vector<ToolPathSegment>& generated, const PathRecipe& recipe)
{
  std::vector<char> used(generated.size(), 0);
  ToolPath passes;
  for (std::size_t p = 0; p < recipe.size(); ++p)
  {
    if (recipe[p].empty())
    {
      throw std::runtime_error("Delivered pass " + std::to_string(p + 1) + " of the recipe lists no source.");
    }
    passes.push_back(assembleOnePass(generated, recipe[p], used, p));
  }
  return passes;
}

std::vector<std::string> validateRecipe(const std::vector<ToolPathSegment>& generated,
                                        const PathRecipe& recipe,
                                        std::size_t min_poses)
{
  std::vector<std::string> faults;
  ToolPath passes;
  try
  {
    passes = assemblePasses(generated, recipe);
  }
  catch (const std::exception& ex)
  {
    faults.push_back(ex.what());
    return faults;
  }

  if (passes.empty())
  {
    faults.push_back("The recipe delivers no pass at all.");
    return faults;
  }
  for (std::size_t p = 0; p < passes.size(); ++p)
  {
    if (passes[p].size() < min_poses)
    {
      faults.push_back("Delivered pass " + std::to_string(p + 1) + " carries " +
                       std::to_string(passes[p].size()) + " waypoint(s); at least " +
                       std::to_string(min_poses) + " are needed.");
    }
  }
  return faults;
}

std::vector<SegmentFingerprint> fingerprintSegments(const std::vector<ToolPathSegment>& generated)
{
  std::vector<SegmentFingerprint> prints;
  for (std::size_t i = 0; i < generated.size(); ++i)
  {
    SegmentFingerprint print;
    print.pose_count = generated[i].size();
    if (!generated[i].empty())
    {
      print.first = generated[i].front().translation();
      print.last = generated[i].back().translation();
    }
    prints.push_back(print);
  }
  return prints;
}

std::vector<std::size_t> staleSegments(const std::vector<SegmentFingerprint>& recorded,
                                       const std::vector<SegmentFingerprint>& current,
                                       double tolerance)
{
  std::vector<std::size_t> stale;
  const std::size_t common = std::min(recorded.size(), current.size());
  for (std::size_t i = 0; i < common; ++i)
  {
    if (!sameSegment(recorded[i], current[i], tolerance))
    {
      stale.push_back(i);
    }
  }
  // Une generation qui n'a pas le meme nombre de passes ne peut pas porter la recette :
  // tout ce qui depasse de part et d'autre est signale comme perime.
  for (std::size_t i = common; i < std::max(recorded.size(), current.size()); ++i)
  {
    stale.push_back(i);
  }
  return stale;
}

WeldMetrics computeWeldMetrics(const ToolPathSegment& before, const ToolPathSegment& after)
{
  WeldMetrics metrics;
  if (before.size() < 2 || after.size() < 2)
  {
    return metrics;
  }
  // Ce sont ces deux poses qui deviennent voisines : la soudure retire celle qui les separe
  // de chaque cote.
  const Eigen::Isometry3d& tail = before[before.size() - 2];
  const Eigen::Isometry3d& head = after[1];
  metrics.gap = (head.translation() - tail.translation()).norm();
  const double cosine = tail.rotation().col(2).dot(head.rotation().col(2));
  metrics.angle = std::acos(std::max(-1.0, std::min(1.0, cosine)));
  return metrics;
}

std::vector<SourceSegment> reverseSources(const std::vector<SourceSegment>& sources)
{
  std::vector<SourceSegment> reversed(sources.rbegin(), sources.rend());
  for (std::size_t i = 0; i < reversed.size(); ++i)
  {
    reversed[i].reversed = !reversed[i].reversed;
  }
  return reversed;
}

namespace
{
/// @brief La passe restante la plus proche de `end`, et si on l'aborde par sa fin.
/// @param passes Passes livrees assemblees
/// @param remaining Rangs encore a enchainer, non vide
std::pair<std::size_t, bool> nearestPass(const ToolPath& passes,
                                         const std::vector<std::size_t>& remaining,
                                         const Eigen::Vector3d& end)
{
  std::pair<std::size_t, bool> best(remaining.front(), false);
  double best_distance = std::numeric_limits<double>::infinity();
  for (const std::size_t index : remaining)
  {
    const double to_start = (passes[index].front().translation() - end).norm();
    const double to_end = (passes[index].back().translation() - end).norm();
    if (to_start < best_distance)
    {
      best_distance = to_start;
      best = std::make_pair(index, false);
    }
    if (to_end < best_distance)
    {
      best_distance = to_end;
      best = std::make_pair(index, true);
    }
  }
  return best;
}

/// @brief Les rangs des passes livrees autres que `start_pass`, dans l'ordre.
std::vector<std::size_t> otherPasses(const std::size_t count, const std::size_t start_pass)
{
  std::vector<std::size_t> others;
  for (std::size_t p = 0; p < count; ++p)
  {
    if (p != start_pass)
    {
      others.push_back(p);
    }
  }
  return others;
}
}  // namespace

PathRecipe chainPasses(const std::vector<ToolPathSegment>& generated,
                       const PathRecipe& recipe,
                       const std::size_t start_pass)
{
  if (start_pass >= recipe.size())
  {
    throw std::out_of_range("No delivered pass at rank " + std::to_string(start_pass + 1));
  }
  const ToolPath passes = assemblePasses(generated, recipe);

  std::vector<SourceSegment> chain = recipe[start_pass];
  Eigen::Vector3d end = passes[start_pass].back().translation();
  std::vector<std::size_t> remaining = otherPasses(recipe.size(), start_pass);
  while (!remaining.empty())
  {
    const std::pair<std::size_t, bool> next = nearestPass(passes, remaining, end);
    const std::vector<SourceSegment> sources = next.second ? reverseSources(recipe[next.first]) : recipe[next.first];
    chain.insert(chain.end(), sources.begin(), sources.end());
    end = next.second ? passes[next.first].front().translation() : passes[next.first].back().translation();
    remaining.erase(std::find(remaining.begin(), remaining.end(), next.first));
  }
  return PathRecipe(1, chain);
}

namespace
{
/// @brief Reads the fingerprints of the generation a recipe was written against.
/// @param root Root node of the recipe file
/// @throws std::runtime_error if the sequence is missing or malformed
std::vector<SegmentFingerprint> readFingerprints(const YAML::Node& root)
{
  const YAML::Node segments = root["source"]["segments"];
  if (!segments.IsSequence())
  {
    throw std::runtime_error("Recipe field 'source.segments' is missing or is not a sequence.");
  }

  std::vector<SegmentFingerprint> prints;
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    SegmentFingerprint print;
    print.pose_count = segments[i]["poses"].as<std::size_t>();
    print.first = readPosition(segments[i]["first"], "source.segments.first");
    print.last = readPosition(segments[i]["last"], "source.segments.last");
    prints.push_back(print);
  }
  return prints;
}

/// @brief Reads what every delivered pass takes from the generation.
/// @param root Root node of the recipe file
/// @throws std::runtime_error if the sequence is missing or malformed
PathRecipe readPasses(const YAML::Node& root)
{
  const YAML::Node passes = root["passes"];
  if (!passes.IsSequence())
  {
    throw std::runtime_error("Recipe field 'passes' is missing or is not a sequence.");
  }

  PathRecipe recipe;
  for (std::size_t p = 0; p < passes.size(); ++p)
  {
    std::vector<SourceSegment> sources;
    for (std::size_t s = 0; s < passes[p].size(); ++s)
    {
      SourceSegment source;
      source.segment = passes[p][s]["segment"].as<std::size_t>();
      source.reversed = passes[p][s]["reversed"].as<bool>();
      sources.push_back(source);
    }
    recipe.push_back(sources);
  }
  return recipe;
}


/// @brief Lit les poses deplacees a la main ; absent dans les fichiers de version 1.
/// @param root Racine du fichier
/// @throws std::runtime_error si le champ existe mais n'est pas une sequence
std::vector<PoseMove> readMoves(const YAML::Node& root)
{
  std::vector<PoseMove> moves;
  const YAML::Node node = root["moves"];
  if (!node)
  {
    return moves;
  }
  if (!node.IsSequence())
  {
    throw std::runtime_error("Recipe field 'moves' is not a sequence.");
  }
  for (std::size_t m = 0; m < node.size(); ++m)
  {
    PoseMove move;
    move.segment = node[m]["segment"].as<std::size_t>();
    move.index = node[m]["index"].as<std::size_t>();
    move.pose.translation() = readPosition(node[m]["position"], "moves.position");
    const YAML::Node q = node[m]["quaternion"];
    if (!q.IsSequence() || q.size() != 4)
    {
      throw std::runtime_error("Recipe field 'moves.quaternion' must hold exactly four numbers (x, y, z, w).");
    }
    move.pose.linear() =
        Eigen::Quaterniond(q[3].as<double>(), q[0].as<double>(), q[1].as<double>(), q[2].as<double>()).toRotationMatrix();
    moves.push_back(move);
  }
  return moves;
}
}  // namespace

RecipeFile readRecipeFile(const std::string& file)
{
  // Tout le parcours est enveloppe, pas seulement l'ouverture : une cle absente ou d'un
  // type inattendu leve une exception yaml-cpp dont le message ne nomme pas le fichier,
  // et l'operateur la lirait sans savoir lequel des siens est en cause.
  try
  {
    const YAML::Node root = YAML::LoadFile(file);

    const int version = root["schema_version"] ? root["schema_version"].as<int>() : 0;
    if (version < kOldestReadableSchemaVersion || version > kSchemaVersion)
    {
      throw std::runtime_error("schema version " + std::to_string(version) + "; this build reads versions " +
                               std::to_string(kOldestReadableSchemaVersion) + " to " + std::to_string(kSchemaVersion));
    }

    RecipeFile content;
    content.source = readFingerprints(root);
    content.recipe = readPasses(root);
    content.moves = readMoves(root);
    return content;
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error("Cannot read recipe file '" + file + "': " + ex.what());
  }
}

namespace
{
/// @brief Ecrit un deplacement de pose sur une ligne : passe, rang, position, quaternion (x, y, z, w).
/// @param out Emetteur
/// @param move Deplacement a ecrire
void emitMove(YAML::Emitter& out, const PoseMove& move)
{
  const Eigen::Quaterniond q(move.pose.linear());
  const Eigen::Vector3d& t = move.pose.translation();
  out << YAML::Flow << YAML::BeginMap;
  out << YAML::Key << "segment" << YAML::Value << move.segment;
  out << YAML::Key << "index" << YAML::Value << move.index;
  out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginSeq << t.x() << t.y() << t.z()
      << YAML::EndSeq;
  out << YAML::Key << "quaternion" << YAML::Value << YAML::Flow << YAML::BeginSeq << q.x() << q.y() << q.z() << q.w()
      << YAML::EndSeq;
  out << YAML::EndMap;
}

/// @brief Writes a whole recipe into an emitter.
/// @param out Emitter to write to
/// @param content The recipe and the generation it was written against
void emitRecipe(YAML::Emitter& out, const RecipeFile& content)
{
  out << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << kSchemaVersion;
  out << YAML::Key << "source" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "segments" << YAML::Value << YAML::BeginSeq;
  for (std::size_t i = 0; i < content.source.size(); ++i)
  {
    emitFingerprint(out, content.source[i]);
  }
  out << YAML::EndSeq << YAML::EndMap;
  out << YAML::Key << "passes" << YAML::Value << YAML::BeginSeq;
  for (std::size_t p = 0; p < content.recipe.size(); ++p)
  {
    emitPass(out, content.recipe[p]);
  }
  out << YAML::EndSeq;
  out << YAML::Key << "moves" << YAML::Value << YAML::BeginSeq;
  for (std::size_t m = 0; m < content.moves.size(); ++m)
  {
    emitMove(out, content.moves[m]);
  }
  out << YAML::EndSeq << YAML::EndMap;
}

}  // namespace

std::vector<ToolPathSegment> applyPoseMoves(const std::vector<ToolPathSegment>& generated,
                                            const std::vector<PoseMove>& moves)
{
  std::vector<ToolPathSegment> moved = generated;
  for (const PoseMove& move : moves)
  {
    if (move.segment < moved.size() && move.index < moved[move.segment].size())
    {
      moved[move.segment][move.index] = move.pose;
    }
  }
  return moved;
}

bool locatePose(const std::vector<ToolPathSegment>& generated,
                const Eigen::Vector3d& position,
                const double tolerance,
                PoseMove& found)
{
  double best = tolerance * tolerance;
  bool any = false;
  for (std::size_t s = 0; s < generated.size(); ++s)
  {
    for (std::size_t i = 0; i < generated[s].size(); ++i)
    {
      const double distance = (generated[s][i].translation() - position).squaredNorm();
      if (distance <= best)
      {
        best = distance;
        found.segment = s;
        found.index = i;
        found.pose = generated[s][i];
        any = true;
      }
    }
  }
  return any;
}

void writeRecipeFile(const std::string& file, const RecipeFile& content)
{
  YAML::Emitter out;
  out.SetDoublePrecision(kFingerprintPrecision);
  emitRecipe(out, content);

  std::ofstream stream(file.c_str());
  if (!stream)
  {
    throw std::runtime_error("Cannot open recipe file for writing: " + file);
  }
  stream << kFileHeader << out.c_str() << "\n";
}

}  // namespace noether
