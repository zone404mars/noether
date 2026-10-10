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

#include <noether_gui/widgets/step_import.h>
#include <noether_cad/mesh_writer.h>

#include <QDir>
#include <QFileInfo>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace
{
/// @brief Deflexion lue sous une cle, refusee si absente, non numerique, infinie ou non positive
double readDeflection(const YAML::Node& root, const std::string& key)
{
  if (!root[key])
  {
    throw std::runtime_error("Cle '" + key + "' absente de la configuration STEP");
  }
  double value = 0.0;
  try
  {
    value = root[key].as<double>();
  }
  catch (const YAML::Exception&)
  {
    throw std::runtime_error("'" + key + "' n'est pas un nombre dans la configuration STEP");
  }
  if (!std::isfinite(value) || !(value > 0.0))
  {
    throw std::runtime_error("'" + key + "' doit etre un nombre fini strictement positif");
  }
  return value;
}
}  // namespace

namespace noether
{
StepTessellation readStepTessellation(const std::string& file)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(file);
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error("Configuration STEP illisible (" + file + ") : " + ex.what());
  }
  StepTessellation tessellation;
  tessellation.linear_deflection_m = readDeflection(root, "linear_deflection_m");
  tessellation.angular_deflection_rad = readDeflection(root, "angular_deflection_rad");
  tessellation.solids = (root["solids"] && root["solids"].IsScalar()) ? root["solids"].Scalar() : std::string();
  if (tessellation.solids != "all" && tessellation.solids != "one")
  {
    throw std::runtime_error("'solids' doit valoir all ou one dans la configuration STEP");
  }
  return tessellation;
}

bool isStepFile(const QString& file)
{
  const QString suffix = QFileInfo(file).suffix().toLower();
  return suffix == "step" || suffix == "stp";
}

QString convertedMeshPrefix(const QString& step_file)
{
  const QFileInfo info(step_file);
  return info.dir().filePath(info.completeBaseName());
}

cad::SolidSelection solidSelectionOf(const StepTessellation& tessellation)
{
  // "one" laisse la selection vide : un fichier a plusieurs solides est alors refuse
  cad::SolidSelection selection;
  selection.all = (tessellation.solids == "all");
  return selection;
}

QString extentText(const Eigen::Vector3d& extent, const Language language)
{
  return QString(language == Language::FRENCH ? "étendue (m) : %1 x %2 x %3" : "extent (m): %1 x %2 x %3")
      .arg(extent.x(), 0, 'f', 4)
      .arg(extent.y(), 0, 'f', 4)
      .arg(extent.z(), 0, 'f', 4);
}

QString conversionSummary(const cad::CadModel& model, const Language language)
{
  const auto box = model.boundingBox();
  const bool french = (language == Language::FRENCH);
  const QString forced = french ? QString::fromUtf8("imposée") : QString("forced");
  const QString read = french ? QString("lue dans le fichier") : QString("read from the file");
  const QString format =
      french ? QString::fromUtf8("unité retenue : %1 (%2) ; triangles : %3 ; %4") : QString("unit: %1 (%2); triangles: %3; %4");
  return format.arg(QString::fromStdString(model.declared_unit))
      .arg(model.unit_forced ? forced : read)
      .arg(model.mesh.triangles.size())
      .arg(extentText(box.second - box.first, language));
}

StepConversion convertStepToPly(const QString& step_file, const StepTessellation& tessellation)
{
  const cad::TessellationSettings settings{ tessellation.linear_deflection_m, tessellation.angular_deflection_rad };
  // Unite toujours lue dans le fichier : la GUI n'offre pas de l'imposer
  const cad::CadModel model = cad::buildModel(
      step_file.toStdString(), settings, cad::Recentering::BOUNDING_BOX, "", solidSelectionOf(tessellation));
  const std::string prefix = convertedMeshPrefix(step_file).toStdString();
  cad::writePly(prefix + ".ply", model.mesh);
  cad::writeYaml(prefix + ".yaml", model);
  return { QString::fromStdString(prefix + ".ply"),
           { conversionSummary(model, Language::FRENCH), conversionSummary(model, Language::ENGLISH) } };
}

}  // namespace noether
