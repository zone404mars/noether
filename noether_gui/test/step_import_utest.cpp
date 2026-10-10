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

#include <QDir>
#include <QTemporaryDir>
#include <fstream>
#include <gtest/gtest.h>
#include <stdexcept>

namespace
{
/// @brief Ecrit un fichier de configuration STEP dans un dossier temporaire et rend son chemin
std::string writeConfig(const QTemporaryDir& dir, const std::string& content)
{
  const std::string file = QDir(dir.path()).filePath("step_tessellation.yaml").toStdString();
  std::ofstream(file) << content;
  return file;
}
}  // namespace

/// Une configuration complete se lit telle quelle
TEST(StepImport, ReadsCompleteConfiguration)
{
  QTemporaryDir dir;
  const std::string file =
      writeConfig(dir, "linear_deflection_m: 0.0005\nangular_deflection_rad: 0.2\nsolids: all\n");
  const noether::StepTessellation tessellation = noether::readStepTessellation(file);
  EXPECT_DOUBLE_EQ(tessellation.linear_deflection_m, 0.0005);
  EXPECT_DOUBLE_EQ(tessellation.angular_deflection_rad, 0.2);
  EXPECT_EQ(tessellation.solids, "all");
}

/// Aucune valeur n'a de defaut : une cle absente est refusee
TEST(StepImport, RefusesMissingKeys)
{
  QTemporaryDir dir;
  EXPECT_THROW(noether::readStepTessellation(writeConfig(dir, "angular_deflection_rad: 0.2\nsolids: all\n")),
               std::runtime_error);
  EXPECT_THROW(noether::readStepTessellation(writeConfig(dir, "linear_deflection_m: 0.0005\nsolids: all\n")),
               std::runtime_error);
  EXPECT_THROW(
      noether::readStepTessellation(writeConfig(dir, "linear_deflection_m: 0.0005\nangular_deflection_rad: 0.2\n")),
      std::runtime_error);
}

/// Une deflexion nulle ou negative, ou un mode de solides inconnu, est refuse
TEST(StepImport, RefusesInvalidValues)
{
  QTemporaryDir dir;
  EXPECT_THROW(noether::readStepTessellation(
                   writeConfig(dir, "linear_deflection_m: 0\nangular_deflection_rad: 0.2\nsolids: all\n")),
               std::runtime_error);
  EXPECT_THROW(noether::readStepTessellation(
                   writeConfig(dir, "linear_deflection_m: 0.0005\nangular_deflection_rad: -1\nsolids: all\n")),
               std::runtime_error);
  EXPECT_THROW(noether::readStepTessellation(
                   writeConfig(dir, "linear_deflection_m: 0.0005\nangular_deflection_rad: 0.2\nsolids: some\n")),
               std::runtime_error);
}

/// Un fichier illisible est refuse
TEST(StepImport, RefusesMissingFile)
{
  EXPECT_THROW(noether::readStepTessellation("/nonexistent/step_tessellation.yaml"), std::runtime_error);
}

/// L'extension STEP se reconnait quelle que soit sa casse
TEST(StepImport, RecognisesStepExtensions)
{
  EXPECT_TRUE(noether::isStepFile("/a/piece.step"));
  EXPECT_TRUE(noether::isStepFile("/a/piece.STEP"));
  EXPECT_TRUE(noether::isStepFile("/a/piece.stp"));
  EXPECT_TRUE(noether::isStepFile("/a/piece.Stp"));
  EXPECT_FALSE(noether::isStepFile("/a/piece.ply"));
  EXPECT_FALSE(noether::isStepFile("/a/piece.step.ply"));
}

/// Les fichiers produits s'ecrivent a cote du STEP, sous son nom sans extension
TEST(StepImport, WritesBesideTheStep)
{
  EXPECT_EQ(noether::convertedMeshPrefix("/a/b/Capot clim.v2.STEP"), "/a/b/Capot clim.v2");
}

/// La cle solids donne la selection de solides : all pour tous, one pour une selection vide
TEST(StepImport, MapsSolidsToSelection)
{
  noether::StepTessellation tessellation;
  tessellation.solids = "all";
  EXPECT_TRUE(noether::solidSelectionOf(tessellation).all);
  tessellation.solids = "one";
  const noether::cad::SolidSelection one = noether::solidSelectionOf(tessellation);
  EXPECT_FALSE(one.all);
  EXPECT_TRUE(one.kept.empty());
}

/// Le resume montre l'unite et son origine, le nombre de triangles et l'etendue, dans les deux langues
TEST(StepImport, SummarisesConversion)
{
  noether::cad::CadModel model;
  model.declared_unit = "mm";
  model.mesh.vertices = { Eigen::Vector3d(-0.25, -0.15, -0.05), Eigen::Vector3d(0.25, 0.15, 0.05),
                          Eigen::Vector3d(0.0, 0.0, 0.0) };
  model.mesh.triangles = { Eigen::Vector3i(0, 1, 2) };
  EXPECT_EQ(noether::conversionSummary(model, noether::Language::FRENCH),
            QString::fromUtf8("unité retenue : mm (lue dans le fichier) ; triangles : 1 ; "
                              "étendue (m) : 0.5000 x 0.3000 x 0.1000"));
  EXPECT_EQ(noether::conversionSummary(model, noether::Language::ENGLISH),
            "unit: mm (read from the file); triangles: 1; extent (m): 0.5000 x 0.3000 x 0.1000");
  model.unit_forced = true;
  EXPECT_NE(noether::conversionSummary(model, noether::Language::FRENCH).indexOf(QString::fromUtf8("(imposée)")), -1);
  EXPECT_NE(noether::conversionSummary(model, noether::Language::ENGLISH).indexOf("(forced)"), -1);
}

/// Une deflexion infinie ou non numerique est refusee
TEST(StepImport, RefusesNonFiniteOrNonNumeric)
{
  QTemporaryDir dir;
  EXPECT_THROW(noether::readStepTessellation(
                   writeConfig(dir, "linear_deflection_m: .inf\nangular_deflection_rad: 0.2\nsolids: all\n")),
               std::runtime_error);
  EXPECT_THROW(noether::readStepTessellation(
                   writeConfig(dir, "linear_deflection_m: fin\nangular_deflection_rad: 0.2\nsolids: all\n")),
               std::runtime_error);
}
