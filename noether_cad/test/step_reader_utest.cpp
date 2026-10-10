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

#include <noether_cad/mesh_writer.h>
#include <noether_cad/step_reader.h>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <limits>
#include <BRep_Builder.hxx>
#include <Interface_Static.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Compound.hxx>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace
{
using namespace noether::cad;

/** @brief Boite a poser dans un STEP de test, en millimetres */
struct Box
{
  /** @brief Coin minimal */
  Eigen::Vector3d corner;
  /** @brief Dimensions */
  Eigen::Vector3d size;
};

/** @brief Finesse grossiere : une boite n'a besoin que de deux triangles par face */
const TessellationSettings kCoarse{ 0.001, 0.5 };

/** @brief Dossier temporaire propre a ce processus de test */
std::filesystem::path testDirectory()
{
  const std::filesystem::path directory = std::filesystem::temp_directory_path() / "noether_cad_utest";
  std::filesystem::create_directories(directory);
  return directory;
}

/**
 * @brief Ecrit un STEP contenant des boites, un solide par boite
 * @param file_unit Unite declaree dans le fichier, "MM" ou "M" ; les nombres sont ecrits tels
 * quels, les boites doivent donc etre donnees dans cette unite
 */
std::string writeBoxStep(const std::string& name, const std::vector<Box>& boxes, const std::string& file_unit)
{
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const Box& box : boxes)
  {
    const gp_Pnt corner(box.corner.x(), box.corner.y(), box.corner.z());
    builder.Add(compound, BRepPrimAPI_MakeBox(corner, box.size.x(), box.size.y(), box.size.z()).Shape());
  }
  // Le writer reinitialise les parametres d'ecriture a sa creation : l'unite se fixe apres
  STEPControl_Writer writer;
  Interface_Static::SetCVal("xstep.cascade.unit", "MM");
  Interface_Static::SetCVal("write.step.unit", file_unit.c_str());
  writer.Transfer(compound, STEPControl_AsIs);
  const std::string path = (testDirectory() / name).string();
  writer.Write(path.c_str());
  return path;
}

/** @brief Ecrit un STEP en millimetres contenant une forme quelconque */
std::string writeShapeStep(const std::string& name, const TopoDS_Shape& shape)
{
  STEPControl_Writer writer;
  Interface_Static::SetCVal("xstep.cascade.unit", "MM");
  Interface_Static::SetCVal("write.step.unit", "MM");
  writer.Transfer(shape, STEPControl_AsIs);
  const std::string path = (testDirectory() / name).string();
  writer.Write(path.c_str());
  return path;
}

/** @brief Fichier texte de test, pour les cas que l'ecriture STEP ne produit pas */
std::string writeText(const std::string& name, const std::string& content)
{
  const std::string path = (testDirectory() / name).string();
  std::ofstream(path) << content;
  return path;
}

/** @brief Etendue de la boite englobante du maillage */
Eigen::Vector3d extentOf(const CadModel& model)
{
  const auto box = model.boundingBox();
  return box.second - box.first;
}

/** @brief Boite de 100 x 50 x 20 mm posee a (10, 20, 30) mm */
const Box kBox{ Eigen::Vector3d(10.0, 20.0, 30.0), Eigen::Vector3d(100.0, 50.0, 20.0) };
}  // namespace

/// Un fichier en millimetres donne un maillage en metres
TEST(StepReader, ConvertsMillimetersToMeters)
{
  const std::string step = writeBoxStep("box_mm.step", { kBox }, "MM");
  EXPECT_EQ(readDeclaredUnit(step), "mm");
  const CadModel model = buildModel(step, kCoarse, Recentering::NONE, "", SolidSelection());
  EXPECT_TRUE(extentOf(model).isApprox(Eigen::Vector3d(0.1, 0.05, 0.02), 1e-9));
  EXPECT_TRUE(model.boundingBox().first.isApprox(Eigen::Vector3d(0.01, 0.02, 0.03), 1e-9));
}

/// La meme piece ecrite en metres donne le meme maillage : l'unite est lue, pas supposee
TEST(StepReader, ReadsUnitFromFile)
{
  // La meme boite, decrite en metres dans un fichier qui declare le metre
  const Box box_in_meters{ kBox.corner * 1e-3, kBox.size * 1e-3 };
  const std::string step = writeBoxStep("box_m.step", { box_in_meters }, "M");
  EXPECT_EQ(readDeclaredUnit(step), "m");
  const CadModel model = buildModel(step, kCoarse, Recentering::NONE, "", SolidSelection());
  EXPECT_TRUE(extentOf(model).isApprox(Eigen::Vector3d(0.1, 0.05, 0.02), 1e-9));
  EXPECT_FALSE(model.unit_forced);
}

/// Une unite imposee reinterprete les nombres bruts du fichier
TEST(StepReader, AppliesForcedUnit)
{
  const std::string step = writeBoxStep("box_forced.step", { kBox }, "MM");
  const CadModel model = buildModel(step, kCoarse, Recentering::NONE, "m", SolidSelection());
  EXPECT_TRUE(model.unit_forced);
  EXPECT_TRUE(extentOf(model).isApprox(Eigen::Vector3d(100.0, 50.0, 20.0), 1e-9));
}

/// Le recentrage est une translation pure vers le centre de la boite, notee dans T_source_output
TEST(StepReader, RecentersOnBoundingBox)
{
  const std::string step = writeBoxStep("box_recenter.step", { kBox }, "MM");
  const CadModel model = buildModel(step, kCoarse, Recentering::BOUNDING_BOX, "", SolidSelection());
  const auto box = model.boundingBox();
  EXPECT_TRUE((box.first + box.second).isZero(1e-9));
  EXPECT_TRUE(model.T_source_output.translation().isApprox(Eigen::Vector3d(-0.06, -0.045, -0.04), 1e-9));
  EXPECT_TRUE(model.T_source_output.linear().isIdentity(1e-12));
}

/// Chaque face occupe un intervalle de triangles, les intervalles se suivent et couvrent le maillage
TEST(StepReader, BuildsFaceTable)
{
  const std::string step = writeBoxStep("box_faces.step", { kBox }, "MM");
  const CadModel model = buildModel(step, kCoarse, Recentering::BOUNDING_BOX, "", SolidSelection());
  ASSERT_EQ(model.faces.size(), 6u);
  int next = 0;
  double area = 0.0;
  for (const Face& face : model.faces)
  {
    EXPECT_EQ(face.type, FaceType::PLANE);
    EXPECT_EQ(face.triangle_begin, next);
    EXPECT_GT(face.triangle_end, face.triangle_begin);
    next = face.triangle_end;
    area += face.area;
  }
  EXPECT_EQ(next, static_cast<int>(model.mesh.triangles.size()));
  EXPECT_NEAR(area, 2.0 * (0.1 * 0.05 + 0.05 * 0.02 + 0.02 * 0.1), 1e-12);
}

/// Normales des plans et triangles orientes vers l'exterieur de la matiere
TEST(StepReader, OrientsNormalsOutward)
{
  const std::string step = writeBoxStep("box_normals.step", { kBox }, "MM");
  const CadModel model = buildModel(step, kCoarse, Recentering::BOUNDING_BOX, "", SolidSelection());
  for (const Face& face : model.faces)
  {
    // Boite centree sur l'origine : le centre d'une face sortante est du cote de sa normale
    EXPECT_GT(face.normal.dot(face.centroid), 0.0);
    for (int t = face.triangle_begin; t < face.triangle_end; ++t)
    {
      const Eigen::Vector3i& tri = model.mesh.triangles[static_cast<std::size_t>(t)];
      const Eigen::Vector3d& a = model.mesh.vertices[static_cast<std::size_t>(tri.x())];
      const Eigen::Vector3d& b = model.mesh.vertices[static_cast<std::size_t>(tri.y())];
      const Eigen::Vector3d& c = model.mesh.vertices[static_cast<std::size_t>(tri.z())];
      EXPECT_GT((b - a).cross(c - a).dot(face.normal), 0.0);
    }
  }
}

/// Les sommets des faces voisines sont fusionnes : une boite garde ses huit coins
TEST(StepReader, WeldsSharedVertices)
{
  const std::string step = writeBoxStep("box_weld.step", { kBox }, "MM");
  const CadModel model = buildModel(step, kCoarse, Recentering::NONE, "", SolidSelection());
  EXPECT_EQ(model.mesh.vertices.size(), 8u);
  EXPECT_EQ(model.mesh.triangles.size(), 12u);
}

/// Plusieurs solides sans selection : refus ; tous, une liste ou une exclusion : accepte
TEST(StepReader, SelectsSolids)
{
  const Box second{ Eigen::Vector3d(200.0, 0.0, 0.0), Eigen::Vector3d(10.0, 10.0, 10.0) };
  const std::string step = writeBoxStep("two_boxes.step", { kBox, second }, "MM");
  EXPECT_THROW(buildModel(step, kCoarse, Recentering::NONE, "", SolidSelection()), StepError);

  SolidSelection all;
  all.all = true;
  const CadModel both = buildModel(step, kCoarse, Recentering::NONE, "", all);
  EXPECT_EQ(both.solid_count, 2);
  EXPECT_EQ(both.faces.size(), 12u);

  SolidSelection only_second;
  only_second.kept = { 1 };
  const CadModel single = buildModel(step, kCoarse, Recentering::NONE, "", only_second);
  EXPECT_EQ(single.solid_count, 1);
  EXPECT_EQ(single.excluded_solids, std::vector<int>{ 0 });
  EXPECT_TRUE(extentOf(single).isApprox(Eigen::Vector3d(0.01, 0.01, 0.01), 1e-9));
}

/// Une selection qui cite un solide inexistant, ou qui ne retient rien, est refusee
TEST(StepReader, RefusesInvalidSolidSelection)
{
  const Box second{ Eigen::Vector3d(200.0, 0.0, 0.0), Eigen::Vector3d(10.0, 10.0, 10.0) };
  const std::string step = writeBoxStep("two_boxes_invalid.step", { kBox, second }, "MM");
  SolidSelection missing;
  missing.kept = { 5 };
  EXPECT_THROW(buildModel(step, kCoarse, Recentering::NONE, "", missing), StepError);
  SolidSelection nothing;
  nothing.all = true;
  nothing.excluded = { 0, 1 };
  EXPECT_THROW(buildModel(step, kCoarse, Recentering::NONE, "", nothing), StepError);
}

/// Une deflexion nulle est refusee : la finesse n'a pas de valeur par defaut
TEST(StepReader, RefusesNonPositiveDeflection)
{
  const std::string step = writeBoxStep("box_deflection.step", { kBox }, "MM");
  EXPECT_THROW(buildModel(step, TessellationSettings{ 0.0, 0.5 }, Recentering::NONE, "", SolidSelection()),
               StepError);
}

/// L'unite est lue dans l'entite LENGTH_UNIT : absente, multiple ou en pouces
TEST(StepReader, ParsesLengthUnitEntity)
{
  EXPECT_THROW(readDeclaredUnit(writeText("no_unit.step", "ISO-10303-21;\nDATA;\nENDSEC;\n")), StepError);
  EXPECT_THROW(readDeclaredUnit(writeText("two_units.step",
                                          "#1=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n"
                                          "#2=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );\n")),
               StepError);
  EXPECT_EQ(readDeclaredUnit(writeText("inch.step",
                                       "#1=( CONVERSION_BASED_UNIT('INCH',#2) LENGTH_UNIT() NAMED_UNIT(#3) );\n")),
            "in");
  EXPECT_THROW(readDeclaredUnit("/nonexistent/piece.step"), StepError);
}

/// readInfo compte solides et faces sans mailler
TEST(StepReader, DescribesContent)
{
  const Box second{ Eigen::Vector3d(200.0, 0.0, 0.0), Eigen::Vector3d(10.0, 10.0, 10.0) };
  const StepInfo info = readInfo(writeBoxStep("two_boxes_info.step", { kBox, second }, "MM"), "");
  EXPECT_EQ(info.declared_unit, "mm");
  EXPECT_EQ(info.solids.size(), 2u);
  EXPECT_EQ(info.face_count, 12);
  EXPECT_EQ(info.faces_by_type.at(FaceType::PLANE), 12);
}

/// Le PLY a la taille attendue : en-tete, 12 octets par sommet, 13 par triangle
TEST(MeshWriter, WritesBinaryPly)
{
  const CadModel model =
      buildModel(writeBoxStep("box_ply.step", { kBox }, "MM"), kCoarse, Recentering::NONE, "", SolidSelection());
  const std::string ply = (testDirectory() / "box.ply").string();
  writePly(ply, model.mesh);
  std::ifstream file(ply, std::ios::binary);
  std::string header;
  for (std::string line; std::getline(file, line) && line != "end_header";)
  {
    header += line + "\n";
  }
  const std::size_t header_size = header.size() + std::string("end_header\n").size();
  EXPECT_NE(header.find("element vertex 8\n"), std::string::npos);
  EXPECT_NE(header.find("element face 12\n"), std::string::npos);
  EXPECT_EQ(std::filesystem::file_size(ply), header_size + 8 * 12 + 12 * 13);
}

/// La fiche YAML se relit et porte la transformation, la finesse et la table des faces
TEST(MeshWriter, WritesReadableYaml)
{
  const CadModel model = buildModel(
      writeBoxStep("box_yaml.step", { kBox }, "MM"), kCoarse, Recentering::BOUNDING_BOX, "", SolidSelection());
  const std::string yaml = (testDirectory() / "box.yaml").string();
  writeYaml(yaml, model);
  const YAML::Node cad = YAML::LoadFile(yaml)["cad"];
  EXPECT_EQ(cad["declared_unit"].as<std::string>(), "mm");
  EXPECT_EQ(cad["recentering"].as<std::string>(), "bounding_box");
  EXPECT_NEAR(cad["T_source_output"]["translation"][0].as<double>(), -0.06, 1e-6);
  EXPECT_NEAR(cad["tessellation"]["linear_deflection_m"].as<double>(), 0.001, 1e-9);
  EXPECT_EQ(cad["tessellation"]["triangles"].as<int>(), 12);
  EXPECT_NEAR(cad["bounding_box_m"]["extent"][0].as<double>(), 0.1, 1e-6);
  ASSERT_EQ(cad["faces"].size(), 6u);
  EXPECT_EQ(cad["faces"][0]["type"].as<std::string>(), "plane");
  EXPECT_EQ(cad["faces"][0]["normal"].size(), 3u);
}

/// Un cylindre porte son axe et son rayon exacts ; le point d'axe suit le recentrage
TEST(StepReader, DescribesCylinder)
{
  // Cylindre de rayon 20 mm et de hauteur 60 mm, d'axe z passant par (100, 50) mm
  const gp_Ax2 axis(gp_Pnt(100.0, 50.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
  const std::string step = writeShapeStep("cylinder.step", BRepPrimAPI_MakeCylinder(axis, 20.0, 60.0).Shape());
  const CadModel model = buildModel(step, kCoarse, Recentering::BOUNDING_BOX, "", SolidSelection());
  int cylinders = 0;
  for (const Face& face : model.faces)
  {
    if (face.type != FaceType::CYLINDER)
    {
      continue;
    }
    ++cylinders;
    EXPECT_NEAR(face.radius, 0.02, 1e-9);
    EXPECT_NEAR(std::abs(face.axis_direction.z()), 1.0, 1e-9);
    // Recentre sur sa boite, le cylindre a son axe sur z : x et y du point d'axe sont nuls
    EXPECT_NEAR(face.axis_point.x(), 0.0, 1e-6);
    EXPECT_NEAR(face.axis_point.y(), 0.0, 1e-6);
  }
  EXPECT_EQ(cylinders, 1);
  // Les cordes passent sous la surface courbe : l'etendue en x et y ne depasse pas le diametre
  const Eigen::Vector3d extent = extentOf(model);
  EXPECT_NEAR(extent.z(), 0.06, 1e-9);
  EXPECT_LE(extent.x(), 0.04 + 1e-9);
  EXPECT_GT(extent.x(), 0.035);
}

/// Une sphere porte son centre et son rayon ; le centre suit le recentrage
TEST(StepReader, DescribesSphere)
{
  const std::string step =
      writeShapeStep("sphere.step", BRepPrimAPI_MakeSphere(gp_Pnt(300.0, -40.0, 10.0), 25.0).Shape());
  const CadModel model = buildModel(step, kCoarse, Recentering::BOUNDING_BOX, "", SolidSelection());
  ASSERT_FALSE(model.faces.empty());
  const Face& sphere = model.faces.front();
  EXPECT_EQ(sphere.type, FaceType::SPHERE);
  EXPECT_NEAR(sphere.radius, 0.025, 1e-9);
  // Translation appliquee : le centre exact, en metres, recentre sur la boite des triangles
  const Eigen::Vector3d expected_center = Eigen::Vector3d(0.3, -0.04, 0.01) + model.T_source_output.translation();
  EXPECT_TRUE(sphere.axis_point.isApprox(expected_center, 1e-9));
}

/// Les parametres d'un cylindre sont ecrits dans la fiche sous leurs cles
TEST(MeshWriter, WritesCurvedFaceParameters)
{
  const gp_Ax2 axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
  const std::string step = writeShapeStep("cylinder_yaml.step", BRepPrimAPI_MakeCylinder(axis, 20.0, 60.0).Shape());
  const CadModel model = buildModel(step, kCoarse, Recentering::NONE, "", SolidSelection());
  const std::string yaml = (testDirectory() / "cylinder.yaml").string();
  writeYaml(yaml, model);
  bool found = false;
  for (const YAML::Node& face : YAML::LoadFile(yaml)["cad"]["faces"])
  {
    if (face["type"].as<std::string>() == "cylinder")
    {
      found = true;
      EXPECT_NEAR(face["radius_m"].as<double>(), 0.02, 1e-6);
      EXPECT_EQ(face["axis_point"].size(), 3u);
      EXPECT_EQ(face["axis_direction"].size(), 3u);
    }
  }
  EXPECT_TRUE(found);
}

/// Toute deflexion nulle, negative, infinie ou NaN est refusee, lineaire comme angulaire
TEST(StepReader, RefusesInvalidDeflections)
{
  const std::string step = writeBoxStep("box_deflections.step", { kBox }, "MM");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const TessellationSettings& settings : { TessellationSettings{ 0.001, 0.0 }, TessellationSettings{ -0.001, 0.5 },
                                                TessellationSettings{ nan, 0.5 }, TessellationSettings{ 0.001, inf } })
  {
    EXPECT_THROW(buildModel(step, settings, Recentering::NONE, "", SolidSelection()), StepError);
  }
}

/// Un indice cite deux fois compte une fois ; une liste se combine avec une exclusion
TEST(StepReader, HandlesDuplicateAndExcludedSolids)
{
  const Box second{ Eigen::Vector3d(200.0, 0.0, 0.0), Eigen::Vector3d(10.0, 10.0, 10.0) };
  const std::string step = writeBoxStep("two_boxes_duplicates.step", { kBox, second }, "MM");
  SolidSelection duplicated;
  duplicated.kept = { 0, 0 };
  const CadModel first_only = buildModel(step, kCoarse, Recentering::NONE, "", duplicated);
  EXPECT_EQ(first_only.retained_solids, std::vector<int>{ 0 });
  EXPECT_EQ(first_only.faces.size(), 6u);

  SolidSelection listed_then_excluded;
  listed_then_excluded.kept = { 0, 1 };
  listed_then_excluded.excluded = { 0 };
  const CadModel second_only = buildModel(step, kCoarse, Recentering::NONE, "", listed_then_excluded);
  EXPECT_EQ(second_only.retained_solids, std::vector<int>{ 1 });
  EXPECT_EQ(second_only.excluded_solids, std::vector<int>{ 0 });
}

/// Une unite imposee ne dispense pas le fichier de declarer la sienne
TEST(StepReader, ForcedUnitStillNeedsFileUnit)
{
  const std::string step = writeText("forced_no_unit.step", "ISO-10303-21;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n");
  EXPECT_THROW(buildModel(step, kCoarse, Recentering::NONE, "mm", SolidSelection()), StepError);
}

/// Le centimetre est reconnu
TEST(StepReader, ConvertsCentimeters)
{
  EXPECT_DOUBLE_EQ(metersPerUnit("cm"), 0.01);
  EXPECT_THROW(metersPerUnit("ft"), StepError);
}
