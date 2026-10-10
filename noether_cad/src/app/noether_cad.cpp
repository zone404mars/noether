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

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using namespace noether::cad;

/** @brief Options `--cle valeur` de la ligne de commande */
using Options = std::map<std::string, std::string>;

/** @brief Aide affichee sur une ligne de commande incomplete */
void printUsage()
{
  std::cerr << "usage :\n"
            << "  noether_cad info <fichier.step> [--unit mm|cm|m|in]\n\n"
            << "  noether_cad mesh <fichier.step>\n"
            << "        --linear-deflection <metres>\n"
            << "        --angular-deflection <radians>\n"
            << "        [--output <prefixe>]                 defaut : le STEP sans son extension\n"
            << "        [--unit mm|cm|m|in]                  impose l'unite au lieu de lire celle du fichier\n"
            << "        [--recentering bounding_box|none]    defaut : bounding_box\n"
            << "        [--solids all|one|<i,j,...>]         defaut : one, un fichier a plusieurs solides\n"
            << "                                             est alors refuse ; indices de info\n"
            << "        [--excluded-solids <i,j,...>]        solides ecartes, avec all ou une liste\n\n"
            << "Les deflexions sont obligatoires : la finesse d'un maillage est un choix d'ingenierie.\n";
}

/**
 * @brief Lit les options `--cle valeur` a partir de l'argument `first`
 * @throws std::runtime_error sur un argument qui n'est pas une option, ou une option sans valeur
 */
Options parseOptions(const int argc, char** argv, const int first)
{
  Options options;
  for (int index = first; index < argc; index += 2)
  {
    const std::string key = argv[index];
    if (key.rfind("--", 0) != 0)
    {
      throw std::runtime_error("Argument inattendu : " + key);
    }
    if (index + 1 >= argc)
    {
      throw std::runtime_error("Option sans valeur : " + key);
    }
    options[key] = argv[index + 1];
  }
  return options;
}

/** @brief Valeur d'une option facultative, `fallback` si elle est absente */
std::string option(const Options& options, const std::string& key, const std::string& fallback)
{
  const auto found = options.find(key);
  return (found == options.end()) ? fallback : found->second;
}

/**
 * @brief Un indice de solide, entier en entier
 * @throws std::runtime_error si l'element n'est pas un entier, "1x" compris
 */
int parseIndex(const std::string& key, const std::string& item)
{
  std::size_t read = 0;
  int index = 0;
  try
  {
    index = std::stoi(item, &read);
  }
  catch (const std::exception&)
  {
    read = 0;
  }
  // stoi s'arrete au premier caractere non numerique : la longueur lue doit couvrir l'element
  if (read == 0 || read != item.size())
  {
    throw std::runtime_error("Indice de solide illisible pour " + key + " : " + item);
  }
  return index;
}

/**
 * @brief Liste d'indices `i,j,k` d'une option
 * @throws std::runtime_error si un element n'est pas un entier
 */
std::vector<int> indexList(const std::string& key, const std::string& text)
{
  std::vector<int> indices;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    indices.push_back(parseIndex(key, item));
  }
  return indices;
}

/**
 * @brief Selection de solides lue dans --solids et --excluded-solids
 * @throws std::runtime_error si --excluded-solids accompagne le mode one
 */
SolidSelection solidSelection(const Options& options)
{
  SolidSelection selection;
  const std::string mode = option(options, "--solids", "one");
  selection.all = (mode == "all");
  if (mode != "all" && mode != "one")
  {
    selection.kept = indexList("--solids", mode);
  }
  const auto excluded = options.find("--excluded-solids");
  if (excluded != options.end())
  {
    if (mode == "one")
    {
      throw std::runtime_error("--excluded-solids demande --solids all ou une liste");
    }
    selection.excluded = indexList("--excluded-solids", excluded->second);
  }
  return selection;
}

/**
 * @brief Option obligatoire convertie en reel
 * @throws std::runtime_error si elle est absente ou non numerique
 */
double requiredNumber(const Options& options, const std::string& key)
{
  const auto found = options.find(key);
  if (found == options.end())
  {
    throw std::runtime_error("Option obligatoire manquante : " + key);
  }
  try
  {
    return std::stod(found->second);
  }
  catch (const std::exception&)
  {
    throw std::runtime_error("Valeur non numerique pour " + key + " : " + found->second);
  }
}

/**
 * @brief Mode de recentrage lu dans --recentering
 * @throws std::runtime_error si le mode est inconnu
 */
Recentering recentering(const Options& options)
{
  const std::string mode = option(options, "--recentering", "bounding_box");
  if (mode == "none")
  {
    return Recentering::NONE;
  }
  if (mode != "bounding_box")
  {
    throw std::runtime_error("Recentrage inconnu : " + mode + " (attendu bounding_box ou none)");
  }
  return Recentering::BOUNDING_BOX;
}

/** @brief Sous-commande info : decrit le STEP sans rien ecrire */
int runInfo(const std::string& path, const Options& options)
{
  const StepInfo info = readInfo(path, option(options, "--unit", ""));
  std::cout << "fichier         : " << info.source << "\n";
  std::cout << "unite declaree  : " << info.declared_unit << "\n";
  std::cout << "solides         : " << info.solids.size() << "\n";
  for (const std::string& solid : info.solids)
  {
    std::cout << "                  " << solid << "\n";
  }
  std::cout << "faces           : " << info.face_count << "\n";
  for (const auto& entry : info.faces_by_type)
  {
    std::cout << "                  " << std::setw(9) << std::left << faceTypeName(entry.first) << " " << entry.second
              << "\n";
  }
  if (info.solids.size() > 1)
  {
    std::cout << "\nPlusieurs solides : mesh les refuse sans --solids all ou une liste.\n";
  }
  return 0;
}

/** @brief Ligne des solides ecartes, absente quand aucun ne l'est */
void printExcludedSolids(const CadModel& model)
{
  if (model.excluded_solids.empty())
  {
    return;
  }
  std::cout << "solides ecartes :";
  for (const int index : model.excluded_solids)
  {
    std::cout << " " << index;
  }
  std::cout << "\n";
}

/** @brief Compte rendu d'une conversion : unite, solides, taille et etendue du maillage */
void printReport(const CadModel& model)
{
  const auto box = model.boundingBox();
  const Eigen::Vector3d extent = box.second - box.first;
  std::cout << "unite retenue   : " << model.declared_unit
            << (model.unit_forced ? "  (imposee par l'appelant)" : "  (lue dans le fichier)") << "\n";
  std::cout << "solides mailles : " << model.solid_count << " sur " << model.solid_total
            << (model.solid_count > 1 ? "  (union, le solide n'est plus un regroupement)" : "") << "\n";
  printExcludedSolids(model);
  std::cout << "faces           : " << model.faces.size() << "\n";
  std::cout << "sommets         : " << model.mesh.vertices.size() << "\n";
  std::cout << "triangles       : " << model.mesh.triangles.size() << "\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "etendue (m)     : " << extent.x() << " x " << extent.y() << " x " << extent.z() << "\n";
  std::cout << std::setprecision(6);
  std::cout << "translation (m) : " << model.T_source_output.translation().transpose() << "\n";
  std::cout << "\nVERIFIEZ L'ETENDUE : c'est le seul controle qui detecte une unite fausse.\n\n";
}

/** @brief Sous-commande mesh : produit le PLY et sa fiche YAML */
int runMesh(const std::string& path, const Options& options)
{
  const TessellationSettings settings{ requiredNumber(options, "--linear-deflection"),
                                       requiredNumber(options, "--angular-deflection") };
  const std::filesystem::path step(path);
  const std::string prefix = option(options, "--output", (step.parent_path() / step.stem()).string());
  const CadModel model =
      buildModel(path, settings, recentering(options), option(options, "--unit", ""), solidSelection(options));
  printReport(model);
  const std::filesystem::path directory = std::filesystem::path(prefix).parent_path();
  if (!directory.empty())
  {
    std::filesystem::create_directories(directory);
  }
  writePly(prefix + ".ply", model.mesh);
  writeYaml(prefix + ".yaml", model);
  std::cout << "ecrit : " << prefix << ".ply\n";
  std::cout << "ecrit : " << prefix << ".yaml\n";
  return 0;
}

/** @brief Lance la sous-commande demandee ; 2 si elle est inconnue */
int runCommand(const std::string& command, const std::string& path, const Options& options)
{
  if (command == "info")
  {
    return runInfo(path, options);
  }
  if (command == "mesh")
  {
    return runMesh(path, options);
  }
  std::cerr << "Sous-commande inconnue : " << command << "\n\n";
  printUsage();
  return 2;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    printUsage();
    return 2;
  }
  try
  {
    return runCommand(argv[1], argv[2], parseOptions(argc, argv, 3));
  }
  catch (const StepError& error)
  {
    std::cerr << "STEP inexploitable : " << error.what() << "\n";
    return 3;
  }
  catch (const std::exception& error)
  {
    std::cerr << "Erreur : " << error.what() << "\n";
    return 4;
  }
}
