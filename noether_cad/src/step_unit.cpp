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

#include <noether_cad/step_reader.h>

#include <fstream>
#include <iterator>

namespace
{
/**
 * @brief Unite d'une entite STEP qui contient LENGTH_UNIT, vide si elle n'en definit pas
 * @details Exemple d'entite : ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )
 */
std::string unitOfEntity(const std::string& entity)
{
  if (entity.find("CONVERSION_BASED_UNIT") != std::string::npos)
  {
    return entity.find("INCH") != std::string::npos ? "in" : "";
  }
  if (entity.find(".MILLI.") != std::string::npos)
  {
    return "mm";
  }
  if (entity.find(".CENTI.") != std::string::npos)
  {
    return "cm";
  }
  if (entity.find(".METRE.") != std::string::npos)
  {
    return "m";
  }
  return "";
}

/** @brief Texte de l'entite STEP qui contient la position `hit` : du point-virgule precedent au suivant */
std::string entityAround(const std::string& content, const std::size_t hit)
{
  const std::size_t begin = content.rfind(';', hit);
  const std::size_t end = content.find(';', hit);
  const std::size_t from = (begin == std::string::npos) ? 0 : begin + 1;
  const std::size_t to = (end == std::string::npos) ? content.size() : end;
  return content.substr(from, to - from);
}

/** @brief Contenu texte complet d'un fichier */
std::string readText(const std::string& path)
{
  std::ifstream file(path);
  if (!file)
  {
    throw noether::cad::StepError("Fichier illisible : " + path);
  }
  return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}
}  // namespace

namespace noether
{
namespace cad
{
std::string readDeclaredUnit(const std::string& path)
{
  // Lecture en texte plutot que par OpenCASCADE : l'API qui expose l'unite change entre
  // versions d'OCCT, le format de l'entite est fige par la norme
  const std::string content = readText(path);
  std::string found;
  for (std::size_t hit = content.find("LENGTH_UNIT"); hit != std::string::npos;
       hit = content.find("LENGTH_UNIT", hit + 1))
  {
    const std::string unit = unitOfEntity(entityAround(content, hit));
    if (unit.empty() || unit == found)
    {
      continue;
    }
    if (!found.empty())
    {
      throw StepError("Le fichier declare plusieurs unites de longueur ('" + found + "' et '" + unit +
                      "') : fichier non pris en charge");
    }
    found = unit;
  }
  if (found.empty())
  {
    throw StepError("Aucune unite de longueur declaree dans " + path);
  }
  return found;
}

double metersPerUnit(const std::string& unit)
{
  if (unit == "mm")
  {
    return 1e-3;
  }
  if (unit == "cm")
  {
    return 1e-2;
  }
  if (unit == "m")
  {
    return 1.0;
  }
  if (unit == "in")
  {
    return 0.0254;
  }
  throw StepError("Unite inconnue : '" + unit + "' (attendu mm, cm, m ou in)");
}

}  // namespace cad
}  // namespace noether
