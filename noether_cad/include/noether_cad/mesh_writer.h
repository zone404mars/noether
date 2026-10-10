#pragma once

#include <noether_cad/cad_model.h>

#include <string>

namespace noether
{
namespace cad
{
/**
 * @brief Ecrit le maillage en PLY binaire little-endian, sommets en metres
 * @throws std::runtime_error si le fichier ne peut etre ecrit
 */
void writePly(const std::string& path, const Mesh& mesh);

/**
 * @brief Ecrit la fiche du maillage : unite, solides, transformation, tessellation, boite
 * englobante et table des faces
 * @details La transformation est le champ critique : sans elle, un maillage recentre ne peut
 * plus etre rattache a ce qui a ete palpe sur la piece reelle.
 * @throws std::runtime_error si le fichier ne peut etre ecrit
 */
void writeYaml(const std::string& path, const CadModel& model);

}  // namespace cad
}  // namespace noether
