#pragma once

#include <noether_cad/cad_model.h>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace noether
{
namespace cad
{
/** @brief Erreur de lecture ou de contenu d'un fichier STEP */
class StepError : public std::runtime_error
{
public:
  explicit StepError(const std::string& message) : std::runtime_error(message) {}
};

/** @brief Ce que la lecture d'un STEP rapporte, sans tessellation ni ecriture */
struct StepInfo
{
  /** @brief Chemin du STEP lu */
  std::string source;
  /** @brief Unite retenue : "mm", "cm", "m" ou "in" */
  std::string declared_unit;
  /** @brief Description de chaque solide, dans l'ordre de parcours du fichier */
  std::vector<std::string> solids;
  /** @brief Nombre total de faces */
  int face_count = 0;
  /** @brief Nombre de faces par type analytique */
  std::map<FaceType, int> faces_by_type;
};

/** @brief Mode de recentrage du maillage */
enum class Recentering
{
  /** @brief Translation vers le centre de la boite englobante des triangles */
  BOUNDING_BOX,
  /** @brief Repere du STEP conserve tel quel */
  NONE
};

/**
 * @brief Solides a mailler dans un fichier qui en contient plusieurs
 * @details Les indices sont ceux de readInfo. Une selection vide sur un fichier a plusieurs
 * solides est refusee : le choix revient a l'operateur.
 */
struct SolidSelection
{
  /** @brief Vrai : tous les solides, moins `excluded`. Faux : les seuls `kept` */
  bool all = false;
  /** @brief Indices des solides retenus quand `all` est faux */
  std::vector<int> kept;
  /** @brief Indices des solides ecartes, avec `all` comme avec `kept` */
  std::vector<int> excluded;
};

/**
 * @brief Lit l'unite de longueur declaree dans un fichier STEP
 * @details L'unite est portee par une entite LENGTH_UNIT de la section DATA, pas par le HEADER.
 * @param path Fichier STEP
 * @return "mm", "cm", "m" ou "in"
 * @throws StepError si le fichier est illisible, s'il ne declare aucune unite de longueur, ou
 * s'il en declare plusieurs differentes
 */
std::string readDeclaredUnit(const std::string& path);

/**
 * @brief Facteur de conversion d'une unite vers le metre
 * @param unit "mm", "cm", "m" ou "in"
 * @throws StepError si l'unite n'est pas reconnue
 */
double metersPerUnit(const std::string& unit);

/**
 * @brief Lit un STEP et en rapporte le contenu, sans tessellation ni ecriture
 * @param path Fichier STEP
 * @param forced_unit "mm", "cm", "m" ou "in" pour imposer l'unite ; vide pour lire celle du fichier
 * @throws StepError si le fichier est illisible ou ne contient aucune geometrie
 */
StepInfo readInfo(const std::string& path, const std::string& forced_unit);

/**
 * @brief Chaine complete : lecture, mise en metres, tessellation, table des faces, recentrage
 * @details La mise en metres precede la tessellation : les deflexions s'expriment dans l'unite
 * du modele.
 * @param path Fichier STEP
 * @param settings Finesse de la tessellation
 * @param recentering Mode de recentrage ; la transformation appliquee est dans T_source_output
 * @param forced_unit Unite imposee ; vide pour lire celle du fichier
 * @param selection Solides a mailler
 * @throws StepError si le fichier est illisible, s'il a plusieurs solides sans selection, si un
 * indice de solide n'existe pas, ou si la tessellation ne produit aucun triangle
 */
CadModel buildModel(const std::string& path,
                    const TessellationSettings& settings,
                    Recentering recentering,
                    const std::string& forced_unit,
                    const SolidSelection& selection);

}  // namespace cad
}  // namespace noether
