#pragma once

#include <noether_cad/cad_model.h>
#include <noether_cad/step_reader.h>
#include <noether_gui/widgets/translator.h>

#include <Eigen/Core>
#include <QString>
#include <string>

namespace noether
{
/**
 * @brief Finesse de conversion d'un STEP en maillage, lue dans un fichier de configuration
 * @details Aucune valeur n'a de defaut : une finesse de maillage est un choix d'ingenierie,
 * fait une fois dans la configuration du projet, pas a l'ouverture de chaque piece.
 */
struct StepTessellation
{
  /** @brief Ecart maximal entre la surface exacte et ses triangles, en metres */
  double linear_deflection_m = 0.0;
  /** @brief Ecart angulaire maximal entre deux triangles voisins, en radians */
  double angular_deflection_rad = 0.0;
  /** @brief Solides a mailler : "all" pour tous, "one" pour refuser un fichier a plusieurs solides */
  std::string solids;
};

/**
 * @brief Lit la finesse de conversion d'un STEP
 * @param file Fichier YAML portant linear_deflection_m, angular_deflection_rad et solids
 * @return La finesse lue
 * @throws std::runtime_error si le fichier ne se lit pas, si une cle manque, si une deflexion
 * n'est pas un nombre fini strictement positif ou si solids ne vaut ni "all" ni "one"
 */
StepTessellation readStepTessellation(const std::string& file);

/** @brief Vrai pour un fichier .step ou .stp, quelle que soit la casse de l'extension */
bool isStepFile(const QString& file);

/**
 * @brief Prefixe des fichiers produits par la conversion : le STEP sans son extension
 * @details Le maillage et sa fiche YAML s'ecrivent a cote du STEP. La fiche porte la
 * transformation de recentrage, dont le recalage a besoin : elle doit rester avec le maillage.
 */
QString convertedMeshPrefix(const QString& step_file);

/** @brief Selection de solides correspondant a la cle solids de la configuration */
cad::SolidSelection solidSelectionOf(const StepTessellation& tessellation);

/** @brief Etendue d'une boite englobante, en metres, telle qu'elle est montree a l'operateur */
QString extentText(const Eigen::Vector3d& extent, Language language);

/**
 * @brief Unite retenue, nombre de triangles et etendue d'un STEP converti
 * @details L'etendue est le seul controle qui detecte une unite fausse : elle est montree a
 * l'operateur a chaque conversion.
 */
QString conversionSummary(const cad::CadModel& model, Language language);

/** @brief Resultat d'une conversion de STEP */
struct StepConversion
{
  /** @brief Chemin du PLY produit, a cote du STEP */
  QString ply_file;
  /** @brief Unite retenue, nombre de triangles et etendue dans les deux langues, voir ::conversionSummary */
  Bilingual summary;
};

/**
 * @brief Convertit un STEP en PLY et en fiche YAML a cote de lui
 * @details Un PLY et une fiche deja presents sous le meme prefixe sont remplaces. Le maillage
 * est recentre sur sa boite englobante ; la translation est ecrite dans la fiche.
 * @param step_file STEP a convertir
 * @param tessellation Finesse de la conversion
 * @return Le PLY produit et le resume de la conversion
 * @throws noether::cad::StepError si le STEP est inexploitable (unite, solides multiples...)
 * @throws std::runtime_error si le PLY ou la fiche ne peut etre ecrit
 */
StepConversion convertStepToPly(const QString& step_file, const StepTessellation& tessellation);

}  // namespace noether
