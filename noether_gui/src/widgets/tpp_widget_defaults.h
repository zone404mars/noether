#pragma once

#include <cstddef>
#include <vtkSmartPointer.h>

class vtkActor;
class vtkPolyData;
class vtkRenderer;

/**
 * @file tpp_widget_defaults.h
 * @brief Valeurs initiales, constantes et outil de rendu partages par les fichiers sources de TPPWidget
 */

namespace noether
{
/** @brief Diametre de la brosse a l'ouverture, en millimetres */
constexpr double kDefaultBrushDiameterMm = 30.0;

/** @brief Rayon d'outil a l'ouverture, en millimetres : le disque de 125 mm de la cellule */
constexpr double kDefaultToolRadiusMm = 62.5;

/** @brief Pas du trace manuel a l'ouverture, en millimetres */
constexpr double kDefaultTraceStepMm = 20.0;

/** @brief Hauteur d'approche et de retrait a l'ouverture, en millimetres */
constexpr double kDefaultApproachHeightMm = 50.0;

/** @brief Angle de planeite de l'outil Region a l'ouverture, en degres */
constexpr double kDefaultFlatnessAngleDeg = 20.0;

/**
 * @brief Nombre minimal de poses d'une passe livree
 * @details Une pose d'approche, une de retrait et au moins deux de contact, comme le decoupage
 * d'une passe par la planification ; une passe plus courte y est refusee.
 */
constexpr std::size_t kMinPassPoses = 4;

/** @brief Surelevation des disques dessines a l'ouverture, en millimetres, pour qu'ils passent devant la piece */
constexpr double kDefaultDiscLiftMm = 2.0;

/** @brief Pas cible de l'outil Densifier a l'ouverture, en millimetres */
constexpr double kDefaultDensifyStepMm = 10.0;

/**
 * @brief Poses hors contact a chaque bout d'une passe livree : une d'approche, une de retrait.
 * @details Les memes que kMinPassPoses compte ; elles ne poncent rien et ne couvrent donc rien.
 */
constexpr std::size_t kSkirtPoses = 1;

/** @brief Couleur des disques autour des points du chemin, hors palette des surfaces (Brewer Set1) */
constexpr unsigned char kDiscRgb[3] = { 0, 235, 255 };

/** @brief Couleur de la pose saisie pour deplacement, et des poses choisies pour densifier */
constexpr unsigned char kGrabRgb[3] = { 255, 60, 220 };

/**
 * @brief Couleur des zones que le disque n'atteint pas : vert citron, hors palette des surfaces
 * @details Lisible sur le fond sombre de la vue comme sur toute couleur de surface ; distinct de
 * l'orange de la selection et du cyan des disques.
 */
constexpr unsigned char kUncoveredRgb[3] = { 170, 255, 0 };

/** @brief Opacite d'un disque plein : les disques voisins et la piece restent visibles dessous */
constexpr double kFilledDiscOpacity = 0.3;

/** @brief Segments du bord d'un disque */
constexpr int kCircleSegments = 48;

/** @brief Distance, en metres, au-dela de laquelle un clic ne saisit aucune pose */
constexpr double kGrabToleranceM = 0.05;

/**
 * @brief Remplace un acteur dans le rendu par un nouveau construit sur `poly`
 * @return Le nouvel acteur, deja ajoute au rendu ; l'ancien en est retire
 */
vtkSmartPointer<vtkActor> replaceActor(vtkRenderer& renderer, vtkSmartPointer<vtkActor> previous, vtkPolyData* poly);

}  // namespace noether
