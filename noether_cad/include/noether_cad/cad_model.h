#pragma once

#include <Eigen/Geometry>
#include <string>
#include <utility>
#include <vector>

namespace noether
{
namespace cad
{
/** @brief Type analytique d'une face STEP */
enum class FaceType
{
  /** @brief Normale exacte constante */
  PLANE,
  /** @brief Axe et rayon exacts */
  CYLINDER,
  /** @brief Axe, demi-angle */
  CONE,
  /** @brief Centre, rayon */
  SPHERE,
  /** @brief Axe, deux rayons */
  TORUS,
  /** @brief Surface libre, aucun parametre analytique exploitable */
  BSPLINE,
  /** @brief Tout le reste : revolution, extrusion, offset */
  OTHER
};

/** @brief Nom d'un type de face tel qu'il est ecrit dans la fiche YAML */
std::string faceTypeName(FaceType type);

/**
 * @brief Une face du STEP et les triangles qui la representent
 * @details Les triangles d'une face sont consecutifs dans le maillage : la face occupe
 * l'intervalle [triangle_begin, triangle_end[.
 */
struct Face
{
  /** @brief Rang de la face dans le parcours du STEP */
  int id = -1;
  /** @brief Type analytique de la surface support */
  FaceType type = FaceType::OTHER;
  /** @brief Premier triangle de la face */
  int triangle_begin = 0;
  /** @brief Triangle qui suit le dernier de la face */
  int triangle_end = 0;
  /** @brief Aire exacte, en metres carres */
  double area = 0.0;
  /** @brief Centre de gravite de la surface exacte */
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  /** @brief Normale sortante d'un plan, nulle pour les autres types */
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  /** @brief Point de l'axe (cylindre, cone, tore) ou centre (sphere) */
  Eigen::Vector3d axis_point = Eigen::Vector3d::Zero();
  /** @brief Direction de l'axe (cylindre, cone, tore), nulle sinon */
  Eigen::Vector3d axis_direction = Eigen::Vector3d::Zero();
  /** @brief Rayon (cylindre, sphere) ou grand rayon (tore), en metres ; 0 sinon */
  double radius = 0.0;
};

/** @brief Maillage triangule, sommets en metres, sommets fusionnes entre faces voisines */
struct Mesh
{
  /** @brief Position de chaque sommet */
  std::vector<Eigen::Vector3d> vertices;
  /** @brief Trois indices de sommets par triangle, sens direct vers l'exterieur de la matiere */
  std::vector<Eigen::Vector3i> triangles;
};

/**
 * @brief Finesse de la tessellation, toujours imposee par l'appelant
 * @details Les deux deflexions n'ont pas de valeur par defaut : une finesse de maillage est un
 * choix d'ingenierie.
 */
struct TessellationSettings
{
  /** @brief Ecart maximal entre la surface exacte et ses triangles, en metres */
  double linear_deflection;
  /** @brief Ecart angulaire maximal entre deux triangles voisins, en radians */
  double angular_deflection;
};

/** @brief Ce que la conversion d'un STEP produit : maillage, table des faces et tracabilite */
struct CadModel
{
  /** @brief Chemin du STEP lu */
  std::string source;
  /** @brief Unite retenue : "mm", "cm", "m" ou "in" */
  std::string declared_unit;
  /** @brief Vrai si l'unite a ete imposee par l'appelant au lieu d'etre lue dans le fichier */
  bool unit_forced = false;
  /** @brief Nombre de solides mailles */
  int solid_count = 1;
  /** @brief Nombre de solides du fichier */
  int solid_total = 1;
  /** @brief Indices des solides mailles, dans l'ordre de parcours du fichier */
  std::vector<int> retained_solids;
  /** @brief Indices des solides ecartes : ils n'existent plus dans le maillage */
  std::vector<int> excluded_solids;
  /**
   * @brief Transformation appliquee au maillage apres mise en metres
   * @details output_frame = T_source_output * source_frame_in_meters. Sans elle, ce qui a ete
   * palpe sur la piece reelle ne peut plus etre rattache au maillage recentre.
   */
  Eigen::Isometry3d T_source_output = Eigen::Isometry3d::Identity();
  /** @brief Finesse utilisee */
  TessellationSettings tessellation{ 0.0, 0.0 };
  /** @brief Maillage produit */
  Mesh mesh;
  /** @brief Une entree par face STEP, dans l'ordre du maillage */
  std::vector<Face> faces;

  /** @brief Boite englobante mesuree sur les sommets du maillage : (min, max) */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingBox() const;
};

}  // namespace cad
}  // namespace noether
