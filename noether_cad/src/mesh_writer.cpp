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

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace
{
using noether::cad::CadModel;
using noether::cad::Face;
using noether::cad::FaceType;

/** @brief Liste d'entiers en sequence YAML sur une ligne, [] si vide */
std::string toYamlList(const std::vector<int>& values)
{
  std::ostringstream text;
  text << "[";
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    text << (i > 0 ? ", " : "") << values[i];
  }
  text << "]";
  return text.str();
}

/** @brief Vecteur en sequence YAML sur une ligne, six decimales */
std::string toYaml(const Eigen::Vector3d& vector)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(6);
  text << "[" << vector.x() << ", " << vector.y() << ", " << vector.z() << "]";
  return text.str();
}

/** @brief Fichier ouvert en ecriture, refuse s'il ne s'ouvre pas */
std::ofstream openForWriting(const std::string& path, const std::ios::openmode mode)
{
  std::ofstream file(path, mode);
  if (!file)
  {
    throw std::runtime_error("Ecriture impossible : " + path);
  }
  return file;
}

/** @brief Sommets puis triangles du PLY, en binaire */
void writePlyBody(std::ostream& file, const noether::cad::Mesh& mesh)
{
  // Sommets en float : c'est ce qu'attendent PCL et Noether, et 24 bits de mantisse valent
  // environ un micron sur une piece metrique
  for (const Eigen::Vector3d& vertex : mesh.vertices)
  {
    const Eigen::Vector3f coordinates = vertex.cast<float>();
    file.write(reinterpret_cast<const char*>(coordinates.data()), 3 * sizeof(float));
  }
  const unsigned char corner_count = 3;
  for (const Eigen::Vector3i& triangle : mesh.triangles)
  {
    file.write(reinterpret_cast<const char*>(&corner_count), sizeof(corner_count));
    file.write(reinterpret_cast<const char*>(triangle.data()), 3 * sizeof(int));
  }
}

/** @brief Source, unite et solides du modele */
void writeOrigin(std::ostream& file, const CadModel& model)
{
  file << "cad:\n";
  file << "  source: " << model.source << "\n";
  file << "  declared_unit: " << model.declared_unit << "\n";
  file << "  unit_forced: " << (model.unit_forced ? "true" : "false") << "\n";
  // Au-dela de un, le maillage est l'union de plusieurs pieces : les faces restent distinguables
  file << "  solid_count: " << model.solid_count << "\n";
  file << "  solid_total: " << model.solid_total << "\n";
  // Indices de readInfo ; un solide ecarte n'existe plus dans le maillage
  file << "  retained_solids: " << toYamlList(model.retained_solids) << "\n";
  file << "  excluded_solids: " << toYamlList(model.excluded_solids) << "\n\n";
}

/** @brief Transformation appliquee au maillage, champ critique de la fiche */
void writeTransform(std::ostream& file, const CadModel& model)
{
  const Eigen::Quaterniond rotation(model.T_source_output.rotation());
  file << "  # Transformation appliquee au maillage, en metres, APRES conversion d'unite.\n";
  file << "  # output_frame = T_source_output * source_frame_in_meters\n";
  file << "  # SANS ELLE, TOUT CE QUI A ETE PALPE AVANT DEVIENT FAUX EN SILENCE.\n";
  file << "  recentering: " << (model.T_source_output.translation().isZero() ? "none" : "bounding_box") << "\n";
  file << "  T_source_output:\n";
  file << "    translation: " << toYaml(model.T_source_output.translation()) << "\n";
  file << std::fixed << std::setprecision(6);
  file << "    rotation_quat: [" << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ", "
       << rotation.w() << "]\n\n";
}

/** @brief Finesse, taille du maillage et boite englobante */
void writeTessellation(std::ostream& file, const CadModel& model)
{
  const auto box = model.boundingBox();
  file << std::fixed << std::setprecision(6);
  file << "  tessellation:\n";
  file << "    linear_deflection_m: " << model.tessellation.linear_deflection << "\n";
  file << "    angular_deflection_rad: " << model.tessellation.angular_deflection << "\n";
  file << "    vertices: " << model.mesh.vertices.size() << "\n";
  file << "    triangles: " << model.mesh.triangles.size() << "\n\n";
  file << "  # Mesuree sur les TRIANGLES, apres recentrage.\n";
  file << "  bounding_box_m:\n";
  file << "    min: " << toYaml(box.first) << "\n";
  file << "    max: " << toYaml(box.second) << "\n";
  file << "    extent: " << toYaml(box.second - box.first) << "\n\n";
}

/** @brief Une entree de la table des faces ; les parametres analytiques selon le type */
void writeFace(std::ostream& file, const Face& face)
{
  file << "    - id: " << face.id << "\n";
  file << "      type: " << noether::cad::faceTypeName(face.type) << "\n";
  file << "      triangles: [" << face.triangle_begin << ", " << face.triangle_end << "]\n";
  file << "      area_m2: " << std::setprecision(8) << face.area << std::setprecision(6) << "\n";
  file << "      centroid: " << toYaml(face.centroid) << "\n";
  if (face.type == FaceType::PLANE)
  {
    file << "      normal: " << toYaml(face.normal) << "\n";
  }
  if (face.type == FaceType::CYLINDER || face.type == FaceType::CONE || face.type == FaceType::TORUS)
  {
    file << "      axis_point: " << toYaml(face.axis_point) << "\n";
    file << "      axis_direction: " << toYaml(face.axis_direction) << "\n";
  }
  if (face.type == FaceType::SPHERE)
  {
    file << "      center: " << toYaml(face.axis_point) << "\n";
  }
  if (face.radius > 0.0)
  {
    file << "      radius_m: " << face.radius << "\n";
  }
}
}  // namespace

namespace noether
{
namespace cad
{
void writePly(const std::string& path, const Mesh& mesh)
{
  std::ofstream file = openForWriting(path, std::ios::binary);
  file << "ply\n"
       << "format binary_little_endian 1.0\n"
       << "comment genere par noether_cad, sommets en metres\n"
       << "element vertex " << mesh.vertices.size() << "\n"
       << "property float x\nproperty float y\nproperty float z\n"
       << "element face " << mesh.triangles.size() << "\n"
       << "property list uchar int vertex_indices\n"
       << "end_header\n";
  writePlyBody(file, mesh);
  if (!file)
  {
    throw std::runtime_error("Ecriture interrompue : " + path);
  }
}

void writeYaml(const std::string& path, const CadModel& model)
{
  std::ofstream file = openForWriting(path, std::ios::out);
  writeOrigin(file, model);
  writeTransform(file, model);
  writeTessellation(file, model);
  file << "  # Une entree par face STEP.\n";
  file << "  # triangles : l'intervalle [debut, fin[ dans l'ordre du maillage.\n";
  file << "  faces:\n";
  for (const Face& face : model.faces)
  {
    writeFace(file, face);
  }
  if (!file)
  {
    throw std::runtime_error("Ecriture interrompue : " + path);
  }
}

}  // namespace cad
}  // namespace noether
