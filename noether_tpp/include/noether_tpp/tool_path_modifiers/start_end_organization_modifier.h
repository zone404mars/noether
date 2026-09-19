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
#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Ordonne et oriente les lignes de raster pour partir d'un point et finir a un autre.
 * @details Chaque ToolPath est une ligne de raster, faite d'un ou plusieurs segments. Remplace les
 * organisateurs en raster ou en serpent quand l'operateur a designe le
 * depart et l'arrivee d'une surface. Toutes les lignes sont conservees, donc la couverture que le
 * planificateur a produite l'est aussi : seul l'ordre et le sens de parcours changent. Pour
 * chaque ligne prise comme premiere, dans chaque sens, la suite est enchainee du plus proche au
 * plus proche ; l'organisation retenue est celle dont le premier point est le plus pres du depart
 * et le dernier le plus pres de l'arrivee, les deux distances additionnees. L'arrivee n'est pas
 * toujours atteignable exactement : avec un nombre pair de lignes un serpent finit du cote ou il
 * a commence. L'ecart restant se lit avec ::endGap.
 */
class StartEndOrganizationModifier : public OneTimeToolPathModifier
{
public:
  /**
   * @param start Point pres duquel le chemin doit commencer, dans le repere du maillage
   * @param end Point pres duquel il doit finir
   */
  StartEndOrganizationModifier(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

  ToolPaths modify(ToolPaths tool_paths) const override;

  /** @brief Distance du dernier point des lignes organisees au point d'arrivee demande, en metres */
  double endGap(const ToolPaths& tool_paths) const;

protected:
  /** @brief Point de depart demande */
  Eigen::Vector3d start_{ Eigen::Vector3d::Zero() };
  /** @brief Point d'arrivee demande */
  Eigen::Vector3d end_{ Eigen::Vector3d::Zero() };

  StartEndOrganizationModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(StartEndOrganizationModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::StartEndOrganizationModifier)
