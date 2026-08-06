/**
 * @file curvature_threshold_modifier_widget.h
 * @copyright Copyright (c) 2026, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <noether_gui/widgets.h>

class QDoubleSpinBox;
class QComboBox;

namespace noether
{
/**
 * @ingroup gui_widgets_mesh_modifiers
 * @brief Widget for configuring a CurvatureThresholdMeshModifier
 */
class CurvatureThresholdMeshModifierWidget : public BaseWidget
{
public:
  CurvatureThresholdMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* max_abs_curvature_;
  QComboBox* curvature_type_;
};

}  // namespace noether
