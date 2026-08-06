/**
 * @file curvature_threshold_modifier_widget.cpp
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
#include <noether_gui/widgets/mesh_modifiers/curvature_threshold_modifier_widget.h>

#include <noether_tpp/mesh_modifiers/curvature_threshold_modifier.h>
#include <noether_tpp/serialization.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>

static const std::string MAX_ABS_CURVATURE_KEY = "max_abs_curvature";
static const std::string CURVATURE_TYPE_KEY = "curvature_type";

namespace noether
{
CurvatureThresholdMeshModifierWidget::CurvatureThresholdMeshModifierWidget(QWidget* parent)
  : BaseWidget(parent), max_abs_curvature_(new QDoubleSpinBox(this)), curvature_type_(new QComboBox(this))
{
  max_abs_curvature_->setDecimals(3);
  max_abs_curvature_->setMinimum(0.0);
  max_abs_curvature_->setMaximum(std::numeric_limits<double>::max());
  max_abs_curvature_->setSingleStep(1.0);
  // Default corresponds to a Ø125 mm disc tool on a mesh in meters (threshold = 2 / 0.125 = 16 m^-1)
  max_abs_curvature_->setValue(16.0);
  max_abs_curvature_->setToolTip(
      "Maximum allowed absolute curvature [1/length]. For a mesh in meters, a tool of diameter D "
      "corresponds to a threshold of 2/D (e.g. Ø125 mm -> 16).");

  // Populate the curvature-type combo box from the canonical enum values
  using CurvatureType = CurvatureThresholdMeshModifier::CurvatureType;
  for (CurvatureType type : { CurvatureType::MAX_PRINCIPAL,
                              CurvatureType::MEAN,
                              CurvatureType::MIN_PRINCIPAL,
                              CurvatureType::GAUSSIAN })
    curvature_type_->addItem(QString::fromStdString(toString(type)));
  curvature_type_->setToolTip(
      "Curvature measure to threshold against. 'max_principal' reflects the tightest radius in any "
      "direction and is the physically correct criterion for whether a rigid disc tool fits.");

  auto layout = new QFormLayout(this);
  layout->addRow(new QLabel("Max. absolute curvature [1/length]", this), max_abs_curvature_);
  layout->addRow(new QLabel("Curvature type", this), curvature_type_);
}

void CurvatureThresholdMeshModifierWidget::configure(const YAML::Node& node)
{
  max_abs_curvature_->setValue(YAML::getMember<double>(node, MAX_ABS_CURVATURE_KEY));

  // curvature_type is optional; fall back to the modifier's default (mean) if absent
  const std::string type =
      node[CURVATURE_TYPE_KEY] ? node[CURVATURE_TYPE_KEY].as<std::string>() :
                                 toString(CurvatureThresholdMeshModifier::CurvatureType::MEAN);
  const int index = curvature_type_->findText(QString::fromStdString(type));
  if (index >= 0)
    curvature_type_->setCurrentIndex(index);
}

void CurvatureThresholdMeshModifierWidget::save(YAML::Node& node) const
{
  node["name"] = "CurvatureThreshold";
  node[MAX_ABS_CURVATURE_KEY] = max_abs_curvature_->value();
  node[CURVATURE_TYPE_KEY] = curvature_type_->currentText().toStdString();
}

}  // namespace noether
