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
#include <noether_gui/widgets/tpp_widget.h>
#include <noether_gui/plugin_interface.h>

#include <QApplication>
#include <gtest/gtest.h>

/** @brief La fenetre complete se construit : tous les panneaux et le cube sont branches sans erreur */
TEST(TPPWidget, TheWindowBuilds)
{
  noether::TPPWidget widget(std::make_shared<noether::WidgetFactory>());
  SUCCEED();
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
