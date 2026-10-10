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

#include <noether_gui/widgets/process_ribbon.h>
#include <noether_gui/widgets/translator.h>

#include <QAction>
#include <QApplication>
#include <QFile>
#include <QLabel>
#include <QPushButton>
#include <QSettings>
#include <QTabBar>
#include <QTemporaryDir>
#include <QToolButton>
#include <gtest/gtest.h>

namespace
{
/** @brief Texte d'essai dans les deux langues */
const noether::Bilingual kOpen{ QString::fromUtf8("Ouvrir une pièce"), "Open part" };
}  // namespace

/// Le texte rendu suit la langue choisie
TEST(Translator, PicksLanguage)
{
  noether::Translator translator(noether::Language::FRENCH);
  EXPECT_EQ(translator.text(kOpen), kOpen.french);
  translator.setLanguage(noether::Language::ENGLISH);
  EXPECT_EQ(translator.language(), noether::Language::ENGLISH);
  EXPECT_EQ(translator.text(kOpen), "Open part");
}

/// Une liaison s'applique tout de suite, puis a chaque changement de langue
TEST(Translator, ReplaysBindings)
{
  noether::Translator translator(noether::Language::ENGLISH);
  QPushButton button;
  QLabel label;
  QAction action(nullptr);
  translator.bindText(&button, kOpen);
  translator.bindText(&label, kOpen);
  translator.bindText(&action, kOpen);
  translator.bindToolTip(&button, kOpen);
  int calls = 0;
  translator.bind([&calls]() { ++calls; });
  EXPECT_EQ(button.text(), "Open part");
  EXPECT_EQ(calls, 1);

  translator.setLanguage(noether::Language::FRENCH);
  EXPECT_EQ(button.text(), kOpen.french);
  EXPECT_EQ(button.toolTip(), kOpen.french);
  EXPECT_EQ(label.text(), kOpen.french);
  EXPECT_EQ(action.text(), kOpen.french);
  EXPECT_EQ(calls, 2);
}

/// La langue gardee est relue au demarrage suivant ; le francais est la langue par defaut
TEST(Translator, SavesLanguage)
{
  QTemporaryDir dir;
  // Les reglages de l'essai vont dans un dossier temporaire, pas dans ceux de l'utilisateur
  QSettings::setPath(QSettings::NativeFormat, QSettings::UserScope, dir.path());
  QSettings().clear();
  EXPECT_EQ(noether::Translator::savedLanguage(), noether::Language::FRENCH);
  noether::Translator::saveLanguage(noether::Language::ENGLISH);
  EXPECT_EQ(noether::Translator::savedLanguage(), noether::Language::ENGLISH);
  noether::Translator::saveLanguage(noether::Language::FRENCH);
  EXPECT_EQ(noether::Translator::savedLanguage(), noether::Language::FRENCH);
}

/// Chaque onglet a sa rangee d'outils ; le changement d'onglet est signale avec son rang
TEST(ProcessRibbon, TabsAndTools)
{
  noether::Translator translator(noether::Language::FRENCH);
  noether::ProcessRibbon ribbon(translator);
  const int file_tab = ribbon.addTab({ "1  Fichier", "1  File" });
  const int view_tab = ribbon.addTab({ "Affichage", "View" });
  EXPECT_EQ(file_tab, 0);
  EXPECT_EQ(view_tab, 1);
  QToolButton* open = ribbon.addTool(file_tab, ":/icons/open.svg", kOpen, kOpen);
  ribbon.addSeparator(file_tab);
  EXPECT_EQ(open->text(), kOpen.french);
  // Un QIcon n'est pas nul meme sans fichier : c'est la ressource elle-meme qui est verifiee
  EXPECT_TRUE(QFile::exists(":/icons/open.svg"));
  EXPECT_FALSE(QFile::exists(":/icons/absent.svg"));

  int entered = -1;
  ribbon.on_tab_changed = [&entered](int tab) { entered = tab; };
  ribbon.setCurrentTab(view_tab);
  EXPECT_EQ(ribbon.currentTab(), view_tab);
  EXPECT_EQ(entered, view_tab);
  // Le bouton d'un onglet cache n'est pas visible
  ribbon.show();
  EXPECT_FALSE(open->isVisible());
  ribbon.setCurrentTab(file_tab);
  EXPECT_TRUE(open->isVisible());
}

/// Les libelles des onglets et des outils suivent la langue
TEST(ProcessRibbon, Translates)
{
  noether::Translator translator(noether::Language::FRENCH);
  noether::ProcessRibbon ribbon(translator);
  ribbon.addTab({ "1  Fichier", "1  File" });
  QToolButton* open = ribbon.addTool(0, ":/icons/open.svg", kOpen, kOpen);
  translator.setLanguage(noether::Language::ENGLISH);
  EXPECT_EQ(open->text(), "Open part");
  EXPECT_EQ(open->toolTip(), "Open part");
  EXPECT_EQ(ribbon.findChild<QTabBar*>()->tabText(0), "1  File");
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  app.setOrganizationName("noether");
  app.setApplicationName("noether_gui_translator_utest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
