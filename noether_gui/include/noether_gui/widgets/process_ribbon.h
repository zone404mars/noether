#pragma once

#include <noether_gui/widgets/translator.h>

#include <QWidget>
#include <functional>
#include <vector>

class QHBoxLayout;
class QStackedWidget;
class QTabBar;
class QToolButton;

namespace noether
{
/**
 * @brief Barre du haut : un onglet par etape du process, et sous l'onglet actif ses outils
 * @details Les onglets suivent l'ordre du travail de l'operateur. Chaque onglet porte une rangee
 * de boutons a icone et libelle ; changer d'onglet ne change que la rangee affichee.
 */
class ProcessRibbon : public QWidget
{
public:
  /**
   * @param translator Langue de l'interface ; doit vivre plus longtemps que la barre
   * @param parent Widget parent
   */
  ProcessRibbon(Translator& translator, QWidget* parent = nullptr);

  /**
   * @brief Ajoute un onglet et sa rangee d'outils, vide
   * @return Rang de l'onglet
   */
  int addTab(const Bilingual& label);

  /**
   * @brief Ajoute un outil a la rangee d'un onglet
   * @param tab Rang de l'onglet, rendu par ::addTab
   * @param icon Chemin de l'icone, une ressource Qt
   * @param label Libelle affiche sous l'icone
   * @param tip Infobulle
   * @return Le bouton cree, que l'appelant connecte et rend eventuellement checkable
   */
  QToolButton* addTool(int tab, const QString& icon, const Bilingual& label, const Bilingual& tip);

  /** @brief Separe visuellement deux groupes d'outils d'un onglet */
  void addSeparator(int tab);

  /** @brief Onglet affiche */
  int currentTab() const;

  /** @brief Affiche un onglet et sa rangee */
  void setCurrentTab(int tab);

  /** @brief Appele avec le rang de l'onglet quand l'operateur en change */
  std::function<void(int)> on_tab_changed;

private:
  /** @brief Langue de l'interface */
  Translator& translator_;
  /** @brief Les onglets */
  QTabBar* tab_bar_;
  /** @brief Une rangee d'outils par onglet, seule celle de l'onglet actif est visible */
  QStackedWidget* tool_rows_;
  /** @brief Disposition de chaque rangee, dans l'ordre des onglets */
  std::vector<QHBoxLayout*> row_layouts_;
};

}  // namespace noether
