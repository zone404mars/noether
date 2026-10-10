#pragma once

#include <QString>
#include <functional>
#include <vector>

class QAbstractButton;
class QAction;
class QDockWidget;
class QLabel;
class QMenu;
class QWidget;

namespace noether
{
/** @brief Langue des textes montres a l'operateur */
enum class Language
{
  FRENCH,
  ENGLISH
};

/** @brief Un texte montre a l'operateur, ecrit dans les deux langues au point ou il sert */
struct Bilingual
{
  /** @brief Texte en francais */
  QString french;
  /** @brief Texte en anglais */
  QString english;
};

/**
 * @brief Langue courante de l'interface et textes qui en dependent
 * @details Un widget lie par ::bind recoit son texte tout de suite, puis a chaque changement de
 * langue. Les messages ponctuels (barre d'etat, boites de dialogue) prennent la langue courante
 * par ::text au moment ou ils s'affichent.
 */
class Translator
{
public:
  /** @param language Langue de depart */
  explicit Translator(Language language);

  /** @brief Langue courante */
  Language language() const { return language_; }

  /** @brief Change la langue et reapplique tous les textes lies */
  void setLanguage(Language language);

  /** @brief Le texte dans la langue courante */
  QString text(const Bilingual& text) const;

  /** @brief Lie le texte d'un bouton */
  void bindText(QAbstractButton* button, const Bilingual& text);
  /** @brief Lie le texte d'une etiquette */
  void bindText(QLabel* label, const Bilingual& text);
  /** @brief Lie le texte d'une action de menu */
  void bindText(QAction* action, const Bilingual& text);
  /** @brief Lie le titre d'un menu */
  void bindText(QMenu* menu, const Bilingual& text);
  /** @brief Lie le titre d'un dock */
  void bindText(QDockWidget* dock, const Bilingual& text);
  /** @brief Lie l'infobulle d'un widget */
  void bindToolTip(QWidget* widget, const Bilingual& text);
  /** @brief Lie une mise a jour quelconque, appelee tout de suite puis a chaque changement de langue */
  void bind(const std::function<void()>& apply);

  /** @brief Langue enregistree dans les reglages de l'application, le francais si aucune */
  static Language savedLanguage();
  /** @brief Enregistre la langue dans les reglages de l'application */
  static void saveLanguage(Language language);

private:
  /** @brief Langue courante */
  Language language_;
  /** @brief Mises a jour des textes lies, rejouees a chaque changement de langue */
  std::vector<std::function<void()>> bindings_;
};

}  // namespace noether
